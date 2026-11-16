//! Everything the app does with the machine, without any UI: connection, jogging, homing, jobs,
//! soft limits, spindle, CAM import. The egui window (`app.rs`) only draws this state and calls these methods.

use crate::cam::{self, ConvertResult, Options};
use crate::detect::{self, Detected, SILENT_SAMPLE};
use crate::firmware::{by_name, Command, Firmware, SPINDLE_MAX, SPINDLE_STEP};
use crate::gcode::{clean_gcode, decimal_words, fmt, looks_like_cam, short, AXES};
use crate::link::{Event, Events, Link, LinkConfig, LogTag, Waker};
use crate::pcb;
use crate::tracker::PositionTracker;
use regex::Regex;
use std::collections::{BTreeMap, HashSet};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, LazyLock};
use std::time::{Duration, Instant};

const MAX_LOG_LINES: usize = 3000;
static MOTION_LINE_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"(?i)\bG0?[0-3]\b|\bG28\b|^[XYZ]").unwrap());
static MARLIN_POS_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"X:(-?[\d.]+)\s*Y:(-?[\d.]+)\s*Z:(-?[\d.]+)").unwrap());
static M3_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\bM0?3\b").unwrap());
static M5_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\bM0?5\b").unwrap());
static S_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"S\s*(\d+)").unwrap());

pub struct LogLine {
    pub text: String,
    pub tag: LogTag,
    pub ok: bool, // the machine answered "ok" to this command line
}

/// One job of a loaded program (one tool), as shown in the Jobs list.
#[derive(Debug, Clone)]
pub struct JobEntry {
    pub title: String,
    /// What to do before running it (flip the board, change the bit…).
    pub hint: String,
}

/// The loaded file, kept so each job can be converted again with the current board position and surface Z.
#[derive(Debug, Clone)]
pub struct Program {
    text: String,
    pub name: String,
    cam_style: bool,
    pub jobs: Vec<JobEntry>,
    /// The job that Start runs.
    pub loaded: Option<usize>,
}

/// Name of a converted section for the Jobs list: the PCB job title, or tool + comment from a CAM file.
fn section_title(section: &cam::Section) -> String {
    if let Some(job) = section.note.strip_prefix("Job: ") {
        return job.to_string();
    }
    match (section.tool.as_str(), section.note.is_empty()) {
        ("(no tool)", true) => String::new(),
        ("(no tool)", false) => section.note.clone(),
        (tool, true) => tool.to_string(),
        (tool, false) => format!("{tool} · {}", section.note),
    }
}

pub struct Controller {
    events: Events,
    rx: Receiver<Event>,
    link: Option<Link>,
    detect_cancel: Option<Arc<AtomicBool>>,
    pending: BTreeMap<u64, (Option<PositionTracker>, String)>, // tag -> (position before the line, line)
    sent: HashSet<u64>,                                        // tags already written to the port
    next_tag: u64,
    last_poll: Instant,
    history: Vec<String>,
    history_pos: usize,

    pub fw: &'static Firmware,
    pub port: String,
    pub baud: String,
    pub log: Vec<LogLine>,
    pub last_msg: Option<(String, LogTag)>,
    /// Position from what is queued: used for soft limits and the next jog target.
    pub tracker: Option<PositionTracker>,
    /// Position from what has actually been sent: shown in the readout.
    pub display: Option<PositionTracker>,
    /// Position reported by GRBL/Marlin: (x, y, z, note).
    pub reported: Option<([String; 3], String)>,
    pub job_lines: Vec<String>,
    pub job_total: usize,
    pub job_done: usize,
    pub file_label: String,
    /// How the last loaded file was read (CAM or machine coordinates), for the log.
    how_read: String,
    /// The loaded file and its jobs (Cirqoid only; other firmwares get the file as written).
    pub program: Option<Program>,
    pub spindle_on: bool,
    pub spindle_text: String,
    // user inputs
    pub step: String,
    pub feed: String,
    pub speed: i64,
    pub origin_x: String,
    pub origin_y: String,
    pub surface: String,
    pub air_test: bool,
    pub air_offset: String,
}

impl Controller {
    pub fn new(waker: Option<Waker>) -> Self {
        let (events, rx) = Events::new(waker);
        Self {
            events,
            rx,
            link: None,
            detect_cancel: None,
            pending: BTreeMap::new(),
            sent: HashSet::new(),
            next_tag: 0,
            last_poll: Instant::now(),
            history: Vec::new(),
            history_pos: 0,
            fw: &crate::firmware::CIRQOID,
            port: String::new(),
            baud: crate::firmware::CIRQOID.default_baud.to_string(),
            log: Vec::new(),
            last_msg: None,
            tracker: None,
            display: None,
            reported: None,
            job_lines: Vec::new(),
            job_total: 0,
            job_done: 0,
            file_label: "no file loaded".into(),
            how_read: String::new(),
            program: None,
            spindle_on: false,
            spindle_text: "off".into(),
            step: "1".into(),
            feed: "500".into(),
            speed: 1000,
            origin_x: String::new(),
            origin_y: String::new(),
            surface: String::new(),
            air_test: false, // tick for a dry run: the whole job is lifted by the offset
            air_offset: "5".into(),
        }
    }

    // ---------- state ----------
    pub fn connected(&self) -> bool {
        self.link.is_some()
    }

    pub fn detecting(&self) -> bool {
        self.detect_cancel.is_some()
    }

    pub fn job_running(&self) -> bool {
        self.job_total > 0 && self.job_done < self.job_total
    }

    pub fn paused(&self) -> bool {
        self.link.as_ref().is_some_and(Link::is_paused)
    }

    /// Position readout: (x, y, z) texts and a short note.
    pub fn readout(&self) -> ([String; 3], String) {
        let dashes = || ["—".to_string(), "—".to_string(), "—".to_string()];
        if !self.connected() {
            return (dashes(), "disconnected".into());
        }
        // the readout shows what has been sent, not what is queued
        if let Some(source) = self.display.as_ref().or(self.tracker.as_ref()) {
            return match source.work() {
                Some(w) => (w.map(|v| format!("{v:.3}")), "homed · position as sent".into()),
                None => (dashes(), "not homed — press Home".into()),
            };
        }
        self.reported.clone().unwrap_or_else(|| (dashes(), "connected".into()))
    }

    // ---------- log ----------
    pub fn log(&mut self, text: impl Into<String>, tag: LogTag) {
        let text = text.into();
        if matches!(tag, LogTag::Info | LogTag::Err) {
            self.last_msg = Some((text.trim().to_string(), tag)); // also shown on the PCB job tab
        }
        self.log.push(LogLine { text, tag, ok: false });
        if self.log.len() > MAX_LOG_LINES {
            self.log.drain(..500);
        }
    }

    /// Show an "ok" at the end of the command line it answers, instead of on a line of its own.
    fn log_ok(&mut self) {
        match self.log.last_mut() {
            Some(last) if last.tag == LogTag::Tx && !last.ok => last.ok = true,
            _ => self.log("ok", LogTag::Ok),
        }
    }

    pub fn clear_log(&mut self) {
        self.log.clear();
    }

    // ---------- connection ----------
    pub fn set_firmware(&mut self, name: &str) {
        self.fw = by_name(name);
        self.baud = self.fw.default_baud.to_string();
    }

    pub fn connect(&mut self) -> Result<(), String> {
        let baud: u32 = self.baud.trim().parse().map_err(|_| format!("'{}' is not a baud rate", self.baud))?;
        let config = LinkConfig {
            silent_wait: self.fw.silent_wait,
            reply_expected: self.fw.replies_to_moves.then(|| MOTION_LINE_RE.clone()),
        };
        let link = Link::open(&self.port, baud, self.events.clone(), config).map_err(|e| e.to_string())?;
        self.link = Some(link);
        self.pending.clear();
        self.sent.clear();
        self.reported = None;
        self.tracker = self.fw.absolute_only.then(|| PositionTracker::new(self.fw.soft_limits));
        self.display = self.fw.absolute_only.then(PositionTracker::default); // follows lines as they go out
        self.set_spindle(false, None);
        self.log(format!("Connected to {} @ {baud} ({})", self.port, self.fw.name), LogTag::Info);
        if self.fw.absolute_only {
            self.log("Cirqoid: it doesn't reply to accepted setting lines, so silence means OK and only an", LogTag::Info);
            self.log("'error'/'nack' is a refusal. Press Home (G90, G21, G28 Y0.835) before moving.", LogTag::Info);
        }
        if let Some(limits) = self.fw.soft_limits {
            let text: Vec<String> =
                (0..3).map(|a| format!("{} {}..{}", AXES[a], short(limits[a].0), short(limits[a].1))).collect();
            self.log(format!("Soft limits (mm from home): {}", text.join(", ")), LogTag::Info);
        }
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.link = None; // dropping the link stops its threads and closes the port
        self.job_total = 0;
        self.job_done = 0;
        self.tracker = None;
        self.display = None;
        self.reported = None;
        self.pending.clear();
        self.sent.clear();
        self.set_spindle(false, None);
        self.log("Disconnected", LogTag::Info);
    }

    // ---------- auto-detect ----------
    pub fn auto_detect(&mut self) {
        if let Some(cancel) = &self.detect_cancel {
            cancel.store(true, Ordering::SeqCst);
            return;
        }
        if self.connected() {
            self.disconnect();
        }
        self.log(format!("Auto-detect on {} (info queries only, no motion). Takes up to ~2 minutes.", self.port),
                 LogTag::Info);
        let cancel = Arc::new(AtomicBool::new(false));
        detect::spawn(self.port.clone(), self.events.clone(), cancel.clone());
        self.detect_cancel = Some(cancel);
    }

    fn detect_done(&mut self, found: Option<Detected>) {
        self.detect_cancel = None;
        let Some(found) = found else {
            self.log("Auto-detect: no working setting found. Check the cable/power, or the controller may not \
                      use a serial text protocol.", LogTag::Err);
            return;
        };
        let baud = found.baud;
        match found.firmware {
            Some(name) => {
                self.fw = by_name(name);
                self.log(format!("Auto-detect: {name} at {baud} baud. Settings applied - click Connect."), LogTag::Info);
                if found.sample == SILENT_SAMPLE {
                    self.log(format!("The controller accepts commands at {baud} baud without replying, as a \
                                      Cirqoid does."), LogTag::Info);
                }
                if found.sample.contains("communication error") {
                    self.log("Warning: the controller answers 'nack: communication error' to every byte. Unplug \
                              the USB cable, power-cycle the machine, plug it back in and try again.", LogTag::Err);
                }
            }
            None => {
                self.fw = by_name("Generic");
                let hint = if found.sample.to_lowercase().contains("nack") {
                    " It rejects the commands (nack), so it probably needs its own protocol."
                } else {
                    ""
                };
                self.log(format!("Auto-detect: the controller talks at {baud} baud, but its firmware isn't one this \
                                  app knows.{hint} Baud applied, firmware set to Generic."), LogTag::Err);
            }
        }
        self.baud = baud.to_string();
    }

    // ---------- sending ----------
    fn dialect(&self, line: &str) -> String {
        if self.fw.decimal_words { decimal_words(line) } else { line.to_string() }
    }

    /// Send one line; returns false if it was refused (soft limit / unknown position).
    pub fn send_line(&mut self, line: &str) -> bool {
        if self.link.is_none() || line.trim().is_empty() {
            return false;
        }
        let line = self.dialect(line.trim());
        let snapshot = self.tracker.clone();
        if let Some(tracker) = &mut self.tracker {
            if let Err(error) = tracker.apply(&line) {
                self.log(format!("Not sent: {line}  ({error})"), LogTag::Err);
                return false;
            }
        }
        self.enqueue(&line, snapshot, false);
        true
    }

    fn enqueue(&mut self, line: &str, snapshot: Option<PositionTracker>, job: bool) {
        let tag = self.next_tag;
        self.next_tag += 1;
        self.pending.insert(tag, (snapshot, line.to_string()));
        if let Some(link) = &self.link {
            link.send(line, job, tag);
        }
    }

    /// A line has actually been written to the machine: update spindle state and the position readout.
    fn on_sent(&mut self, line: &str, tag: u64) {
        self.sent.insert(tag);
        self.note_spindle(line);
        if let Some(display) = &mut self.display {
            let _ = display.apply(line);
        }
    }

    /// The machine answered (or the wait ended). On a refusal, put the tracked position back.
    fn line_done(&mut self, tag: u64, reply: Option<String>, job: bool) {
        let (snapshot, line) = self.pending.remove(&tag).unwrap_or((None, String::new()));
        self.sent.remove(&tag);
        let refused = reply.as_deref().is_some_and(|r| {
            let r = r.to_lowercase();
            r.starts_with("error") || r.starts_with("nack")
        });
        if refused && job && self.job_running() {
            if let Some(link) = &self.link {
                link.clear();
            }
            self.discard_unsent(false);
            self.job_total = 0;
            self.job_done = 0;
            self.log(format!("Job stopped: the machine refused '{line}'"), LogTag::Err);
        }
        match (refused, &self.tracker, snapshot) {
            (true, Some(_), Some(snapshot)) => {
                let mut tracker = snapshot.clone();
                for (later_snapshot, later_line) in self.pending.values_mut() {
                    // replay what is still queued
                    *later_snapshot = Some(tracker.clone());
                    let _ = tracker.apply(later_line);
                }
                self.tracker = Some(tracker);
                if self.display.is_some() {
                    // redo the display from before the refused line, plus what went out after it
                    let mut display = snapshot;
                    display.limits = None;
                    for (later, (_, later_line)) in &self.pending {
                        if self.sent.contains(later) {
                            let _ = display.apply(later_line);
                        }
                    }
                    self.display = Some(display);
                }
                self.log(format!("Position kept as before '{line}' (the machine refused it)"), LogTag::Err);
            }
            _ if job && self.job_total > 0 => {
                self.job_done += 1;
                if self.job_done >= self.job_total {
                    self.log("Job finished", LogTag::Info);
                }
            }
            _ => {}
        }
    }

    /// After the queue is cleared: forget lines that never went out, and their effect on the position.
    fn discard_unsent(&mut self, restore: bool) {
        let unsent: Vec<u64> = self.pending.keys().filter(|t| !self.sent.contains(t)).copied().collect();
        if restore && self.tracker.is_some() {
            if let Some((Some(snapshot), _)) = unsent.first().and_then(|t| self.pending.get(t)) {
                self.tracker = Some(snapshot.clone());
            }
        }
        for tag in unsent {
            self.pending.remove(&tag);
        }
    }

    // ---------- console ----------
    pub fn send_entry(&mut self, line: &str) {
        let line = line.trim();
        if line.is_empty() {
            return;
        }
        self.history.push(line.to_string());
        self.history_pos = self.history.len();
        if matches!(line, "?" | "!" | "~") {
            // GRBL real-time characters: no newline, no "ok"
            if let Some(link) = &self.link {
                let _ = link.realtime(line.as_bytes());
            }
            self.log(line, LogTag::Tx);
        } else {
            self.send_line(line);
        }
    }

    /// Previous (-1) or next (+1) command from the history.
    pub fn history_step(&mut self, delta: i64) -> Option<String> {
        if self.history.is_empty() {
            return None;
        }
        self.history_pos = (self.history_pos as i64 + delta).clamp(0, self.history.len() as i64) as usize;
        Some(self.history.get(self.history_pos).cloned().unwrap_or_default())
    }

    // ---------- machine commands ----------
    pub fn fw_command(&mut self, name: &str, command: &'static Command) {
        match command {
            Command::None => self.log(format!("'{name}' is not available for {}", self.fw.name), LogTag::Err),
            Command::Realtime(bytes) => {
                if let Some(link) = &self.link {
                    let _ = link.realtime(bytes);
                }
                if name != "position" {
                    self.log(format!("[{name}]"), LogTag::Tx);
                }
            }
            Command::Line(line) => {
                self.send_line(line);
            }
            Command::Lines(lines) => {
                for line in lines.iter() {
                    if !self.send_line(line) {
                        break;
                    }
                }
            }
        }
    }

    pub fn home(&mut self) {
        let fw = self.fw;
        self.fw_command("home", &fw.home);
    }

    /// A real-time command (GRBL feed hold / resume): written straight away, even mid-job.
    pub fn realtime_command(&mut self, name: &str, bytes: &[u8]) {
        if let Some(link) = &self.link {
            let _ = link.realtime(bytes);
        }
        self.log(format!("[{name}]"), LogTag::Tx);
    }

    pub fn jog(&mut self, axis: usize, sign: f64) {
        let (Ok(step), Ok(feed)) = (self.step.trim().parse::<f64>(), self.feed.trim().parse::<f64>()) else {
            self.log("Step and feed must be numbers", LogTag::Err);
            return;
        };
        let step = step * sign;
        let feed = match (axis, self.fw.max_z_feed) {
            (2, Some(max)) => feed.min(max),
            _ => feed,
        };
        let name = AXES[axis];
        if self.fw.name == "GRBL" {
            self.send_line(&format!("$J=G91 G21 {name}{} F{}", short(step), short(feed)));
        } else if self.fw.absolute_only {
            let Some(work) = self.tracker.as_ref().and_then(PositionTracker::work) else {
                self.log("Position unknown: press Home first.", LogTag::Err);
                return;
            };
            let mut target = work[axis] + step;
            if let Some((lo, hi)) = self.tracker.as_ref().and_then(|t| t.work_range(axis)) {
                let clamped = target.clamp(lo, hi);
                if (clamped - work[axis]).abs() < 1e-9 {
                    self.log(format!("{name} is already at its soft limit"), LogTag::Err);
                    return;
                }
                if clamped != target {
                    self.log(format!("{name} jog shortened to stop at the soft limit"), LogTag::Info);
                }
                target = clamped;
            }
            // same shape as Cirqoid's own programs: G0 X1.0 F500.0
            self.send_line(&format!("G0 {name}{} F{}", short((target * 1000.0).round() / 1000.0), short(feed)));
        } else {
            for line in ["G91".to_string(), format!("G1 {name}{} F{}", short(step), short(feed)), "G90".into()] {
                self.send_line(&line);
            }
        }
    }

    pub fn estop(&mut self) {
        let Some(link) = &self.link else { return };
        link.clear();
        let _ = link.realtime(self.fw.estop);
        self.handle_events(); // learn about every line that did go out before the queue was cleared
        self.discard_unsent(true);
        self.set_spindle(false, None);
        self.job_total = 0;
        self.job_done = 0;
        if self.fw.name == "Cirqoid" {
            self.log("*** STOP: queue cleared, spindle off (M5). The Cirqoid has no software e-stop: moves already \
                      sent will finish - use the power switch to halt motion. ***", LogTag::Err);
        } else {
            self.log("*** E-STOP sent — queue cleared ***", LogTag::Err);
        }
    }

    // ---------- spindle ----------
    fn checked_speed(&mut self) -> i64 {
        let speed = self.speed.clamp(SPINDLE_STEP, SPINDLE_MAX);
        if speed != self.speed {
            self.log(format!("Spindle speed limited to S{speed}"), LogTag::Info);
            self.speed = speed;
        }
        speed
    }

    pub fn spindle_start(&mut self) {
        let speed = self.checked_speed();
        self.send_line(&format!("M3 S{speed}"));
    }

    pub fn spindle_set(&mut self) {
        let speed = self.checked_speed();
        if self.spindle_on {
            self.send_line(&format!("M3 S{speed}"));
        } else {
            self.log(format!("Spindle is off; S{speed} will be used when you press Start"), LogTag::Info);
        }
    }

    pub fn spindle_stop(&mut self) {
        self.send_line("M5");
    }

    fn note_spindle(&mut self, line: &str) {
        let upper = line.to_uppercase();
        if M3_RE.is_match(&upper) {
            let speed = S_RE.captures(&upper).map(|c| c[1].to_string());
            self.set_spindle(true, speed);
        } else if M5_RE.is_match(&upper) {
            self.set_spindle(false, None);
        }
    }

    fn set_spindle(&mut self, on: bool, speed: Option<String>) {
        self.spindle_on = on;
        self.spindle_text = match (on, speed) {
            (true, Some(s)) => format!("● on S{s}"),
            (true, None) => "● on".into(),
            _ => "off".into(),
        };
    }

    // ---------- files and jobs ----------
    fn file_name(path: &Path) -> String {
        path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default()
    }

    /// Load any G-code file.
    ///
    /// For the Cirqoid it is always rewritten into G-code the Cirqoid accepts (decimal numbers, a feed on
    /// every move, arcs and drill cycles as straight moves, G91 turned into absolute positions, inches into mm,
    /// one job per tool). How positions are read is decided from the file itself:
    /// - CAM G-code (FlatCAM / pcb2gcode: goes above Z0, never homes): Z0 is the copper surface, so the file is
    ///   placed at the board origin and surface Z set on the PCB job tab;
    /// - anything else is taken to be in machine coordinates already (Z0 = machine top) and keeps its positions.
    ///
    /// Other firmwares understand ordinary G-code, so their files are sent as written.
    pub fn load(&mut self, path: &Path) -> Result<(), String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("Can't read {}: {e}", path.display()))?;
        let name = Self::file_name(path);
        if !self.fw.absolute_only {
            let lines = clean_gcode(&text);
            self.file_label = format!("{name} — {} lines", lines.len());
            self.log(format!("Loaded {}: {} lines", path.display(), lines.len()), LogTag::Info);
            self.job_lines = lines;
            self.program = None;
            return Ok(());
        }
        let cam_style = looks_like_cam(&clean_gcode(&text));
        // the job list doesn't depend on where the board is, so read it with any position
        let probe = cam::convert(&text, 0.0, 0.0, 0.0, Options { machine_coords: !cam_style, ..Default::default() })
            .map_err(|e| format!("This file can't be turned into G-code the Cirqoid understands:\n\n{e}"))?;
        let jobs: Vec<JobEntry> =
            probe.sections.iter().map(|s| JobEntry { title: section_title(s), hint: String::new() }).collect();
        if jobs.len() > 1 {
            self.log(format!("{name} has {} jobs (one per tool). The first is loaded; load the next one from the \
                              Jobs list after changing the bit and setting the surface Z.", jobs.len()), LogTag::Info);
        }
        self.program = Some(Program { text, name, cam_style, jobs, loaded: None });
        self.load_job(0)
    }

    /// Jobs made from Gerber + drill files by the PCB setup (see `pcb`). They are CAM G-code, placed on the
    /// machine at the board origin and surface Z when each job is loaded.
    pub fn load_pcb_jobs(&mut self, name: String, jobs: &[pcb::toolpath::Job], notes: &[String]) {
        let text = pcb::toolpath::program(jobs);
        let entries = jobs.iter().map(|j| JobEntry { title: j.title.clone(), hint: j.hint.clone() }).collect();
        self.program = Some(Program { text, name: name.clone(), cam_style: true, jobs: entries, loaded: None });
        self.log(format!("{name}: {} jobs made from the PCB files, to run in this order:", jobs.len()), LogTag::Info);
        for (index, job) in jobs.iter().enumerate() {
            self.log(format!("  {}. {}", index + 1, job.title), LogTag::Rx);
        }
        for note in notes {
            self.log(format!("  note: {note}"), LogTag::Rx);
        }
        if self.board_position().is_err() {
            self.log("Set the board origin and surface Z (step 1), then press Load next to the first job.",
                     LogTag::Info);
        } else if let Err(err) = self.load_job(0) {
            self.log(err, LogTag::Err);
        }
    }

    fn board_position(&self) -> Result<(f64, f64, f64), String> {
        let parse = |s: &str| s.trim().parse::<f64>();
        match (parse(&self.origin_x), parse(&self.origin_y), parse(&self.surface)) {
            (Ok(x), Ok(y), Ok(z)) => Ok((x, y, z)),
            _ => Err("This job is measured from the board (CAM G-code: Z0 = copper surface), so the app needs to \
                      know where the board is.\n\nHome, jog the bit over the board corner and press 'Set origin \
                      here', then lower it onto the copper and press 'Set surface Z here'. Then load it again."
                .into()),
        }
    }

    /// Convert job `index` of the current program for the board position and surface Z set right now
    /// (they change with every bit), and make it the job that Start runs.
    pub fn load_job(&mut self, index: usize) -> Result<(), String> {
        let program = self.program.as_ref().ok_or("Load a file first")?;
        let (text, name, cam_style) = (program.text.clone(), program.name.clone(), program.cam_style);
        let air = if self.air_test {
            self.air_offset.trim().parse::<f64>().map_err(|_| "The air test lift must be a number".to_string())?
        } else {
            0.0
        };
        let (origin_x, origin_y, surface) = if cam_style { self.board_position()? } else { (0.0, 0.0, 0.0) };
        let options = Options { air_offset: air, max_spindle: SPINDLE_MAX, max_z_feed: 300.0, rapid_z_feed: 300.0,
                                machine_coords: !cam_style, ..Default::default() };
        let result = cam::convert(&text, origin_x, origin_y, surface, options)
            .map_err(|e| format!("This file can't be turned into G-code the Cirqoid understands:\n\n{e}"))?;
        if index >= result.sections.len() {
            return Err(format!("There is no job {} in {name}", index + 1));
        }
        self.how_read = if cam_style {
            format!("CAM G-code (Z0 = copper surface): placed at board origin X{} Y{}, copper at Z{}",
                    fmt(origin_x), fmt(origin_y), fmt(surface))
        } else {
            "machine G-code (Z0 = machine top): positions kept".into()
        };
        self.load_section(&result, index, &name, air);
        let hint = self.program.as_ref().and_then(|p| p.jobs.get(index)).map(|j| j.hint.clone()).unwrap_or_default();
        if let Some(program) = &mut self.program {
            program.loaded = Some(index);
        }
        if !hint.is_empty() {
            self.log(format!("  before you start: {hint}"), LogTag::Info);
        }
        Ok(())
    }

    fn load_section(&mut self, result: &ConvertResult, index: usize, name: &str, air: f64) {
        let section = &result.sections[index];
        self.job_lines = section.lines.clone();
        let count = result.sections.len();
        let job = if count > 1 { format!(" · job {} of {count}", index + 1) } else { String::new() };
        self.file_label = format!("{name}{job} — {} lines, ready for the Cirqoid", section.lines.len());
        let title = section_title(section);
        let title = if title.is_empty() { String::new() } else { format!(" ({title})") };
        self.log(format!("Loaded {name}{job}{title}: {} lines, converted for the Cirqoid", section.lines.len()),
                 LogTag::Info);
        let how = format!("  read as {}", self.how_read);
        self.log(how, LogTag::Info);
        for warning in &result.warnings {
            self.log(format!("  note: {warning}"), LogTag::Rx);
        }
        let span: Vec<String> = (0..3)
            .filter_map(|a| section.bounds[a].map(|(lo, hi)| format!("{} {}..{}", AXES[a], fmt(lo), fmt(hi))))
            .collect();
        self.log(format!("  moves span {}", span.join(", ")), LogTag::Info);
        if air > 0.0 {
            self.log(format!("  AIR TEST: the whole job is lifted {} mm. Untick 'Air test' and load it again for \
                              the real cut.", fmt(air)), LogTag::Info);
        }
    }

    fn known_position(&mut self) -> Option<[f64; 3]> {
        let work = self.tracker.as_ref().and_then(PositionTracker::work);
        if work.is_none() {
            self.log("Position unknown: connect as Cirqoid and press Home first.", LogTag::Err);
        }
        work
    }

    pub fn set_origin_here(&mut self) {
        if let Some(w) = self.known_position() {
            self.origin_x = fmt(w[0]);
            self.origin_y = fmt(w[1]);
            self.log(format!("Board origin set to X{} Y{}", fmt(w[0]), fmt(w[1])), LogTag::Info);
        }
    }

    pub fn set_surface_here(&mut self) {
        if let Some(w) = self.known_position() {
            self.surface = fmt(w[2]);
            self.log(format!("Copper surface set to Z{} (re-do this after every bit change)", fmt(w[2])), LogTag::Info);
        }
    }

    pub fn start_job(&mut self) {
        if self.link.is_none() || self.job_lines.is_empty() {
            return;
        }
        let lines: Vec<String> = self.job_lines.iter().map(|l| self.dialect(l)).collect();
        let mut snapshots = vec![None; lines.len()];
        if let Some(tracker) = &self.tracker {
            // check the whole file against the soft limits before sending anything
            let mut preview = tracker.clone();
            for (index, line) in lines.iter().enumerate() {
                snapshots[index] = Some(preview.clone());
                if let Err(error) = preview.apply(line) {
                    self.log(format!("Job not started: line {} '{line}': {error}", index + 1), LogTag::Err);
                    return;
                }
            }
            self.tracker = Some(preview);
        }
        self.job_total = lines.len();
        self.job_done = 0;
        for (line, snapshot) in lines.iter().zip(snapshots) {
            self.enqueue(line, snapshot, true);
        }
        self.log("Job started", LogTag::Info);
    }

    pub fn pause_job(&mut self) {
        let Some(link) = &self.link else { return };
        if link.is_paused() {
            link.set_paused(false);
            if let Some(resume) = self.fw.resume {
                let _ = link.realtime(resume);
            }
            self.log("Job resumed", LogTag::Info);
        } else {
            link.set_paused(true);
            if let Some(hold) = self.fw.hold {
                let _ = link.realtime(hold);
                self.log("Job paused (feed hold)", LogTag::Info);
            } else {
                self.log("Job paused (moves already buffered may still finish)", LogTag::Info);
            }
        }
    }

    pub fn stop_job(&mut self) {
        if let Some(link) = &self.link {
            link.clear();
        }
        self.handle_events(); // learn about every line that did go out before the queue was cleared
        self.discard_unsent(true);
        self.job_total = 0;
        self.job_done = 0;
        self.log("Job stopped — no more lines will be sent (use E-STOP to halt motion)", LogTag::Info);
    }

    // ---------- events ----------
    fn handle_rx(&mut self, text: String) {
        if text.starts_with('<') && text.ends_with('>') {
            // GRBL status report
            let fields: Vec<&str> = text[1..text.len() - 1].split('|').collect();
            let pos = fields.iter().find_map(|f| f.strip_prefix("MPos:").or_else(|| f.strip_prefix("WPos:")));
            let mut xyz = pos.unwrap_or("").split(',').map(String::from).chain(std::iter::repeat("?".into()));
            let values = [xyz.next().unwrap(), xyz.next().unwrap(), xyz.next().unwrap()];
            self.reported = Some((values, fields[0].to_string()));
            return;
        }
        if let Some(c) = MARLIN_POS_RE.captures(&text) {
            self.reported = Some(([c[1].into(), c[2].into(), c[3].into()], "reported by the machine".into()));
        }
        if text == "ok" {
            self.log_ok();
            return;
        }
        let lower = text.to_lowercase();
        let tag = if ["error", "alarm", "nack"].iter().any(|p| lower.starts_with(p)) { LogTag::Err } else { LogTag::Rx };
        self.log(text, tag);
    }

    /// Handle everything the worker threads reported; call this regularly (every frame).
    pub fn poll(&mut self) {
        self.handle_events();
        if self.fw.name == "GRBL" && self.last_poll.elapsed() >= Duration::from_millis(250) {
            self.last_poll = Instant::now();
            if let Some(link) = &self.link {
                if let Err(err) = link.realtime(b"?") {
                    self.log(format!("status poll failed: {err}"), LogTag::Err);
                    self.disconnect();
                }
            }
        }
    }

    fn handle_events(&mut self) {
        while let Ok(event) = self.rx.try_recv() {
            match event {
                Event::Tx { line, tag } => {
                    self.log(line.clone(), LogTag::Tx);
                    self.on_sent(&line, tag);
                }
                Event::Rx(text) => self.handle_rx(text),
                Event::Error(text) => self.log(text, LogTag::Err),
                Event::Log(text, tag) => self.log(text, tag),
                Event::LineDone { tag, reply, job } => self.line_done(tag, reply, job),
                Event::DetectDone(found) => self.detect_done(found),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detect::Detected;

    fn texts(c: &Controller) -> Vec<&str> {
        c.log.iter().map(|l| l.text.as_str()).collect()
    }

    #[test]
    fn grbl_status_reports_feed_the_readout_without_logging() {
        let mut c = Controller::new(None);
        c.link = None;
        c.handle_rx("<Run|MPos:1.000,2.500,-0.250|FS:500,0>".into());
        assert_eq!(c.reported, Some((["1.000".into(), "2.500".into(), "-0.250".into()], "Run".into())));
        assert!(c.log.is_empty(), "status reports are not logged");
        c.handle_rx("<Idle|WPos:3,4|FS:0,0>".into());
        assert_eq!(c.reported.as_ref().unwrap().0, ["3".to_string(), "4".into(), "?".into()], "missing Z shows ?");
    }

    #[test]
    fn marlin_position_reply_feeds_the_readout_and_errors_are_red() {
        let mut c = Controller::new(None);
        c.handle_rx("X:10.00 Y:20.00 Z:5.00 E:0.00 Count X:800".into());
        assert_eq!(c.reported.as_ref().unwrap().0, ["10.00".to_string(), "20.00".into(), "5.00".into()]);
        c.handle_rx("error:20".into());
        c.handle_rx("ALARM:1".into());
        c.handle_rx("nack: communication error".into());
        let tags: Vec<LogTag> = c.log.iter().skip(1).map(|l| l.tag).collect();
        assert_eq!(tags, [LogTag::Err, LogTag::Err, LogTag::Err]);
    }

    #[test]
    fn ok_is_folded_onto_the_command_it_answers() {
        let mut c = Controller::new(None);
        c.log("G0 X1.0 F500.0", LogTag::Tx);
        c.handle_rx("ok".into());
        c.handle_rx("ok".into()); // a second ok has no command left: it gets a line of its own
        assert!(c.log[0].ok);
        assert_eq!(texts(&c), ["G0 X1.0 F500.0", "ok"]);
    }

    #[test]
    fn the_log_is_trimmed_and_info_errors_become_the_last_message() {
        let mut c = Controller::new(None);
        for i in 0..MAX_LOG_LINES + 1 {
            c.log(format!("line {i}"), LogTag::Rx);
        }
        assert_eq!(c.log.len(), MAX_LOG_LINES + 1 - 500);
        assert!(c.last_msg.is_none(), "plain replies are not 'last messages'");
        c.log("  Job started  ", LogTag::Info);
        assert_eq!(c.last_msg, Some(("Job started".into(), LogTag::Info)));
    }

    #[test]
    fn detect_results_set_firmware_and_baud() {
        let mut c = Controller::new(None);
        c.detect_done(Some(Detected { baud: 115200, firmware: Some("GRBL"), sample: "Grbl 1.1h".into() }));
        assert_eq!((c.fw.name, c.baud.as_str()), ("GRBL", "115200"));
        c.detect_done(Some(Detected { baud: 38400, firmware: Some("Cirqoid"), sample: crate::detect::SILENT_SAMPLE.into() }));
        assert_eq!((c.fw.name, c.baud.as_str()), ("Cirqoid", "38400"));
        assert!(texts(&c).iter().any(|t| t.contains("without replying, as a Cirqoid does")));
        c.detect_done(Some(Detected { baud: 37632, firmware: Some("Cirqoid"), sample: "nack: communication error".into() }));
        assert!(texts(&c).iter().any(|t| t.contains("Unplug the USB cable")));
        c.detect_done(Some(Detected { baud: 19200, firmware: None, sample: "nack: what".into() }));
        assert_eq!(c.fw.name, "Generic");
        assert!(texts(&c).iter().any(|t| t.contains("probably needs its own protocol")));
        c.detect_done(None);
        assert!(texts(&c).iter().any(|t| t.contains("no working setting found")));
        assert!(!c.detecting());
    }

    #[test]
    fn readout_states() {
        let c = Controller::new(None);
        assert_eq!(c.readout().1, "disconnected");
    }

    #[test]
    fn history_steps_back_and_forward() {
        let mut c = Controller::new(None);
        assert_eq!(c.history_step(-1), None);
        c.send_entry("G0 X1");
        c.send_entry("G0 X2");
        assert_eq!(c.history_step(-1).as_deref(), Some("G0 X2"));
        assert_eq!(c.history_step(-1).as_deref(), Some("G0 X1"));
        assert_eq!(c.history_step(-1).as_deref(), Some("G0 X1"), "stays at the oldest");
        assert_eq!(c.history_step(1).as_deref(), Some("G0 X2"));
        assert_eq!(c.history_step(1).as_deref(), Some(""), "past the newest: an empty line");
    }

    #[test]
    fn titles_for_the_jobs_list() {
        let section = |tool: &str, note: &str| cam::Section { tool: tool.into(), note: note.into(), ..Default::default() };
        assert_eq!(section_title(&section("T1", "Job: Drill 0.8 mm")), "Drill 0.8 mm");
        assert_eq!(section_title(&section("T2", "Tool Diameter: 0.1 mm")), "T2 · Tool Diameter: 0.1 mm");
        assert_eq!(section_title(&section("T3", "")), "T3");
        assert_eq!(section_title(&section("(no tool)", "")), "");
        assert_eq!(section_title(&section("(no tool)", "drill 1 mm")), "drill 1 mm");
    }

    #[test]
    fn loading_needs_a_program_and_a_valid_job() {
        let mut c = Controller::new(None);
        assert_eq!(c.load_job(0).unwrap_err(), "Load a file first");
        let path = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/cirqoid_square.gcode"));
        c.load(path).unwrap();
        assert!(c.load_job(3).unwrap_err().contains("There is no job 4"));
        c.air_test = true;
        c.air_offset = "abc".into();
        assert!(c.load_job(0).unwrap_err().contains("air test lift must be a number"));
    }
}
