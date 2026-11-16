//! Convert G-code from PCB CAM tools (FlatCAM, pcb2gcode) into the dialect the Cirqoid accepts.
//!
//! CAM files put Z0 on the copper surface (positive = up) and X0/Y0 on the board corner. The Cirqoid needs
//! machine coordinates (Z 0 at the top, negative going down), absolute moves only, a decimal point on every
//! number and an F on every move. Tool changes split the file into sections, because each bit has to be
//! touched off on the copper again before its part of the job runs.

use crate::gcode::{fmt, AXES};
use regex::Regex;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::LazyLock;

static WORD_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))").unwrap());
static COMMENT_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\((.*?)\)").unwrap());
static DIA_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"(?i)dia").unwrap());
static TOOL_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"(?i)tool|drill|bit").unwrap());
static SIZE_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"(?i)\d\s*mm").unwrap());

/// Per axis X, Y, Z; `None` where the axis isn't involved.
type Target = [Option<f64>; 3];
/// (min, max) per axis in machine coordinates, `None` if the axis never moved.
pub type Bounds = [Option<(f64, f64)>; 3];

/// The file uses something the Cirqoid can't do (e.g. relative moves).
#[derive(Debug, Clone, PartialEq)]
pub struct ConvertError(pub String);

impl std::fmt::Display for ConvertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

#[derive(Debug, Clone)]
pub struct Options {
    pub air_offset: f64,    // lift the whole job by this much (mm) for a dry run in the air
    pub rapid_xy_feed: f64, // feed written on G0 moves
    pub rapid_z_feed: f64,  // feed for G0 moves that only change Z
    pub max_z_feed: f64,    // any move that changes Z is capped at this
    pub max_spindle: i64,
    pub arc_tolerance: f64, // mm, largest gap between an arc and its straight chords
    pub default_feed: f64,  // G1 feed when the file never sets one
    /// The file is already in machine coordinates (Z0 = machine top), not CAM coordinates (Z0 = copper):
    /// keep its homing lines, don't add a safe-Z move before homing, don't guess where the copper is.
    pub machine_coords: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            air_offset: 0.0,
            rapid_xy_feed: 1200.0,
            rapid_z_feed: 300.0,
            max_z_feed: 300.0,
            max_spindle: 1400,
            arc_tolerance: 0.01,
            default_feed: 300.0,
            machine_coords: false,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Section {
    pub tool: String,
    pub lines: Vec<String>,
    pub note: String, // e.g. a tool-diameter comment from the CAM file
    pub moves: usize,
    pub bounds: Bounds,
}

#[derive(Debug, Clone)]
pub struct ConvertResult {
    pub sections: Vec<Section>,
    pub warnings: Vec<String>,
    pub bounds: Bounds,
}

/// How well a comment describes the tool: 3 = diameter, 2 = tool/drill change with a size in mm, 1 = tool/drill.
fn note_rank(comment: &str) -> u8 {
    if comment.starts_with("Job: ") {
        4 // written by the app's own PCB job generator
    } else if DIA_RE.is_match(comment) {
        3
    } else if TOOL_RE.is_match(comment) {
        if SIZE_RE.is_match(comment) && comment.to_lowercase().contains("change") {
            2
        } else {
            1
        }
    } else {
        0
    }
}

fn grow(bounds: &mut Bounds, axis: usize, value: f64) {
    let (lo, hi) = bounds[axis].unwrap_or((value, value));
    bounds[axis] = Some((lo.min(value), hi.max(value)));
}

fn is(g: f64, code: f64) -> bool {
    (g - code).abs() < 1e-6
}

struct Converter {
    offset: [f64; 3],
    opt: Options,
    scale: f64,          // 25.4 after G20
    motion: Option<u32>, // 0, 1, 2, 3, 81, 82, 83
    feed: Option<f64>,   // mm/min, already scaled
    cam: Target,         // current position in CAM coordinates (mm)
    machine: Target,     // last position written, machine coordinates
    arc_absolute: bool,  // G90.1
    relative: bool,      // G91: coordinates are distances; converted to absolute positions
    line_feed: Option<f64>, // F written on the current line (a G0 that names its own speed keeps it)
    retract_to_r: bool,  // G99 (default) vs G98
    cycle: HashMap<char, f64>, // modal canned-cycle words: Z, R, P, Q
    speed: Option<i64>,
    spindle_on: bool,
    warnings: Vec<(String, usize)>,
    bounds: Bounds,
    sections: Vec<Section>,
}

impl Converter {
    fn new(origin_x: f64, origin_y: f64, surface_z: f64, opt: Options) -> Self {
        let mut c = Self {
            offset: [origin_x, origin_y, surface_z + opt.air_offset],
            opt,
            scale: 1.0,
            motion: None,
            feed: None,
            cam: [None; 3],
            machine: [None; 3],
            arc_absolute: false,
            relative: false,
            line_feed: None,
            retract_to_r: true,
            cycle: HashMap::new(),
            speed: None,
            spindle_on: false,
            warnings: Vec::new(),
            bounds: [None; 3],
            sections: Vec::new(),
        };
        c.new_section("(no tool)".into());
        c
    }

    // ---------- helpers ----------
    fn warn(&mut self, message: String) {
        match self.warnings.iter_mut().find(|(m, _)| *m == message) {
            Some((_, count)) => *count += 1,
            None => self.warnings.push((message, 1)),
        }
    }

    fn section(&mut self) -> &mut Section {
        self.sections.last_mut().expect("there is always a section")
    }

    fn emit(&mut self, line: String) {
        self.section().lines.push(line);
    }

    /// Start a new section, or just rename the current one if it hasn't moved yet.
    fn new_section(&mut self, tool: String) {
        if let Some(section) = self.sections.last_mut() {
            if section.moves == 0 {
                section.tool = tool;
                return;
            }
            self.close_section();
        }
        self.sections.push(Section { tool, ..Default::default() });
        self.machine = [None; 3]; // each section runs on its own; the user may jog in between
        self.emit("G90".into());
        self.emit("G21".into());
        if !self.opt.machine_coords {
            // lift to the top first; a machine-coordinate file may not be homed yet, so it starts as written
            self.write_move(0, [None, None, Some(0.0)], true);
        }
    }

    fn close_section(&mut self) {
        if self.section().lines.last().map(String::as_str) != Some("M5") {
            self.emit("M5".into());
        }
        self.spindle_on = false;
        self.write_move(0, [None, None, Some(0.0)], true);
    }

    fn map_to_machine(&mut self, axis: usize, value: f64) -> f64 {
        let mapped = self.offset[axis] + value;
        if axis == 2 && mapped > 0.0 {
            self.warn("Z above the machine top (Z0) clamped to 0 - travel/tool-change height only".into());
            return 0.0;
        }
        mapped
    }

    /// Write one G0/G1 move. target: CAM coordinates (or machine coordinates if `machine`).
    #[allow(clippy::needless_range_loop)] // X/Y/Z arrays indexed together
    fn write_move(&mut self, kind: u32, target: Target, machine: bool) {
        let mut out: Target = [None; 3];
        for axis in 0..3 {
            if let Some(value) = target[axis] {
                out[axis] = Some(if machine { value } else { self.map_to_machine(axis, value) });
            }
        }
        let unchanged = (0..3).all(|a| match (out[a], self.machine[a]) {
            (Some(v), Some(m)) => (v - m).abs() < 1e-9,
            (Some(_), None) => false,
            (None, _) => true,
        });
        if out.iter().all(Option::is_none) || unchanged {
            return; // nothing to move
        }
        let z_changes = match (out[2], self.machine[2]) {
            (Some(z), Some(m)) => (z - m).abs() > 1e-9,
            (Some(_), None) => true,
            _ => false,
        };
        let mut feed = if kind == 0 {
            match self.line_feed.filter(|_| !machine) {
                Some(feed) => feed, // this G0 names its own speed
                None if out[0].is_none() && out[1].is_none() => self.opt.rapid_z_feed,
                None => self.opt.rapid_xy_feed,
            }
        } else {
            match self.feed {
                Some(feed) => feed,
                None => {
                    self.warn(format!("no feed set before the first G1; using F{}", fmt(self.opt.default_feed)));
                    self.feed = Some(self.opt.default_feed);
                    self.opt.default_feed
                }
            }
        };
        if z_changes {
            feed = feed.min(self.opt.max_z_feed);
        }
        let words: Vec<String> =
            (0..3).filter_map(|a| out[a].map(|v| format!("{}{}", AXES[a], fmt(v)))).collect();
        self.emit(format!("G{kind} {} F{}", words.join(" "), fmt(feed)));
        for axis in 0..3 {
            if let Some(value) = out[axis] {
                self.machine[axis] = Some(value);
                grow(&mut self.bounds, axis, value);
                grow(&mut self.section().bounds, axis, value);
            }
        }
        if !machine && (out[0].is_some() || out[1].is_some()) {
            self.section().moves += 1; // Z-only travel alone doesn't make a section worth running
        }
    }

    #[allow(clippy::needless_range_loop)]
    fn move_to(&mut self, kind: u32, target: Target) {
        let z = target[2].or(self.cam[2]);
        // Only in CAM coordinates is "below Z0" known to be inside the copper.
        if kind == 1 && !self.opt.machine_coords && z.is_some_and(|z| z < 0.0) && !self.spindle_on {
            // cutting below the copper surface
            self.warn("cut below the surface with the spindle off; spindle started automatically".into());
            let speed = self.speed.filter(|s| *s != 0).unwrap_or(1000.min(self.opt.max_spindle));
            self.speed = Some(speed);
            self.spindle_on = true;
            self.emit(format!("M3 S{speed}"));
        }
        self.write_move(kind, target, false);
        for axis in 0..3 {
            if target[axis].is_some() {
                self.cam[axis] = target[axis];
            }
        }
    }

    // ---------- arcs ----------
    fn arc(&mut self, clockwise: bool, target: Target, words: &HashMap<char, f64>, number: usize)
        -> Result<(), ConvertError> {
        let (Some(sx), Some(sy)) = (self.cam[0], self.cam[1]) else {
            return Err(ConvertError(format!("line {number}: arc before the X/Y position is known")));
        };
        let (ex, ey) = (target[0].unwrap_or(sx), target[1].unwrap_or(sy));
        let (cx, cy) = if let Some(r) = words.get(&'R') {
            center_from_radius(sx, sy, ex, ey, r * self.scale, clockwise, number)?
        } else {
            let i = words.get(&'I').copied().unwrap_or(0.0) * self.scale;
            let j = words.get(&'J').copied().unwrap_or(0.0) * self.scale;
            if self.arc_absolute { (i, j) } else { (sx + i, sy + j) }
        };
        let radius = (sx - cx).hypot(sy - cy);
        let a0 = (sy - cy).atan2(sx - cx);
        let a1 = (ey - cy).atan2(ex - cx);
        let mut sweep = (if clockwise { a0 - a1 } else { a1 - a0 }).rem_euclid(2.0 * PI);
        if sweep < 1e-9 {
            sweep = 2.0 * PI; // same start and end point: full circle
        }
        let segments = if radius > self.opt.arc_tolerance {
            let step = 2.0 * (1.0 - self.opt.arc_tolerance / radius).max(-1.0).acos();
            ((sweep / step).ceil() as usize).max(1)
        } else {
            1
        };
        let z0 = self.cam[2];
        let z1 = target[2].or(z0);
        let direction = if clockwise { -1.0 } else { 1.0 };
        for k in 1..=segments {
            let mut point: Target = if k == segments {
                [Some(ex), Some(ey), None]
            } else {
                let angle = a0 + direction * sweep * k as f64 / segments as f64;
                [Some(cx + radius * angle.cos()), Some(cy + radius * angle.sin()), None]
            };
            if let (Some(_), Some(z0), Some(z1)) = (target[2], z0, z1) {
                point[2] = Some(z0 + (z1 - z0) * k as f64 / segments as f64);
            }
            self.move_to(1, point);
        }
        if target[2].is_some() {
            self.cam[2] = z1;
        }
        Ok(())
    }

    // ---------- canned drill cycles ----------
    fn drill(&mut self, target: Target, number: usize) -> Result<(), ConvertError> {
        let (Some(&z_bottom), Some(&r_plane)) = (self.cycle.get(&'Z'), self.cycle.get(&'R')) else {
            return Err(ConvertError(format!("line {number}: drill cycle without Z or R")));
        };
        let z_only = |z: f64| -> Target { [None, None, Some(z)] };
        // The F on a drill-cycle line is the drilling speed, not a speed for the rapid moves around the hole.
        self.line_feed = None;
        let start_z = self.cam[2];
        if start_z.is_some_and(|z| z < r_plane) {
            self.move_to(0, z_only(r_plane));
        }
        if target[0].is_some() || target[1].is_some() {
            self.move_to(0, [target[0], target[1], None]);
        }
        self.move_to(0, z_only(r_plane));
        let peck = self.cycle.get(&'Q').copied().filter(|q| *q != 0.0);
        if let (Some(83), Some(q)) = (self.motion, peck) {
            let mut depth = r_plane;
            while depth > z_bottom + 1e-9 {
                depth = (depth - q).max(z_bottom);
                self.move_to(1, z_only(depth));
                self.move_to(0, z_only(r_plane));
                if depth > z_bottom + 1e-9 {
                    self.move_to(0, z_only(r_plane.min(depth + 0.1)));
                }
            }
        } else {
            self.move_to(1, z_only(z_bottom));
            if self.motion == Some(82) && self.cycle.get(&'P').is_some_and(|p| *p != 0.0) {
                self.warn("drill dwell (G82 P) skipped - dwell isn't verified on this machine".into());
            }
            if self.motion == Some(83) {
                self.warn("G83 without Q drilled in one go".into());
            }
        }
        let retract = match start_z {
            Some(z) if !self.retract_to_r => z.max(r_plane),
            _ => r_plane,
        };
        self.move_to(0, z_only(retract));
        Ok(())
    }

    // ---------- main loop ----------
    fn line(&mut self, raw: &str, number: usize) -> Result<(), ConvertError> {
        for c in COMMENT_RE.captures_iter(raw) {
            // remember the tool size for the section picker; the most specific comment wins
            let comment = c[1].trim().to_string();
            if note_rank(&comment) > note_rank(&self.section().note) {
                self.section().note = comment;
            }
        }
        let stripped = COMMENT_RE.replace_all(raw, "");
        let text = stripped.split(';').next().unwrap_or("").split('*').next().unwrap_or("").trim();
        if text.is_empty() || text == "%" {
            return Ok(());
        }
        let words_list: Vec<(char, f64)> = WORD_RE
            .captures_iter(text)
            .filter_map(|c| Some((c[1].chars().next()?.to_ascii_uppercase(), c[2].parse().ok()?)))
            .collect();
        let gcodes: Vec<f64> =
            words_list.iter().filter(|(l, _)| *l == 'G').map(|(_, v)| (v * 10.0).round() / 10.0).collect();
        let mcodes: Vec<i64> = words_list.iter().filter(|(l, _)| *l == 'M').map(|(_, v)| *v as i64).collect();
        let words: HashMap<char, f64> =
            words_list.iter().filter(|(l, _)| !"GMNO".contains(*l)).copied().collect();

        let mut homing = false;
        for &g in &gcodes {
            if is(g, 20.0) {
                self.scale = 25.4;
            } else if is(g, 21.0) {
                self.scale = 1.0;
            } else if is(g, 90.0) {
                self.relative = false;
            } else if is(g, 91.0) {
                self.relative = true; // the Cirqoid has no G91: distances are turned into absolute positions
            } else if is(g, 90.1) {
                self.arc_absolute = true;
            } else if is(g, 91.1) {
                self.arc_absolute = false;
            } else if is(g, 98.0) {
                self.retract_to_r = false;
            } else if is(g, 99.0) {
                self.retract_to_r = true;
            } else if [0.0, 1.0, 2.0, 3.0, 81.0, 82.0, 83.0].iter().any(|c| is(g, *c)) {
                self.motion = Some(g as u32);
            } else if is(g, 80.0) {
                self.motion = None;
            } else if is(g, 4.0) {
                self.warn("dwell (G4) skipped - not verified on this machine".into());
            } else if is(g, 28.0) {
                if self.opt.machine_coords {
                    homing = true; // kept: machine-coordinate files home the machine themselves
                } else {
                    self.warn("homing (G28) inside the file skipped - home with the app's Home button".into());
                }
            } else if [17.0, 40.0, 49.0, 54.0, 61.0, 64.0, 94.0].iter().any(|c| is(g, *c)) {
                // modal settings that don't change anything here
            } else {
                return Err(ConvertError(format!("line {number}: G{} isn't supported", fmt(g).replace(".0", ""))));
            }
        }

        if let Some(t) = words.get(&'T') {
            self.new_section(format!("T{}", *t as i64));
        }
        if let Some(s) = words.get(&'S') {
            let mut speed = s.round() as i64;
            if speed > self.opt.max_spindle {
                self.warn(format!("spindle speed S{speed} limited to S{}", self.opt.max_spindle));
                speed = self.opt.max_spindle;
            }
            self.speed = Some(speed);
            if self.spindle_on && !mcodes.iter().any(|m| *m == 3 || *m == 4) {
                self.emit(format!("M3 S{speed}"));
            }
        }
        self.line_feed = words.get(&'F').map(|f| f * self.scale);
        if let Some(feed) = self.line_feed {
            self.feed = Some(feed);
        }
        if mcodes.contains(&3) || mcodes.contains(&4) {
            if mcodes.contains(&4) {
                self.warn("M4 (counter-clockwise spindle) changed to M3".into());
            }
            let speed = match self.speed.filter(|s| *s != 0) {
                Some(speed) => speed,
                None => {
                    self.warn("spindle started without a speed; using S1000".into());
                    1000.min(self.opt.max_spindle)
                }
            };
            self.speed = Some(speed);
            self.spindle_on = true;
            self.emit(format!("M3 S{speed}"));
        }

        if homing {
            // G28 X/Y/Z words are the positions the machine takes after homing, not a move
            let mut home = [0.0; 3];
            let mut line = "G28".to_string();
            for (axis, value) in home.iter_mut().enumerate() {
                if let Some(v) = words.get(&AXES[axis]) {
                    *value = v * self.scale;
                    line.push_str(&format!(" {}{}", AXES[axis], fmt(*value)));
                }
            }
            self.emit(line);
            self.cam = home.map(Some);
            self.machine = [None; 3]; // the next move is always written in full
            return Ok(());
        }

        let mut target: Target = [None; 3];
        for (axis, slot) in target.iter_mut().enumerate() {
            if let Some(v) = words.get(&AXES[axis]) {
                let v = v * self.scale;
                *slot = Some(match (self.relative, self.cam[axis]) {
                    (false, _) => v,
                    (true, Some(base)) => base + v,
                    (true, None) => {
                        return Err(ConvertError(format!(
                            "line {number}: relative move (G91) before the {} position is known - the file \
                             needs a homing line or an absolute move first", AXES[axis])));
                    }
                });
            }
        }
        let has_target = target.iter().any(Option::is_some);
        match self.motion {
            Some(81..=83) if self.relative => {
                return Err(ConvertError(format!("line {number}: drill cycles in relative mode (G91) aren't \
                                                 supported - export absolute coordinates")));
            }
            Some(81..=83) => {
                for key in ['Z', 'R', 'Q'] {
                    if let Some(v) = words.get(&key) {
                        self.cycle.insert(key, v * self.scale);
                    }
                }
                if let Some(p) = words.get(&'P') {
                    self.cycle.insert('P', *p);
                }
                if target[0].is_some() || target[1].is_some() {
                    self.drill(target, number)?;
                }
            }
            _ if !has_target => {}
            None => return Err(ConvertError(format!("line {number}: move without G0/G1"))),
            Some(m @ (2 | 3)) => self.arc(m == 2, target, &words, number)?,
            Some(m) => self.move_to(m, target),
        }

        if mcodes.contains(&5) {
            self.spindle_on = false;
            self.emit("M5".into());
        }
        if mcodes.contains(&6) && self.section().moves > 0 {
            let tool = self.section().tool.clone();
            self.new_section(tool);
        }
        if (mcodes.contains(&0) || mcodes.contains(&1)) && self.section().moves > 0 {
            let tool = format!("{} (after pause)", self.section().tool);
            self.new_section(tool);
        }
        for m in mcodes {
            if !matches!(m, 0..=9 | 30) {
                self.warn(format!("M{m} skipped")); // 7-9 (coolant): the Cirqoid has none
            }
        }
        Ok(())
    }

    fn finish(mut self) -> ConvertResult {
        self.close_section();
        let sections = self.sections.into_iter().filter(|s| s.moves > 0).collect();
        let warnings = self
            .warnings
            .into_iter()
            .map(|(m, n)| if n > 1 { format!("{m} (x{n})") } else { m })
            .collect();
        ConvertResult { sections, warnings, bounds: self.bounds }
    }
}

fn center_from_radius(sx: f64, sy: f64, ex: f64, ey: f64, r: f64, clockwise: bool, number: usize)
    -> Result<(f64, f64), ConvertError> {
    let chord = (ex - sx).hypot(ey - sy);
    if chord < 1e-9 || r.abs() < chord / 2.0 - 1e-9 {
        return Err(ConvertError(format!("line {number}: arc radius R{} can't reach its end point", fmt(r))));
    }
    let (mx, my) = ((sx + ex) / 2.0, (sy + ey) / 2.0);
    let h = (r * r - (chord / 2.0).powi(2)).max(0.0).sqrt();
    let (ux, uy) = (-(ey - sy) / chord, (ex - sx) / chord); // unit normal, left of the chord
    // Clockwise with a positive R puts the centre right of the chord; a negative R means the long way round.
    let mut side = if clockwise { -1.0 } else { 1.0 };
    if r < 0.0 {
        side = -side;
    }
    Ok((mx + side * h * ux, my + side * h * uy))
}

/// Convert a CAM G-code program. origin/surface are machine coordinates of the board corner and copper top.
pub fn convert(text: &str, origin_x: f64, origin_y: f64, surface_z: f64, options: Options)
    -> Result<ConvertResult, ConvertError> {
    let mut converter = Converter::new(origin_x, origin_y, surface_z, options);
    for (index, raw) in text.lines().enumerate() {
        converter.line(raw, index + 1)?;
    }
    let result = converter.finish();
    if result.sections.is_empty() {
        return Err(ConvertError("no moves found in the file".into()));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::firmware::CIRQOID;
    use crate::gcode::decimal_words;
    use crate::tracker::PositionTracker;

    const FLATCAM: &str = include_str!("../tests/samples/flatcam_isolation.nc");
    const PCB2GCODE: &str = include_str!("../tests/samples/pcb2gcode_drill.ngc");
    const ORIGIN: (f64, f64, f64) = (20.0, 30.0, -20.0);
    static MOVE_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"^G([01]) (.*) F(-?\d+\.\d+)$").unwrap());
    static XYZ_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"([XYZ])(-?\d+\.\d+)").unwrap());

    /// Converted moves as (kind, [x, y, z], feed).
    fn moves(lines: &[String]) -> Vec<(u32, Target, f64)> {
        lines
            .iter()
            .filter_map(|line| {
                let c = MOVE_RE.captures(line)?;
                let mut w: Target = [None; 3];
                for m in XYZ_RE.captures_iter(&c[2]) {
                    w[crate::gcode::axis_index(m[1].chars().next().unwrap())] = m[2].parse().ok();
                }
                Some((c[1].parse().unwrap(), w, c[3].parse().unwrap()))
            })
            .collect()
    }

    fn sample(text: &str) -> ConvertResult {
        convert(text, ORIGIN.0, ORIGIN.1, ORIGIN.2, Options::default()).unwrap()
    }

    fn simple(text: &str) -> ConvertResult {
        convert(text, 0.0, 0.0, -20.0, Options::default()).unwrap()
    }

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn words_without_spaces_and_origin_surface_mapping() {
        let r = sample("G21\nG90\nG00 X10.0000Y10.0000\nG01 F100\nG01 Z-0.0500\n");
        let lines = &r.sections[0].lines;
        assert!(lines.contains(&"G0 X30.0 Y40.0 F1200.0".to_string()));
        assert!(lines.contains(&"G1 Z-20.05 F100.0".to_string()));
    }

    #[test]
    fn every_number_has_a_decimal_point_and_every_move_a_feed() {
        let bare = Regex::new(r"[XYZF]-?\d+(\D|$)").unwrap();
        for section in sample(FLATCAM).sections {
            for line in section.lines {
                for c in bare.captures_iter(&line) {
                    assert!(c.get(1).is_some_and(|m| m.as_str() == "."), "{line}");
                }
                if line.starts_with("G0 ") || line.starts_with("G1 ") {
                    assert!(line.contains(" F"), "{line}");
                }
            }
        }
    }

    #[test]
    fn z_above_machine_top_is_clamped_with_warning() {
        let r = sample(FLATCAM); // Z15 tool-change height maps to -5: fine
        assert!(r.sections.iter().flat_map(|s| moves(&s.lines)).all(|(_, w, _)| w[2].unwrap_or(-1.0) <= 0.0));
        let high = simple("G0 Z30\nG1 F100 X1 Z-0.1\n");
        assert!(high.warnings.iter().any(|w| w.contains("clamped")));
        assert!(high.sections[0].lines.contains(&"G0 Z0.0 F300.0".to_string()));
    }

    #[test]
    fn air_offset_lifts_cutting_depth() {
        let cut = |r: ConvertResult| moves(&r.sections[0].lines).into_iter().rfind(|m| m.0 == 1).unwrap().1[2];
        assert!(approx(cut(simple("G1 F100 X1 Z-0.1\n")).unwrap(), -20.1));
        let air = convert("G1 F100 X1 Z-0.1\n", 0.0, 0.0, -20.0, Options { air_offset: 5.0, ..Default::default() });
        assert!(approx(cut(air.unwrap()).unwrap(), -15.1));
    }

    #[test]
    fn inches_converted_to_mm() {
        let r = simple("G20\nG90\nG1 F10 X1 Y0.5\n");
        let last = moves(&r.sections[0].lines).into_iter().rfind(|m| m.0 == 1).unwrap();
        assert_eq!(last, (1, [Some(25.4), Some(12.7), None], 254.0));
    }

    #[test]
    fn z_moves_are_capped_at_z_feed() {
        let m = moves(&simple("G1 F1000 X5\nG1 Z-0.2\nG0 Z5\nG0 X0 Y0\n").sections[0].lines);
        assert!(m.contains(&(1, [Some(5.0), None, None], 1000.0)));
        assert!(m.contains(&(1, [None, None, Some(-20.2)], 300.0)));
        assert!(m.contains(&(0, [Some(0.0), Some(0.0), None], 1200.0)));
    }

    #[test]
    fn arc_is_linearized_within_tolerance_and_ends_exactly() {
        let r = sample(FLATCAM);
        let lines = &r.sections[0].lines;
        assert!(!lines.iter().any(|l| l.starts_with("G2 ") || l.starts_with("G3 ")));
        // Arc from (30,20) to (20,30) around centre (20,20), r = 10, in CAM coordinates.
        let (cx, cy) = (20.0 + ORIGIN.0, 20.0 + ORIGIN.1);
        let pts: Vec<(f64, f64)> = moves(lines)
            .into_iter()
            .filter_map(|(_, w, _)| Some((w[0]?, w[1]?)))
            .collect();
        let start = pts.iter().position(|p| *p == (50.0, 50.0)).unwrap();
        let end = pts.iter().position(|p| *p == (40.0, 60.0)).unwrap();
        let arc = &pts[start..=end];
        assert!(arc.len() > 5);
        for p in arc {
            assert!(((p.0 - cx).hypot(p.1 - cy) - 10.0).abs() < 0.002);
        }
        for w in arc.windows(2) {
            // chord sagitta within the 0.01 tolerance, plus output rounding to 0.001
            let mid = ((w[0].0 + w[1].0) / 2.0, (w[0].1 + w[1].1) / 2.0);
            assert!(10.0 - (mid.0 - cx).hypot(mid.1 - cy) <= 0.011);
        }
    }

    #[test]
    fn arc_with_radius_word_matches_ij_arc() {
        let ij = simple("G0 X10 Y0\nG1 F100\nG3 X0 Y10 I-10 J0\n");
        let r = simple("G0 X10 Y0\nG1 F100\nG3 X0 Y10 R10\n");
        assert_eq!(moves(&ij.sections[0].lines), moves(&r.sections[0].lines));
    }

    #[test]
    fn tool_changes_split_sections_and_drill_cycles_expand() {
        let r = sample(PCB2GCODE);
        let tools: Vec<&str> = r.sections.iter().map(|s| s.tool.as_str()).collect();
        assert_eq!(tools, ["T1", "T2"]);
        assert!(r.sections[0].note.contains("0.8"));
        let t1 = moves(&r.sections[0].lines);
        let holes: Vec<Target> = t1.iter().filter(|m| m.1[2].is_none() && m.1[0].is_some()).map(|m| m.1).collect();
        assert_eq!(holes, [[Some(25.0), Some(35.0), None], [Some(30.0), Some(35.0), None]]);
        assert!(t1.contains(&(1, [None, None, Some(-21.8)], 150.0))); // drilled to Z-1.8 below the surface
        // G83 with Q0.6 from R1 to Z-1.8: pecks at 0.4, -0.2, -0.8, -1.4, -1.8
        let depths: Vec<f64> = moves(&r.sections[1].lines).iter().filter(|m| m.0 == 1).map(|m| m.1[2].unwrap()).collect();
        let want = [-19.6, -20.2, -20.8, -21.4, -21.8];
        assert_eq!(depths.len(), want.len());
        assert!(depths.iter().zip(want).all(|(a, b)| approx(*a, b)), "{depths:?}");
        assert_eq!(r.sections[1].lines[..3], ["G90", "G21", "G0 Z0.0 F300.0"]);
        let n = r.sections[1].lines.len();
        assert_eq!(r.sections[1].lines[n - 2..], ["M5", "G0 Z0.0 F300.0"]);
    }

    #[test]
    fn spindle_speed_is_clamped() {
        let r = sample(PCB2GCODE);
        assert!(r.sections[0].lines.contains(&"M3 S1400".to_string()));
        assert!(r.warnings.iter().any(|w| w.contains("S10000 limited to S1400")));
    }

    #[test]
    fn section_note_prefers_the_tool_diameter() {
        assert_eq!(sample(FLATCAM).sections[0].note, "Tool Diameter: 0.1 mm");
    }

    #[test]
    fn relative_moves_become_absolute() {
        let m = moves(&simple("G0 X10 Y0\nG91\nG1 X5 F100\nG1 Y-2\nG90\nG1 X0\n").sections[0].lines);
        assert!(m.contains(&(1, [Some(15.0), None, None], 100.0)));
        assert!(m.contains(&(1, [None, Some(-2.0), None], 100.0)));
        assert!(m.contains(&(1, [Some(0.0), None, None], 100.0)));
        // a distance with no known starting point can't be placed
        let err = convert("G91\nG1 X1 F100\n", 0.0, 0.0, -20.0, Options::default()).unwrap_err();
        assert!(err.0.contains("relative move (G91) before the X position is known"), "{}", err.0);
    }

    #[test]
    fn unknown_codes_are_refused() {
        let err = convert("G53 G0 X1\n", 0.0, 0.0, -20.0, Options::default()).unwrap_err();
        assert!(err.0.contains("G53"));
    }

    fn machine(text: &str) -> ConvertResult {
        convert(text, 0.0, 0.0, 0.0, Options { machine_coords: true, ..Default::default() }).unwrap()
    }

    #[test]
    fn machine_coordinate_files_keep_their_homing_and_positions() {
        let r = machine("G90\nG21\nG28 Y0.835\nG0 Z-5 F300\nG0 X5 Y5 F1200\nG1 X10 Z-6 F100\n");
        let lines = &r.sections[0].lines;
        assert_eq!(lines[..3], ["G90", "G21", "G28 Y0.835"], "no safe-Z move before homing");
        assert!(lines.contains(&"G0 Z-5.0 F300.0".to_string()));
        assert!(lines.contains(&"G1 X10.0 Z-6.0 F100.0".to_string()));
        assert!(!lines.iter().any(|l| l.starts_with("M3")), "no guessing that Z<0 is inside the copper");
        // relative moves after homing start from the homed position
        let r = machine("G28 Y0.835\nG91\nG0 Y2 F500\n");
        assert!(r.sections[0].lines.contains(&"G0 Y2.835 F500.0".to_string()));
    }

    #[test]
    fn a_g0_keeps_its_own_speed() {
        let lines = &simple("G0 X5 F800\nG0 X6\n").sections[0].lines;
        assert!(lines.contains(&"G0 X5.0 F800.0".to_string()));
        assert!(lines.contains(&"G0 X6.0 F1200.0".to_string()), "without its own F a G0 is a rapid");
    }

    #[test]
    fn output_passes_the_apps_soft_limits() {
        for text in [FLATCAM, PCB2GCODE] {
            for section in sample(text).sections {
                let mut tracker = PositionTracker::new(CIRQOID.soft_limits);
                tracker.apply("G28 Y0.835").unwrap();
                for line in &section.lines {
                    assert_eq!(&decimal_words(line), line); // already in the Cirqoid dialect
                    tracker.apply(line).unwrap_or_else(|e| panic!("{line}: {e}"));
                }
            }
        }
    }

    #[test]
    fn out_of_range_origin_is_caught_by_soft_limits() {
        let far = convert(FLATCAM, 90.0, 30.0, -20.0, Options::default()).unwrap();
        let mut tracker = PositionTracker::new(CIRQOID.soft_limits);
        tracker.apply("G28 Y0.835").unwrap();
        let errors: Vec<String> = far.sections[0].lines.iter().filter_map(|l| tracker.apply(l).err()).collect();
        assert!(errors.iter().any(|e| e.contains("X would go to")));
    }
}

#[cfg(test)]
mod edge_tests {
    use super::*;

    fn cam(text: &str) -> ConvertResult {
        convert(text, 0.0, 0.0, -20.0, Options::default()).unwrap()
    }

    fn machine(text: &str) -> ConvertResult {
        convert(text, 0.0, 0.0, 0.0, Options { machine_coords: true, ..Default::default() }).unwrap()
    }

    fn err(text: &str) -> String {
        convert(text, 0.0, 0.0, -20.0, Options::default()).unwrap_err().0
    }

    fn lines(r: &ConvertResult) -> Vec<String> {
        r.sections.iter().flat_map(|s| s.lines.clone()).collect()
    }

    fn has(r: &ConvertResult, line: &str) -> bool {
        lines(r).iter().any(|l| l == line)
    }

    fn warned(r: &ConvertResult, text: &str) -> bool {
        r.warnings.iter().any(|w| w.contains(text))
    }

    /// (x, y) of every move, machine coordinates.
    fn xy(r: &ConvertResult) -> Vec<(f64, f64)> {
        let re = Regex::new(r"X(-?[\d.]+) Y(-?[\d.]+)").unwrap();
        lines(r).iter().filter_map(|l| re.captures(l).map(|c| (c[1].parse().unwrap(), c[2].parse().unwrap()))).collect()
    }

    #[test]
    fn drill_cycle_retract_modes() {
        // G99 (default): back to R; G98: back to where Z was before the cycle
        let g99 = cam("G0 Z5\nG81 R1 Z-1 F100 X1 Y1\n");
        assert!(lines(&g99).iter().rev().skip(2).find(|l| l.starts_with("G0 Z")).unwrap() == "G0 Z-19.0 F300.0");
        assert!(has(&g99, "G0 X1.0 Y1.0 F1200.0"), "the move to the hole is a rapid, not at the drill feed");
        let g98 = cam("G0 Z5\nG98\nG81 R1 Z-1 F100 X1 Y1\n");
        assert!(has(&g98, "G0 Z-15.0 F300.0"), "{:?}", lines(&g98));
    }

    #[test]
    fn drill_cycle_warnings_and_errors() {
        assert!(warned(&cam("G0 Z2\nG82 R1 Z-1 P0.5 F100 X1 Y1\n"), "dwell (G82 P) skipped"));
        assert!(warned(&cam("G0 Z2\nG83 R1 Z-1 F100 X1 Y1\n"), "G83 without Q"));
        assert!(err("G81 Z-1 F100 X1 Y1\n").contains("drill cycle without Z or R"));
        assert!(err("G0 X0 Y0 Z2\nG91\nG81 R1 Z-1 F100 X1 Y1\n").contains("drill cycles in relative mode"));
    }

    #[test]
    fn spindle_edge_cases() {
        let r = cam("M4 S800\nG0 X1 Y1\n");
        assert!(has(&r, "M3 S800") && warned(&r, "M4 (counter-clockwise spindle) changed to M3"));
        let r = cam("M3\nG0 X1 Y1\n");
        assert!(has(&r, "M3 S1000") && warned(&r, "without a speed"));
        let r = cam("M3 S500\nG0 X1 Y1\nS900\n");
        assert!(has(&r, "M3 S900"), "a new S while running changes the speed");
    }

    #[test]
    fn skipped_codes() {
        let r = cam("G0 X1 Y1\nG4 P1\nM8\nM9\nM42\n");
        assert!(warned(&r, "dwell (G4) skipped"));
        assert!(warned(&r, "M42 skipped"));
        assert!(!warned(&r, "M8"), "coolant is dropped without a warning");
        assert!(warned(&cam("G28\nG0 X1 Y1\n"), "homing (G28) inside the file skipped"));
    }

    #[test]
    fn line_numbers_checksums_and_percent_lines() {
        let r = cam("%\nN10 G0 X1 Y1*45\nN20 G1 X2 F100 ; comment\n%\n");
        assert!(has(&r, "G0 X1.0 Y1.0 F1200.0") && has(&r, "G1 X2.0 F100.0"), "{:?}", lines(&r));
    }

    #[test]
    fn inch_mode_can_switch_back() {
        let r = cam("G20\nG0 X1 Y0\nG21\nG0 X1 Y0\n");
        assert!(has(&r, "G0 X25.4 Y0.0 F1200.0") && has(&r, "G0 X1.0 Y0.0 F1200.0"), "{:?}", lines(&r));
    }

    #[test]
    fn absolute_arc_centres_g90_1() {
        let rel = cam("G0 X10 Y0\nG1 F100\nG3 X0 Y10 I-10 J0\n");
        let abs = cam("G0 X10 Y0\nG90.1\nG1 F100\nG3 X0 Y10 I0 J0\n");
        assert_eq!(xy(&rel), xy(&abs));
    }

    #[test]
    fn negative_radius_takes_the_long_way_round() {
        let short = cam("G0 X10 Y0\nG1 F100\nG3 X0 Y10 R10\n");
        let long = cam("G0 X10 Y0\nG1 F100\nG3 X0 Y10 R-10\n");
        assert!(xy(&long).len() > 2 * xy(&short).len(), "270° needs more chords than 90°");
        // R-10 from (10,0) to (0,10) goes round the centre (10,10), out to x = 20 and y = 20
        assert!(xy(&long).iter().any(|(x, _)| *x > 19.9) && xy(&long).iter().any(|(_, y)| *y > 19.9));
        assert!(xy(&short).iter().all(|(x, y)| *x <= 10.0 && *y <= 10.0), "the short way stays inside");
    }

    #[test]
    fn relative_and_helical_arcs() {
        let r = cam("G0 X0 Y0\nG91\nG1 F100\nG2 X10 Y0 I5 J0\n");
        assert_eq!(*xy(&r).last().unwrap(), (10.0, 0.0), "relative end point");
        let r = cam("G0 X10 Y0 Z0\nG1 F100\nG3 X10 Y0 I-10 J0 Z-1\n");
        let zs: Vec<f64> = lines(&r).iter().filter_map(|l| l.split(" Z").nth(1)?.split(' ').next()?.parse().ok()).collect();
        assert!((zs.iter().cloned().fold(f64::MAX, f64::min) + 21.0).abs() < 1e-6, "helix ends 1 mm lower");
    }

    #[test]
    fn arc_errors() {
        assert!(err("G1 F100\nG2 X1 Y1 I1 J0\n").contains("arc before the X/Y position is known"));
        assert!(err("G0 X0 Y0\nG1 F100\nG2 X10 Y0 R1\n").contains("can't reach its end point"));
    }

    #[test]
    fn files_without_moves_are_refused() {
        assert_eq!(err("G90\nG21\nM5\n"), "no moves found in the file");
        assert!(err("G1 X1 Y1\nG80\nX2\n").contains("move without G0/G1"));
    }

    #[test]
    fn missing_feed_uses_the_default_with_a_warning() {
        let r = cam("G0 X1 Y1\nG1 X2\n");
        assert!(has(&r, "G1 X2.0 F300.0") && warned(&r, "no feed set"));
    }

    #[test]
    fn a_pause_splits_the_job_and_empty_parts_are_dropped() {
        let r = cam("G0 Z5\nT1\nM6\nG0 X1 Y1\nM0\nG0 X2 Y2\nT2\nT3\nG0 X3 Y3\n");
        let tools: Vec<&str> = r.sections.iter().map(|s| s.tool.as_str()).collect();
        assert_eq!(tools, ["T1", "T1 (after pause)", "T3"], "T2 had no moves of its own");
    }

    #[test]
    fn machine_files_keep_a_bare_g28_and_take_the_air_lift() {
        let r = machine("G28\nG0 X5 Y5 Z-3 F500\n");
        assert_eq!(r.sections[0].lines[2], "G28");
        let air = convert("G28\nG0 X5 Y5 Z-3 F500\n", 0.0, 0.0, 0.0, Options { machine_coords: true, air_offset: 2.0, ..Default::default() }).unwrap();
        assert!(air.sections[0].lines.contains(&"G0 X5.0 Y5.0 Z-1.0 F300.0".to_string()), "{:?}", air.sections[0].lines);
    }

    #[test]
    fn job_comments_win_over_tool_size_comments() {
        let r = cam("(Tool dia 0.8 mm)\n(Job: Drill 0.8 mm)\nG0 X1 Y1\n");
        assert_eq!(r.sections[0].note, "Job: Drill 0.8 mm");
    }

    #[test]
    fn bounds_cover_all_moves() {
        let r = convert("G0 X1 Y2\nG1 X-3 Y4 F100 Z-1\n", 10.0, 20.0, -5.0, Options::default()).unwrap();
        assert_eq!(r.bounds[0], Some((7.0, 11.0)));
        assert_eq!(r.bounds[1], Some((22.0, 24.0)));
        assert_eq!(r.bounds[2], Some((-6.0, 0.0)), "from the safe Z0 at the start down to the cut");
    }
}
