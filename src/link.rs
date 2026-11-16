//! The serial connection: one thread writes lines and waits for each answer, one reads replies.

use crate::firmware::{ACK_TIMEOUT, MOTION_TIMEOUT, REPLIES};
use crate::sim;
use regex::Regex;
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub type Reader = Box<dyn Read + Send>;
pub type Writer = Box<dyn Write + Send>;
/// Called from worker threads after sending an event (the UI uses it to redraw).
pub type Waker = Arc<dyn Fn() + Send + Sync>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogTag {
    Tx,
    Rx,
    Err,
    Info,
    Ok,
}

/// Sent from the worker threads to the controller.
#[derive(Debug)]
pub enum Event {
    Tx { line: String, tag: u64 },
    Rx(String),
    Error(String),
    LineDone { tag: u64, reply: Option<String>, job: bool },
    Log(String, LogTag),
    DetectDone(Option<crate::detect::Detected>),
}

/// Event sender that also wakes the UI so it redraws.
#[derive(Clone)]
pub struct Events {
    tx: Sender<Event>,
    waker: Option<Waker>,
}

impl Events {
    pub fn new(waker: Option<Waker>) -> (Self, Receiver<Event>) {
        let (tx, rx) = channel();
        (Self { tx, waker }, rx)
    }

    pub fn send(&self, event: Event) {
        let _ = self.tx.send(event);
        if let Some(wake) = &self.waker {
            wake();
        }
    }
}

/// Open a serial port (or a simulator) as separate reader and writer halves.
pub fn open_port(name: &str, baud: u32) -> io::Result<(Reader, Writer)> {
    if let Some((reader, writer)) = sim::open(name) {
        return Ok((Box::new(reader), Box::new(writer)));
    }
    let port = serialport::new(name, baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(io::Error::other)?;
    let reader = port.try_clone().map_err(io::Error::other)?;
    Ok((Box::new(reader), Box::new(port)))
}

/// Non-printable bytes are shown as \xNN so garbage from a wrong baud rate is visible.
pub fn printable(raw: &[u8]) -> String {
    raw.iter()
        .map(|&b| if b < 0x80 { (b as char).to_string() } else { format!("\\x{b:02x}") })
        .collect::<String>()
        .trim()
        .to_string()
}

pub struct LinkConfig {
    /// Controllers that stay silent when they accept a line: wait this long for a possible "nack".
    pub silent_wait: Option<Duration>,
    /// Lines that get a real answer even in silent mode (moves and homing on the Cirqoid).
    pub reply_expected: Option<Regex>,
}

struct Item {
    generation: u64,
    line: String,
    job: bool,
    tag: u64,
}

#[derive(Default)]
struct Ack {
    done: bool,
    reply: Option<String>,
}

struct Shared {
    generation: AtomicU64, // bumped by clear() so queued lines become stale
    paused: AtomicBool,
    alive: AtomicBool,
    ack: Mutex<Ack>,
    acked: Condvar,
}

/// Lines are sent one at a time, each waiting for "ok"/"error" (or, in silent mode, a short time).
pub struct Link {
    writer: Arc<Mutex<Writer>>,
    queue: Sender<Item>,
    shared: Arc<Shared>,
    threads: Vec<thread::JoinHandle<()>>,
}

impl Link {
    pub fn open(name: &str, baud: u32, events: Events, config: LinkConfig) -> io::Result<Self> {
        let (reader, writer) = open_port(name, baud)?;
        let writer = Arc::new(Mutex::new(writer));
        let shared = Arc::new(Shared {
            generation: AtomicU64::new(0),
            paused: AtomicBool::new(false),
            alive: AtomicBool::new(true),
            ack: Mutex::new(Ack::default()),
            acked: Condvar::new(),
        });
        let (queue, items) = channel();
        let reader_thread = {
            let (shared, events) = (shared.clone(), events.clone());
            thread::spawn(move || read_loop(reader, shared, events))
        };
        let writer_thread = {
            let (shared, writer) = (shared.clone(), writer.clone());
            thread::spawn(move || write_loop(items, writer, shared, events, config))
        };
        Ok(Self { writer, queue, shared, threads: vec![reader_thread, writer_thread] })
    }

    pub fn send(&self, line: &str, job: bool, tag: u64) {
        let generation = self.shared.generation.load(Ordering::SeqCst);
        let _ = self.queue.send(Item { generation, line: line.to_string(), job, tag });
    }

    pub fn realtime(&self, data: &[u8]) -> io::Result<()> {
        let mut writer = self.writer.lock().unwrap();
        writer.write_all(data)?;
        writer.flush()
    }

    /// Drop everything still waiting to be sent.
    /// Taken while a line is written, so clearing never races with it: when this returns, every line is
    /// either fully sent (its `Tx` event already queued) or will never be sent.
    pub fn clear(&self) {
        let _no_line_being_written = self.writer.lock().unwrap();
        self.shared.generation.fetch_add(1, Ordering::SeqCst);
        self.shared.paused.store(false, Ordering::SeqCst);
        self.shared.acked.notify_all();
    }

    pub fn set_paused(&self, paused: bool) {
        self.shared.paused.store(paused, Ordering::SeqCst);
    }

    pub fn is_paused(&self) -> bool {
        self.shared.paused.load(Ordering::SeqCst)
    }
}

impl Drop for Link {
    /// Stops both threads and waits for them (a fraction of a second), so the port is really closed
    /// and can be opened again straight away.
    fn drop(&mut self) {
        self.shared.alive.store(false, Ordering::SeqCst);
        self.clear();
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

fn is_refusal(reply: &str) -> bool {
    let lower = reply.to_lowercase();
    lower.starts_with("error") || lower.starts_with("nack")
}

fn write_loop(items: Receiver<Item>, writer: Arc<Mutex<Writer>>, shared: Arc<Shared>, events: Events,
              config: LinkConfig) {
    let current = || shared.generation.load(Ordering::SeqCst);
    while shared.alive.load(Ordering::SeqCst) {
        let item = match items.recv_timeout(Duration::from_millis(200)) {
            Ok(item) => item,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        while shared.paused.load(Ordering::SeqCst) && shared.alive.load(Ordering::SeqCst)
            && item.generation == current() {
            thread::sleep(Duration::from_millis(50));
        }
        if item.generation != current() || !shared.alive.load(Ordering::SeqCst) {
            continue;
        }
        *shared.ack.lock().unwrap() = Ack::default();
        {
            // Check, write and report under the same lock that clear() takes (see Link::clear).
            let mut w = writer.lock().unwrap();
            if item.generation != current() {
                continue;
            }
            if let Err(err) = w.write_all(format!("{}\n", item.line).as_bytes()).and_then(|_| w.flush()) {
                events.send(Event::Error(format!("write failed: {err}")));
                return;
            }
            events.send(Event::Tx { line: item.line.clone(), tag: item.tag });
        }

        let expects = config.silent_wait.is_none()
            || config.reply_expected.as_ref().is_some_and(|re| re.is_match(&item.line));
        let wait = match (expects, config.silent_wait) {
            (true, Some(_)) => MOTION_TIMEOUT,
            (true, None) => ACK_TIMEOUT,
            (false, silent) => silent.unwrap_or(ACK_TIMEOUT),
        };
        let deadline = Instant::now() + wait;
        let mut ack = shared.ack.lock().unwrap();
        while !ack.done {
            if !shared.alive.load(Ordering::SeqCst) || item.generation != current() {
                break;
            }
            if Instant::now() > deadline {
                if expects {
                    events.send(Event::Error(format!(
                        "no reply after {}s for: {} (check baud rate / firmware)", wait.as_secs(), item.line)));
                }
                break;
            }
            ack = shared.acked.wait_timeout(ack, Duration::from_millis(50)).unwrap().0;
        }
        let reply = if ack.done { ack.reply.clone() } else { None };
        drop(ack);
        if item.generation == current() {
            if item.job && reply.as_deref().is_some_and(is_refusal) {
                // drop the rest of the job here, before another line can go out
                shared.generation.fetch_add(1, Ordering::SeqCst);
            }
            events.send(Event::LineDone { tag: item.tag, reply, job: item.job });
        }
    }
}

fn read_loop(mut reader: Reader, shared: Arc<Shared>, events: Events) {
    let mut buf: Vec<u8> = Vec::new();
    let mut chunk = [0u8; 256];
    let emit = |raw: &[u8]| {
        let text = printable(raw);
        if text.is_empty() {
            return;
        }
        // GRBL answers "ok" / "error:N"; Marlin answers "ok"; the Cirqoid "ok" / "error: ..." / "nack: ...".
        if REPLIES.iter().any(|r| text.starts_with(r)) {
            let mut ack = shared.ack.lock().unwrap();
            ack.done = true;
            ack.reply = Some(text.clone());
            shared.acked.notify_all();
        }
        events.send(Event::Rx(text));
    };
    while shared.alive.load(Ordering::SeqCst) {
        match reader.read(&mut chunk) {
            Ok(0) => {}
            Ok(n) => {
                buf.extend_from_slice(&chunk[..n]);
                while let Some(end) = buf.iter().position(|&b| b == b'\n') {
                    let line: Vec<u8> = buf.drain(..=end).collect();
                    emit(&line[..line.len() - 1]);
                }
            }
            Err(err) if err.kind() == io::ErrorKind::TimedOut => {
                if !buf.is_empty() {
                    // data with no newline (often a baud mismatch): show it rather than hide it
                    emit(&std::mem::take(&mut buf));
                }
            }
            Err(err) => {
                if shared.alive.load(Ordering::SeqCst) {
                    events.send(Event::Error(format!("read failed: {err}")));
                }
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{SIM_CIRQOID, SIM_GRBL};

    fn open(name: &str, silent: bool) -> (Link, Receiver<Event>) {
        let (events, rx) = Events::new(None);
        let config = LinkConfig {
            silent_wait: silent.then(|| Duration::from_millis(300)),
            reply_expected: silent.then(|| Regex::new(r"\bG0?[0-3]\b|\bG28\b").unwrap()),
        };
        (Link::open(name, 38400, events, config).unwrap(), rx)
    }

    /// Collect events until `count` lines are done (or the time is up).
    fn done_lines(rx: &Receiver<Event>, count: usize, secs: u64) -> (Vec<String>, Vec<(u64, Option<String>)>) {
        let (mut sent, mut done) = (Vec::new(), Vec::new());
        let end = Instant::now() + Duration::from_secs(secs);
        while done.len() < count && Instant::now() < end {
            match rx.recv_timeout(Duration::from_millis(50)) {
                Ok(Event::Tx { line, .. }) => sent.push(line),
                Ok(Event::LineDone { tag, reply, .. }) => done.push((tag, reply)),
                _ => {}
            }
        }
        (sent, done)
    }

    #[test]
    fn printable_shows_garbage_as_hex() {
        assert_eq!(printable(b"ok\r"), "ok");
        assert_eq!(printable(&[b'n', 0xe0, b'k']), r"n\xe0k");
    }

    #[test]
    fn lines_go_out_in_order_each_waiting_for_its_ok() {
        let (link, rx) = open(SIM_GRBL, false);
        for (tag, line) in ["G90", "G0 X1", "G0 X2"].iter().enumerate() {
            link.send(line, false, tag as u64);
        }
        let (sent, done) = done_lines(&rx, 3, 5);
        assert_eq!(sent, ["G90", "G0 X1", "G0 X2"]);
        assert_eq!(done, [(0, Some("ok".into())), (1, Some("ok".into())), (2, Some("ok".into()))]);
    }

    #[test]
    fn silent_lines_pass_after_the_short_wait_and_moves_wait_for_their_answer() {
        let (link, rx) = open(SIM_CIRQOID, true);
        let start = Instant::now();
        link.send("G21", false, 1); // the Cirqoid says nothing
        link.send("G0 Z-1.0 F300.0", false, 2); // answered with ok
        link.send("G0 Z5.0 F300.0", false, 3); // refused
        let (_, done) = done_lines(&rx, 3, 5);
        assert_eq!(done, [(1, None), (2, Some("ok".into())), (3, Some("error: Coordinate out of range".into()))]);
        assert!(start.elapsed() >= Duration::from_millis(300), "the silent line waited for a possible nack");
    }

    #[test]
    fn a_refused_job_line_stops_the_rest_of_the_job() {
        let (link, rx) = open(SIM_CIRQOID, true);
        link.send("G0 Z5.0 F300.0", true, 1);
        link.send("G0 X1.0 F500.0", true, 2);
        let (sent, done) = done_lines(&rx, 2, 2);
        assert_eq!(sent, ["G0 Z5.0 F300.0"], "nothing after the refused job line");
        assert_eq!(done.len(), 1);
    }

    #[test]
    fn clear_drops_what_is_still_queued() {
        let (link, rx) = open(SIM_CIRQOID, true);
        for tag in 0..5 {
            link.send("G21", false, tag); // 300 ms each
        }
        thread::sleep(Duration::from_millis(100));
        link.clear();
        let (sent, _) = done_lines(&rx, 5, 1);
        assert_eq!(sent.len(), 1, "only the line already on its way: {sent:?}");
    }

    #[test]
    fn pause_holds_the_queue_until_resumed() {
        let (link, rx) = open(SIM_GRBL, false);
        link.set_paused(true);
        assert!(link.is_paused());
        link.send("G0 X1", false, 1);
        let (sent, _) = done_lines(&rx, 1, 1);
        assert!(sent.is_empty(), "nothing goes out while paused");
        link.set_paused(false);
        let (sent, _) = done_lines(&rx, 1, 3);
        assert_eq!(sent, ["G0 X1"]);
    }

    #[test]
    fn realtime_bytes_skip_the_queue() {
        let (link, rx) = open(SIM_GRBL, false);
        link.realtime(b"?").unwrap();
        let end = Instant::now() + Duration::from_secs(2);
        let mut status = None;
        while status.is_none() && Instant::now() < end {
            if let Ok(Event::Rx(text)) = rx.recv_timeout(Duration::from_millis(50)) {
                if text.starts_with('<') {
                    status = Some(text);
                }
            }
        }
        assert!(status.unwrap().starts_with("<Idle|MPos:0.000,0.000,0.000"));
    }

    #[test]
    fn closing_waits_for_the_threads_and_the_port_opens_again() {
        let (link, _rx) = open(SIM_GRBL, false);
        let start = Instant::now();
        drop(link);
        assert!(start.elapsed() < Duration::from_secs(1));
        let (_again, _rx2) = open(SIM_GRBL, false);
    }

    #[test]
    fn opening_a_missing_port_is_an_error() {
        let (events, _rx) = Events::new(None);
        let config = LinkConfig { silent_wait: None, reply_expected: None };
        assert!(Link::open("COM250", 38400, events, config).is_err());
    }
}
