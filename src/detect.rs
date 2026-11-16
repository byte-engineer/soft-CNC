//! Auto-detect: try baud rates on one port and recognise the firmware from its replies (in a thread).

use crate::link::{open_port, Event, Events, LogTag, Reader};
use regex::Regex;
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock};
use std::thread;
use std::time::{Duration, Instant};

/// Baud rates to try, most common first.
const DETECT_BAUDS: [u32; 10] = [115200, 250000, 57600, 38400, 19200, 9600, 230400, 74880, 500000, 1000000];
/// Info queries only, never motion.
const DETECT_QUERIES: [&[u8]; 6] = [b"\r\n", b"$I\r\n", b"?", b"\r\n", b"M115\r\n", b"M114\r\n"];
const FINE_QUERIES: [&[u8]; 2] = [b"\r\n", b"M115\r\n"];
/// Percent, for controllers whose clock is slightly off.
const FINE_OFFSETS: [f64; 4] = [-4.0, -2.0, 2.0, 4.0];
pub const SILENT_SAMPLE: &str = "(no reply at this rate, complaints at the others)";

static SIGNATURES: LazyLock<Vec<(&str, Regex)>> = LazyLock::new(|| {
    vec![
        ("GRBL", Regex::new(r"Grbl|\[VER:|<(Idle|Alarm|Run|Hold|Jog|Home)").unwrap()),
        ("Marlin", Regex::new(r"FIRMWARE_NAME|Marlin|echo:").unwrap()),
        ("Cirqoid", Regex::new(r"(?m)^n?ack\b").unwrap()),
        ("Generic", Regex::new(r"(?m)^ok\b|X:-?\d").unwrap()),
    ]
});

#[derive(Debug, Clone)]
pub struct Detected {
    pub baud: u32,
    pub firmware: Option<&'static str>,
    pub sample: String,
}

fn readable(data: &[u8]) -> bool {
    let printable = data.iter().filter(|&&b| (32..127).contains(&b) || matches!(b, 9 | 10 | 13)).count();
    data.len() >= 3 && printable as f64 / data.len() as f64 >= 0.9
}

/// Read everything the port sends for the given time.
fn collect(reader: &mut Reader, time: Duration) -> Vec<u8> {
    let mut data = Vec::new();
    let mut chunk = [0u8; 256];
    let end = Instant::now() + time;
    while Instant::now() < end {
        match reader.read(&mut chunk) {
            Ok(n) => data.extend_from_slice(&chunk[..n]),
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(_) => break,
        }
    }
    data
}

/// Opens the port at a baud rate, sends the queries and returns everything that came back.
/// Replaceable so the detection logic can be tested without hardware.
type Probe = Box<dyn FnMut(u32, Duration, &[&[u8]]) -> io::Result<Vec<u8>> + Send>;

struct Detector {
    probe: Probe,
    events: Events,
    cancel: Arc<AtomicBool>,
    silent: Vec<u32>,
    garbled: usize,
}

impl Detector {
    fn log(&self, text: String, tag: LogTag) {
        self.events.send(Event::Log(text, tag));
    }

    /// The first recognised firmware, else the first readable baud.
    fn try_bauds(&mut self, bauds: &[u32], settle: Duration, queries: &[&[u8]]) -> io::Result<Option<Detected>> {
        let mut fallback = None;
        for &baud in bauds {
            if self.cancel.load(Ordering::SeqCst) {
                self.log("Auto-detect cancelled".into(), LogTag::Info);
                return Ok(fallback);
            }
            let data = (self.probe)(baud, settle, queries)?;
            let text = String::from_utf8_lossy(&data).to_string();
            let sample: String = text.lines().map(str::trim).filter(|l| !l.is_empty()).collect::<Vec<_>>()
                .join(" | ").chars().take(80).collect();
            if data.is_empty() {
                self.log(format!("  {baud:>7}: no reply"), LogTag::Rx);
                self.silent.push(baud);
            } else if !readable(&data) {
                self.log(format!("  {baud:>7}: garbled ({} bytes) - wrong rate", data.len()), LogTag::Rx);
                self.garbled += 1;
            } else if let Some((name, _)) = SIGNATURES.iter().find(|(_, re)| re.is_match(&text)) {
                self.log(format!("  {baud:>7}: {name} detected  \"{sample}\""), LogTag::Info);
                return Ok(Some(Detected { baud, firmware: Some(name), sample }));
            } else {
                self.log(format!("  {baud:>7}: readable, firmware not recognised  \"{sample}\""), LogTag::Err);
                fallback = fallback.or(Some(Detected { baud, firmware: None, sample }));
            }
        }
        Ok(fallback)
    }

    fn run(&mut self) -> io::Result<Option<Detected>> {
        self.log("Trying standard baud rates...".into(), LogTag::Info);
        let found = self.try_bauds(&DETECT_BAUDS, Duration::from_secs(2), &DETECT_QUERIES)?;
        if found.is_some() || self.cancel.load(Ordering::SeqCst) {
            return Ok(found);
        }
        if let (Some(&baud), true) = (self.silent.first(), self.garbled > 0) {
            // Alive (it answered the garbage we sent at wrong rates) but quiet at this rate:
            // it accepted the queries without replying, which is how a Cirqoid behaves.
            return Ok(Some(Detected { baud, firmware: Some("Cirqoid"), sample: SILENT_SAMPLE.into() }));
        }
        self.log("Nothing readable; trying rates a few % off the standard ones...".into(), LogTag::Info);
        let fine: Vec<u32> = DETECT_BAUDS
            .iter()
            .filter(|&&b| b <= 250000)
            .flat_map(|&b| FINE_OFFSETS.iter().map(move |p| (b as f64 * (1.0 + p / 100.0)).round() as u32))
            .collect();
        self.try_bauds(&fine, Duration::from_millis(500), &FINE_QUERIES)
    }
}

/// The real probe: a serial port (or simulator) opened at each baud rate in turn.
fn serial_probe(port: String, cancel: Arc<AtomicBool>) -> Probe {
    Box::new(move |baud, settle, queries| {
        let (mut reader, mut writer) = open_port(&port, baud)?;
        let mut data = collect(&mut reader, settle); // startup banner (many boards reset when the port opens)
        for query in queries {
            if cancel.load(Ordering::SeqCst) {
                break;
            }
            writer.write_all(query)?;
            writer.flush()?;
            data.extend(collect(&mut reader, Duration::from_millis(500)));
        }
        Ok(data)
    })
}

/// Detection itself, in the calling thread.
fn detect(probe: Probe, events: &Events, cancel: Arc<AtomicBool>) -> Option<Detected> {
    let mut detector = Detector { probe, events: events.clone(), cancel, silent: Vec::new(), garbled: 0 };
    match detector.run() {
        Ok(found) => found,
        Err(err) => {
            detector.log(format!("Auto-detect stopped: {err}"), LogTag::Err);
            None
        }
    }
}

/// Run auto-detect in a thread; the result arrives as `Event::DetectDone`.
pub fn spawn(port: String, events: Events, cancel: Arc<AtomicBool>) {
    thread::spawn(move || {
        let found = detect(serial_probe(port, cancel.clone()), &events, cancel);
        events.send(Event::DetectDone(found));
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::link::Events;

    /// A fake port: `answer(baud)` is what the controller sends back at that baud rate.
    fn run_with(answer: impl Fn(u32) -> Vec<u8> + Send + 'static) -> (Option<Detected>, Vec<String>) {
        let (events, rx) = Events::new(None);
        let found = detect(Box::new(move |baud, _, _| Ok(answer(baud))), &events, Arc::new(AtomicBool::new(false)));
        let log = rx.try_iter().filter_map(|e| if let Event::Log(text, _) = e { Some(text) } else { None }).collect();
        (found, log)
    }

    const GARBAGE: &[u8] = &[0xe0, 0xfc, 0x1c, 0x00, 0x8e, 0xfe, 0x80, 0x1c];

    #[test]
    fn readable_needs_mostly_printable_text() {
        assert!(readable(b"ok\r\n"));
        assert!(!readable(b"ok")); // too short to judge
        assert!(!readable(GARBAGE));
    }

    #[test]
    fn grbl_is_recognised_at_its_baud_rate() {
        let (found, log) = run_with(|b| if b == 115200 { b"Grbl 1.1h ['$' for help]\r\nok\r\n".to_vec() } else { GARBAGE.to_vec() });
        let found = found.unwrap();
        assert_eq!((found.baud, found.firmware), (115200, Some("GRBL")));
        assert!(log.iter().any(|l| l.contains("GRBL detected")));
    }

    #[test]
    fn marlin_and_generic_ok() {
        let (found, _) = run_with(|b| if b == 250000 { b"FIRMWARE_NAME:Marlin\r\nok\r\n".to_vec() } else { Vec::new() });
        assert_eq!(found.unwrap().firmware, Some("Marlin"));
        let (found, _) = run_with(|b| if b == 57600 { b"ok\r\nok\r\n".to_vec() } else { GARBAGE.to_vec() });
        assert_eq!(found.unwrap().firmware, Some("Generic"));
    }

    #[test]
    fn a_silent_controller_that_complains_elsewhere_is_a_cirqoid() {
        // quiet at 38400, garbage at every other rate: it accepted the queries without replying
        let (found, _) = run_with(|b| if b == 38400 { Vec::new() } else { GARBAGE.to_vec() });
        let found = found.unwrap();
        assert_eq!((found.baud, found.firmware), (38400, Some("Cirqoid")));
        assert_eq!(found.sample, SILENT_SAMPLE);
    }

    #[test]
    fn readable_nack_at_an_odd_rate_is_found_in_the_fine_pass() {
        // a controller with a slow clock: only readable 2 % below 38400
        let (found, log) = run_with(|b| if b == 37632 { b"nack: communication error\r\n".to_vec() } else { GARBAGE.to_vec() });
        let found = found.unwrap();
        assert_eq!((found.baud, found.firmware), (37632, Some("Cirqoid")));
        assert!(log.iter().any(|l| l.contains("trying rates a few % off")));
    }

    #[test]
    fn readable_but_unknown_firmware_falls_back_to_the_baud_rate() {
        let (found, log) = run_with(|b| if b == 19200 { b"HELLO MACHINE v2\r\n".to_vec() } else { GARBAGE.to_vec() });
        let found = found.unwrap();
        assert_eq!((found.baud, found.firmware), (19200, None));
        assert!(log.iter().any(|l| l.contains("firmware not recognised")));
    }

    #[test]
    fn nothing_at_all_finds_nothing() {
        let (found, _) = run_with(|_| Vec::new());
        assert!(found.is_none(), "silence everywhere is a dead port, not a Cirqoid");
    }

    #[test]
    fn a_port_error_stops_detection_with_a_message() {
        let (events, rx) = Events::new(None);
        let probe: Probe = Box::new(|_, _, _| Err(io::Error::other("Access is denied")));
        assert!(detect(probe, &events, Arc::new(AtomicBool::new(false))).is_none());
        assert!(rx.try_iter().any(|e| matches!(e, Event::Log(t, LogTag::Err) if t.contains("Access is denied"))));
    }

    #[test]
    fn cancel_stops_before_the_next_rate() {
        let (events, _rx) = Events::new(None);
        let cancel = Arc::new(AtomicBool::new(true));
        let probe: Probe = Box::new(|_, _, _| panic!("no port should be opened after cancel"));
        assert!(detect(probe, &events, cancel).is_none());
    }

    #[test]
    fn the_grbl_simulator_is_detected_end_to_end() {
        let (events, rx) = Events::new(None);
        let cancel = Arc::new(AtomicBool::new(false));
        spawn(crate::sim::SIM_GRBL.into(), events, cancel);
        let found = rx.iter().find_map(|e| if let Event::DetectDone(f) = e { Some(f) } else { None }).unwrap();
        assert_eq!(found.unwrap().firmware, Some("GRBL"));
    }
}
