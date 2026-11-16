//! Simulated controllers, so the app can be tried (and tested) without a machine.
//!
//! - `SIMULATOR`: a GRBL-style controller (banner, `ok` for every line, `?` status reports).
//! - `SIMULATOR-CIRQOID`: behaves like the Cirqoid: silent for setting lines, `ok` for moves,
//!   `error: Coordinate out of range` for positive Z.

use crate::gcode::axis_words;
use regex::Regex;
use std::io::{self, Read, Write};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, LazyLock, Mutex};
use std::thread::sleep;
use std::time::Duration;

pub const SIM_GRBL: &str = "SIMULATOR";
pub const SIM_CIRQOID: &str = "SIMULATOR-CIRQOID";
const BANNER: &str = "Grbl 1.1h ['$' for help] (simulator)";
static CIRQOID_REPLIES_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\bG0?[0-3]\b|\bG28\b|^[XYZ]").unwrap());

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Grbl,
    Cirqoid,
}

struct State {
    kind: Kind,
    pos: [f64; 3],
    relative: bool,
    status: &'static str,
    partial: String,
    out: Sender<Vec<u8>>,
}

impl State {
    fn emit(&self, text: &str) {
        let _ = self.out.send(format!("{text}\r\n").into_bytes());
    }

    fn line(&mut self, line: &str) {
        match self.kind {
            Kind::Grbl => self.grbl_line(line),
            Kind::Cirqoid => {
                if !CIRQOID_REPLIES_RE.is_match(line) {
                    return; // G90, G21, M3, M5 ...: silent, like the real machine
                }
                sleep(Duration::from_millis(30)); // the move takes a moment
                let positive_z = axis_words(line).iter().any(|(axis, v)| *axis == 2 && *v > 0.0);
                self.emit(if positive_z { "error: Coordinate out of range" } else { "ok" });
            }
        }
    }

    fn grbl_line(&mut self, line: &str) {
        sleep(Duration::from_millis(20)); // pretend the controller is busy for a moment
        let jog = line.starts_with("$J="); // GRBL jogs don't change the modal G90/G91 state
        let line = if jog { &line[3..] } else { line };
        let relative = line.contains("G91") || (self.relative && !line.contains("G90"));
        if !jog {
            self.relative = relative;
        }
        let words = axis_words(line);
        if line == "$H" || line == "G28" {
            self.pos = [0.0; 3];
        } else if line.contains("G92") {
            for (axis, v) in &words {
                self.pos[*axis] = *v;
            }
        } else {
            for (axis, v) in &words {
                self.pos[*axis] = if relative { self.pos[*axis] + v } else { *v };
            }
        }
        self.emit("ok");
    }
}

pub struct SimWriter(Arc<Mutex<State>>);

impl Write for SimWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        let mut state = self.0.lock().unwrap();
        for &byte in data {
            match (state.kind, byte) {
                (Kind::Grbl, b'?') => {
                    let [x, y, z] = state.pos;
                    let report = format!("<{}|MPos:{x:.3},{y:.3},{z:.3}|FS:0,0>", state.status);
                    state.emit(&report);
                }
                (Kind::Grbl, b'!') => state.status = "Hold",
                (Kind::Grbl, b'~') => state.status = "Idle",
                (Kind::Grbl, 0x18) => {
                    state.status = "Idle";
                    state.partial.clear();
                    state.emit(BANNER);
                }
                (_, b'\n') => {
                    let line = std::mem::take(&mut state.partial).trim().to_uppercase();
                    state.line(&line);
                }
                (_, byte) => state.partial.push(byte as char),
            }
        }
        Ok(data.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

pub struct SimReader {
    rx: Receiver<Vec<u8>>,
    pending: Vec<u8>,
}

impl Read for SimReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.pending.is_empty() {
            match self.rx.recv_timeout(Duration::from_millis(100)) {
                Ok(data) => self.pending = data,
                Err(_) => return Err(io::ErrorKind::TimedOut.into()), // like a serial port with a timeout
            }
        }
        let n = buf.len().min(self.pending.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }
}

/// Open a simulator by port name, or `None` if the name isn't one.
pub fn open(name: &str) -> Option<(SimReader, SimWriter)> {
    let kind = match name {
        SIM_GRBL => Kind::Grbl,
        SIM_CIRQOID => Kind::Cirqoid,
        _ => return None,
    };
    let (out, rx) = channel();
    let state = State { kind, pos: [0.0; 3], relative: false, status: "Idle", partial: String::new(), out };
    if kind == Kind::Grbl {
        state.emit(BANNER);
    }
    Some((SimReader { rx, pending: Vec::new() }, SimWriter(Arc::new(Mutex::new(state)))))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Send `data`, return everything the simulator answers.
    fn talk(name: &str, data: &[u8]) -> String {
        let (mut reader, mut writer) = open(name).unwrap();
        writer.write_all(data).unwrap();
        let mut out = Vec::new();
        let mut buf = [0u8; 256];
        while let Ok(n) = reader.read(&mut buf) {
            out.extend_from_slice(&buf[..n]);
        }
        String::from_utf8(out).unwrap()
    }

    #[test]
    fn unknown_names_are_not_simulators() {
        assert!(open("COM8").is_none());
    }

    #[test]
    fn grbl_banner_ok_and_status() {
        let out = talk(SIM_GRBL, b"G90\nG0 X5 Y-2\n?");
        assert!(out.starts_with("Grbl 1.1h"));
        assert_eq!(out.matches("ok").count(), 2);
        assert!(out.contains("<Idle|MPos:5.000,-2.000,0.000"), "{out}");
    }

    #[test]
    fn grbl_relative_jog_and_g92() {
        let out = talk(SIM_GRBL, b"G0 X10\n$J=G91 X1\nG0 X2\nG92 X0\n?");
        // $J is relative but doesn't change G90; G0 X2 is absolute again; G92 X0 sets X to 0
        assert!(out.contains("MPos:0.000,"), "{out}");
        let out = talk(SIM_GRBL, b"G0 X10\n$J=G91 X1\n?");
        assert!(out.contains("MPos:11.000,"), "{out}");
    }

    #[test]
    fn grbl_hold_resume_and_reset() {
        let out = talk(SIM_GRBL, b"!?~?\x18");
        assert!(out.contains("<Hold|") && out.contains("<Idle|"));
        assert_eq!(out.matches("Grbl 1.1h").count(), 2, "soft reset prints the banner again");
    }

    #[test]
    fn cirqoid_is_silent_for_settings_and_answers_moves() {
        assert_eq!(talk(SIM_CIRQOID, b"G90\nG21\nM3 S1000\n"), "");
        assert_eq!(talk(SIM_CIRQOID, b"G28 Y0.835\nG0 Z-1.0 F300.0\n"), "ok\r\nok\r\n");
        assert_eq!(talk(SIM_CIRQOID, b"G0 Z1.0 F300.0\n"), "error: Coordinate out of range\r\n");
    }
}
