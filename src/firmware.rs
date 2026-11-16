//! What differs between controller firmwares, and the Cirqoid's machine settings.

use crate::tracker::Limits;
use std::time::Duration;

pub const BAUDS: [u32; 6] = [9600, 19200, 38400, 57600, 115200, 250000];
/// How long to wait for "ok" before warning and sending the next line.
pub const ACK_TIMEOUT: Duration = Duration::from_secs(10);
/// Moves and homing can take a while on the Cirqoid.
pub const MOTION_TIMEOUT: Duration = Duration::from_secs(120);
pub const SPINDLE_MAX: i64 = 1400; // above this the Cirqoid's spindle doesn't get any faster
pub const SPINDLE_STEP: i64 = 50;
/// Line starts that count as the controller's answer.
pub const REPLIES: [&str; 4] = ["ok", "error", "ack", "nack"];

pub enum Command {
    None,
    /// A normal G-code line.
    Line(&'static str),
    /// Several lines in order (stops at the first refused one).
    Lines(&'static [&'static str]),
    /// Written straight to the port, no newline, no "ok".
    Realtime(&'static [u8]),
}

pub struct Firmware {
    pub name: &'static str,
    pub default_baud: u32,
    pub home: Command,
    pub unlock: Command,
    pub position: Command,
    pub estop: &'static [u8],
    pub hold: Option<&'static [u8]>,
    pub resume: Option<&'static [u8]>,
    /// Controllers that stay silent when they accept a line: wait this long for a possible "nack".
    pub silent_wait: Option<Duration>,
    /// Even in silent mode, moves and homing get a real "ok"/"error" (wait up to MOTION_TIMEOUT).
    pub replies_to_moves: bool,
    /// No G91: the app tracks the position and jogs with absolute moves.
    pub absolute_only: bool,
    pub soft_limits: Option<Limits>,
    /// Highest jog feed for Z (mm/min).
    pub max_z_feed: Option<f64>,
    /// Every coordinate/feed must have a decimal point (X5.0, not X5).
    pub decimal_words: bool,
}

impl Firmware {
    pub fn has(&self, command: &Command) -> bool {
        !matches!(command, Command::None)
    }
}

/// Cirqoid PCB mill: 38400 baud, silent for accepted setting lines, ok/error for moves, absolute only,
/// Z 0 at the top and negative going down, homing only with the G90/G21/G28 Y0.835 sequence,
/// no position query and no software e-stop (E-STOP can only turn the spindle off).
pub static CIRQOID: Firmware = Firmware {
    name: "Cirqoid",
    default_baud: 38400,
    home: Command::Lines(&["G90", "G21", "G28 Y0.835"]),
    unlock: Command::None,
    position: Command::None,
    estop: b"M5\n",
    hold: None,
    resume: None,
    silent_wait: Some(Duration::from_millis(500)),
    replies_to_moves: true,
    absolute_only: true,
    soft_limits: Some([(0.0, 103.0), (0.0, 224.835), (-26.9, 0.0)]),
    max_z_feed: Some(300.0),
    decimal_words: true,
};

pub static GRBL: Firmware = Firmware {
    name: "GRBL",
    default_baud: 115200,
    home: Command::Line("$H"),
    unlock: Command::Line("$X"),
    position: Command::Realtime(b"?"),
    estop: b"\x18",
    hold: Some(b"!"),
    resume: Some(b"~"),
    silent_wait: None,
    replies_to_moves: false,
    absolute_only: false,
    soft_limits: None,
    max_z_feed: None,
    decimal_words: false,
};

pub static MARLIN: Firmware = Firmware {
    name: "Marlin",
    default_baud: 250000,
    home: Command::Line("G28"),
    unlock: Command::Line("M999"),
    position: Command::Line("M114"),
    estop: b"M112\n",
    hold: None,
    resume: None,
    silent_wait: None,
    replies_to_moves: false,
    absolute_only: false,
    soft_limits: None,
    max_z_feed: None,
    decimal_words: false,
};

pub static GENERIC: Firmware = Firmware {
    name: "Generic",
    default_baud: 115200,
    home: Command::Line("G28"),
    unlock: Command::None,
    position: Command::Line("M114"),
    estop: b"M112\n",
    hold: None,
    resume: None,
    silent_wait: None,
    replies_to_moves: false,
    absolute_only: false,
    soft_limits: None,
    max_z_feed: None,
    decimal_words: false,
};

pub static FIRMWARES: [&Firmware; 4] = [&CIRQOID, &GRBL, &MARLIN, &GENERIC];

pub fn by_name(name: &str) -> &'static Firmware {
    FIRMWARES.iter().find(|f| f.name == name).copied().unwrap_or(&GENERIC)
}
