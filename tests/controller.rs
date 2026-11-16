//! Drive the app's logic headless against the built-in simulators (no window, no hardware).

use soft_cnc::controller::Controller;
use soft_cnc::link::LogTag;
use soft_cnc::sim::{SIM_CIRQOID, SIM_GRBL};
use std::path::Path;
use std::thread::sleep;
use std::time::{Duration, Instant};

const EXAMPLES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples");

/// Keep handling events until `done` is true (or fail after `secs`).
fn wait_for(c: &mut Controller, secs: u64, what: &str, done: impl Fn(&Controller) -> bool) {
    let end = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < end {
        c.poll();
        if done(c) {
            return;
        }
        sleep(Duration::from_millis(5));
    }
    let tail: Vec<&str> = c.log.iter().rev().take(15).map(|l| l.text.as_str()).collect();
    panic!("timed out waiting for {what}; last log lines: {tail:?}");
}

fn settle(c: &mut Controller, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < end {
        c.poll();
        sleep(Duration::from_millis(5));
    }
}

fn readout(c: &Controller) -> [String; 3] {
    c.readout().0
}

fn logged(c: &Controller, text: &str) -> bool {
    c.log.iter().any(|l| l.text.contains(text))
}

/// Connected to the Cirqoid simulator and homed.
fn homed_cirqoid() -> Controller {
    let mut c = Controller::new(None);
    c.set_firmware("Cirqoid");
    c.port = SIM_CIRQOID.into();
    c.connect().unwrap();
    assert_eq!(c.readout().1, "not homed — press Home");
    c.home();
    wait_for(&mut c, 5, "homing", |c| c.readout().1.starts_with("homed"));
    c
}

#[test]
fn cirqoid_home_jog_and_soft_limits() {
    let mut c = homed_cirqoid();
    assert_eq!(readout(&c), ["0.000", "0.835", "0.000"]);
    assert!(logged(&c, "G28 Y0.835"));

    c.jog(2, 1.0); // Z up at the top: refused by the app
    assert!(logged(&c, "Z is already at its soft limit"));
    c.jog(2, -1.0);
    wait_for(&mut c, 3, "Z down", |c| readout(c)[2] == "-1.000");
    assert!(logged(&c, "G0 Z-1.0 F300.0"), "Z jogs use the decimal dialect and the Z feed cap");

    c.step = "50".into();
    c.jog(2, -1.0);
    wait_for(&mut c, 3, "Z clamp", |c| readout(c)[2] == "-26.900");
    assert!(logged(&c, "Z jog shortened to stop at the soft limit"));

    c.send_entry("G0 Z1");
    assert!(logged(&c, "Not sent: G0 Z1.0  (Z would go to 1 mm from home; soft limit is -26.9 to 0)"));

    c.step = "10".into();
    c.jog(0, 1.0);
    wait_for(&mut c, 3, "X move", |c| readout(c)[0] == "10.000");
    let ok_folded = c.log.iter().any(|l| l.text == "G0 X10.0 F500.0" && l.ok);
    assert!(ok_folded, "'ok' is shown on the command's own line");
}

#[test]
fn a_move_the_machine_refuses_keeps_the_position() {
    let mut c = homed_cirqoid();
    c.tracker.as_mut().unwrap().limits = None; // let the line through so the (simulated) machine refuses it
    c.send_entry("G0 Z5");
    wait_for(&mut c, 3, "refusal", |c| logged(c, "Position kept as before 'G0 Z5.0'"));
    assert!(logged(&c, "error: Coordinate out of range"));
    assert_eq!(readout(&c)[2], "0.000");
    assert_eq!(c.tracker.as_ref().unwrap().work().unwrap()[2], 0.0);
}

#[test]
fn cam_file_is_detected_converted_and_run() {
    let mut c = homed_cirqoid();
    c.send_entry("G0 X20 Y30 F1200");
    c.send_entry("G0 Z-10 F300");
    wait_for(&mut c, 3, "positioning", |c| readout(c) == ["20.000", "30.000", "-10.000"]);
    c.set_origin_here();
    c.set_surface_here();
    let etch = Path::new(EXAMPLES).join("etch_square_circle.gcode");
    c.load(&etch).unwrap();
    assert!(c.file_label.ends_with("lines, ready for the Cirqoid"), "{}", c.file_label);
    assert!(logged(&c, "read as CAM G-code (Z0 = copper surface): placed at board origin X20.0 Y30.0, copper at Z-10.0"));

    c.start_job();
    assert!(c.job_running());
    let mut readouts = std::collections::HashSet::new();
    wait_for(&mut c, 30, "job end", |c| !c.job_running());
    c.log.iter().filter(|l| l.tag == LogTag::Tx).for_each(|l| { readouts.insert(l.text.clone()); });
    assert!(logged(&c, "Job finished"));
    assert!(logged(&c, "G1 Z-10.2 F50.0"), "cuts 0.2 mm below the surface");
    assert!(logged(&c, "M3 S1000"), "spindle started by the job");
    assert_eq!(c.spindle_text, "off", "spindle off at the end");
    assert!(readouts.len() > 60);
}

#[test]
fn ready_made_examples_load_and_run() {
    let mut c = homed_cirqoid();
    for name in ["cirqoid_small_square.gcode", "cirqoid_square.gcode"] {
        let path = Path::new(EXAMPLES).join(name);
        c.load(&path).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(logged(&c, "read as machine G-code (Z0 = machine top): positions kept"));
        assert!(c.job_lines.contains(&"G28 Y0.835".to_string()), "its homing line is kept");
        c.start_job();
        assert!(c.job_running(), "{name} passes the soft-limit check");
        wait_for(&mut c, 40, name, |c| !c.job_running());
        assert!(!logged(&c, "error:"), "{name}");
    }
}

#[test]
fn job_with_an_out_of_range_line_does_not_start() {
    let mut c = homed_cirqoid();
    c.origin_x = "90".into();
    c.origin_y = "30".into();
    c.surface = "-10".into();
    let etch = Path::new(EXAMPLES).join("etch_square_circle.gcode");
    c.load(&etch).unwrap();
    c.start_job();
    assert!(!c.job_running());
    assert!(logged(&c, "Job not started: line"));
    assert!(logged(&c, "X would go to 110 mm from home"));
}

#[test]
fn refused_job_line_stops_the_job() {
    let mut c = homed_cirqoid();
    c.tracker.as_mut().unwrap().limits = None;
    c.job_lines = ["G0 X5 F1200", "G0 Z5 F300", "G0 X20 F1200"].map(String::from).to_vec();
    c.start_job();
    wait_for(&mut c, 5, "job stop", |c| !c.job_running());
    assert!(logged(&c, "Job stopped: the machine refused 'G0 Z5.0 F300.0'"));
    assert!(!logged(&c, "G0 X20.0 F1200.0"), "nothing is sent after the refused line");
    assert_eq!(readout(&c), ["5.000", "0.835", "0.000"]);
}

#[test]
fn cam_file_needs_the_board_position_and_estop_clears_the_queue() {
    let mut c = homed_cirqoid();
    let etch = Path::new(EXAMPLES).join("etch_square_circle.gcode");
    assert!(c.load(&etch).unwrap_err().contains("Set origin here"));
    c.job_lines = (1..=20).map(|i| format!("G0 X{i} F1200")).collect();
    c.start_job();
    settle(&mut c, 100);
    c.estop();
    settle(&mut c, 400);
    assert!(!c.job_running());
    assert!(logged(&c, "*** STOP"));
    assert!(!logged(&c, "G0 X20.0 F1200.0"), "queued lines were dropped");
}

/// Typical "foreign" G-code: whole numbers, relative moves, an arc, inches → all made Cirqoid-ready on load.
#[test]
fn any_gcode_is_made_cirqoid_ready_on_load() {
    let mut c = homed_cirqoid();
    let path = std::env::temp_dir().join("soft_cnc_generic_test.nc");
    std::fs::write(&path, "G21 G90\nG0 X10 Y10 F800\nG91\nG1 X5 F200\nG1 Y5\nG90\nG2 X25 Y25 I5 J0\n\
                           G0 Z-2\nG20\nG0 X1\n").unwrap();
    c.load(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let lines = &c.job_lines;
    assert!(lines.contains(&"G0 X10.0 Y10.0 F800.0".to_string()), "decimals added, the file's G0 speed kept");
    assert!(lines.contains(&"G1 X15.0 F200.0".to_string()), "G91 distance turned into an absolute position");
    assert!(lines.contains(&"G1 Y15.0 F200.0".to_string()));
    assert!(!lines.iter().any(|l| l.starts_with("G2 ") || l.starts_with("G91")), "{lines:?}");
    assert!(lines.contains(&"G0 X25.4 F1200.0".to_string()), "inches converted to mm");
    c.start_job();
    wait_for(&mut c, 20, "generic job", |c| !c.job_running());
    assert!(logged(&c, "Job finished") && !logged(&c, "error:"));
}

/// Gerber + drill files → jobs → each job loaded at the board position and run on the Cirqoid simulator.
#[test]
fn pcb_files_become_jobs_that_run() {
    use soft_cnc::pcb::{load_board, toolpath::{generate, Settings}};
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/samples/pcb");
    let files: Vec<std::path::PathBuf> =
        ["demo-F_Cu.gbr", "demo-Edge_Cuts.gbr", "demo.drl"].iter().map(|f| Path::new(dir).join(f)).collect();
    let board = load_board(&files).unwrap();
    let (jobs, notes) = generate(&board, &Settings::default());

    let mut c = homed_cirqoid();
    c.load_pcb_jobs("demo".into(), &jobs, &notes);
    assert!(logged(&c, "Set the board origin and surface Z"), "no board position yet: nothing loaded");
    assert!(c.job_lines.is_empty());

    c.send_entry("G0 X20 Y30 F1200");
    c.send_entry("G0 Z-10 F300");
    wait_for(&mut c, 3, "positioning", |c| readout(c) == ["20.000", "30.000", "-10.000"]);
    c.set_origin_here();
    c.set_surface_here();
    let program = c.program.clone().unwrap();
    assert_eq!(program.jobs.len(), jobs.len());
    for index in 0..program.jobs.len() {
        c.load_job(index).unwrap();
        assert!(c.file_label.contains(&format!("job {} of {}", index + 1, jobs.len())), "{}", c.file_label);
        c.start_job();
        assert!(c.job_running(), "job {} passes the soft limits", index + 1);
        wait_for(&mut c, 120, "PCB job", |c| !c.job_running());
        assert!(!logged(&c, "error:"), "job {}", index + 1);
    }
    assert!(logged(&c, "G1 Z-10.1 F50.0"), "isolation cuts 0.1 mm into the copper");
    assert!(logged(&c, "G1 Z-11.8 F60.0"), "drills go 1.8 mm deep at the drill feed");
    assert_eq!(c.program.as_ref().unwrap().loaded, Some(jobs.len() - 1));
}

#[test]
fn other_firmwares_get_the_file_as_written() {
    let mut c = Controller::new(None);
    c.set_firmware("GRBL");
    let path = Path::new(EXAMPLES).join("etch_square_circle.gcode");
    c.load(&path).unwrap();
    assert!(c.job_lines.contains(&"G1 Z-0.2 F50.0".to_string()));
    assert!(c.job_lines.contains(&"G2 X18.0 Y10.0 I-8.0 J0.0 F100.0".to_string()));
}

#[test]
fn grbl_simulator_jogs_and_reports_status() {
    let mut c = Controller::new(None);
    c.set_firmware("GRBL");
    c.port = SIM_GRBL.into();
    c.connect().unwrap();
    wait_for(&mut c, 3, "banner", |c| logged(c, "Grbl 1.1h"));
    c.step = "5".into();
    c.jog(0, 1.0);
    wait_for(&mut c, 3, "status report", |c| readout(c)[0].starts_with("5.000"));
    assert!(logged(&c, "$J=G91 G21 X5 F500"));
    assert_eq!(c.readout().1, "Idle");
}

#[test]
fn generic_firmware_jogs_with_relative_moves() {
    let mut c = Controller::new(None);
    c.set_firmware("Generic");
    assert_eq!(c.baud, "115200", "choosing a firmware sets its usual baud rate");
    c.port = SIM_GRBL.into();
    c.connect().unwrap();
    c.jog(1, -1.0);
    wait_for(&mut c, 3, "jog", |c| logged(c, "G90"));
    let sent: Vec<&str> = c.log.iter().filter(|l| l.tag == LogTag::Tx).map(|l| l.text.as_str()).collect();
    assert_eq!(sent, ["G91", "G1 Y-1 F500", "G90"]);
}

#[test]
fn connect_errors_are_clear() {
    let mut c = Controller::new(None);
    c.port = SIM_GRBL.into();
    c.baud = "fast".into();
    assert_eq!(c.connect().unwrap_err(), "'fast' is not a baud rate");
    c.baud = "115200".into();
    c.port = "COM250".into();
    assert!(c.connect().is_err());
    assert!(!c.connected());
}

#[test]
fn spindle_speed_limits_and_live_changes() {
    let mut c = homed_cirqoid();
    c.spindle_set();
    assert!(logged(&c, "Spindle is off; S1000 will be used when you press Start"));
    c.speed = 5000;
    c.spindle_start();
    assert!(logged(&c, "Spindle speed limited to S1400"));
    wait_for(&mut c, 3, "spindle on", |c| c.spindle_on);
    assert_eq!(c.spindle_text, "● on S1400");
    c.speed = 700;
    c.spindle_set();
    wait_for(&mut c, 3, "new speed", |c| c.spindle_text == "● on S700");
    c.spindle_stop();
    wait_for(&mut c, 3, "spindle off", |c| !c.spindle_on);
}

#[test]
fn disconnect_resets_everything() {
    let mut c = homed_cirqoid();
    c.spindle_start();
    wait_for(&mut c, 3, "spindle on", |c| c.spindle_on);
    c.disconnect();
    assert!(!c.connected());
    assert_eq!(c.readout().1, "disconnected");
    assert!(!c.spindle_on && c.tracker.is_none());
}

#[test]
fn pause_resume_and_stop_keep_the_position_honest() {
    let mut c = homed_cirqoid();
    c.job_lines = (1..=20).map(|i| format!("G0 X{i} F1200")).collect();
    c.start_job();
    wait_for(&mut c, 5, "first lines", |c| c.job_done >= 2);
    c.pause_job();
    assert!(c.paused() && logged(&c, "Job paused"));
    settle(&mut c, 300);
    let done = c.job_done;
    settle(&mut c, 300);
    assert!(c.job_done <= done + 1, "at most the line already on its way finishes while paused");
    c.pause_job();
    assert!(!c.paused() && logged(&c, "Job resumed"));
    wait_for(&mut c, 5, "more lines", |c| c.job_done >= done + 2);
    c.stop_job();
    settle(&mut c, 300);
    assert!(!c.job_running());
    // the jog target comes from the queued position: after Stop it must match what was really sent
    let sent = c.display.as_ref().unwrap().work().unwrap();
    let queued = c.tracker.as_ref().unwrap().work().unwrap();
    assert_eq!(sent, queued, "Stop forgets the lines that never went out");
    assert!(sent[0] < 20.0);
}

#[test]
fn estop_on_grbl_resets_the_controller() {
    let mut c = Controller::new(None);
    c.set_firmware("GRBL");
    c.port = SIM_GRBL.into();
    c.connect().unwrap();
    wait_for(&mut c, 3, "banner", |c| logged(c, "Grbl 1.1h"));
    c.estop();
    wait_for(&mut c, 3, "reset banner", |c| c.log.iter().filter(|l| l.text.starts_with("Grbl 1.1h")).count() == 2);
    assert!(logged(&c, "*** E-STOP sent"));
}

#[test]
fn air_test_and_a_new_surface_z_change_the_loaded_job() {
    let mut c = homed_cirqoid();
    c.origin_x = "20".into();
    c.origin_y = "30".into();
    c.surface = "-10".into();
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/samples/pcb2gcode_drill.ngc");
    c.load(&path).unwrap();
    let program = c.program.clone().unwrap();
    assert_eq!(program.jobs.len(), 2, "two drill sizes");
    assert!(c.job_lines.contains(&"G1 Z-11.8 F150.0".to_string()));
    // a different bit sits at a different height: load job 2 after setting the new surface Z
    c.surface = "-12".into();
    c.load_job(1).unwrap();
    assert!(c.job_lines.iter().any(|l| l == "G1 Z-13.8 F150.0"), "{:?}", c.job_lines);
    c.air_test = true;
    c.air_offset = "5".into();
    c.load_job(1).unwrap();
    assert!(c.job_lines.iter().any(|l| l == "G1 Z-8.8 F150.0"), "lifted 5 mm");
    assert!(logged(&c, "AIR TEST"));
}

#[test]
fn nothing_runs_while_a_job_is_running() {
    let mut c = homed_cirqoid();
    c.job_lines = (1..=10).map(|i| format!("G0 X{i} F1200")).collect();
    c.start_job();
    let total = c.job_total;
    c.start_job(); // pressing Start again changes nothing
    assert_eq!(c.job_total, total);
    c.estop();
}
