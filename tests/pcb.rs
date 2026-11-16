//! Gerber + drill files → board → jobs → Cirqoid G-code, with real KiCad-style files.

use geo::{Area, BoundingRect};
use soft_cnc::cam::{convert, Options};
use soft_cnc::pcb::toolpath::{generate, program, JobKind, Settings};
use soft_cnc::pcb::{load_board, Layer};
use soft_cnc::tracker::PositionTracker;
use std::path::PathBuf;

fn sample(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/samples/pcb")).join(name)
}

fn demo() -> Vec<PathBuf> {
    ["demo-F_Cu.gbr", "demo-B_Cu.gbr", "demo-Edge_Cuts.gbr", "demo.drl"].map(sample).to_vec()
}

#[test]
fn layers_are_recognised_and_the_board_is_moved_to_zero() {
    let board = load_board(&demo()).unwrap();
    let layers: Vec<Layer> = board.files.iter().map(|f| f.layer).collect();
    assert_eq!(layers, [Layer::CopperTop, Layer::CopperBottom, Layer::Outline, Layer::Drills]);
    assert!(board.files[0].reason.contains("Copper,L1,Top"));
    assert!((board.width - 30.0).abs() < 0.01 && (board.height - 20.0).abs() < 0.01, "{} x {}", board.width, board.height);
    let outline = board.outline.as_ref().unwrap().bounding_rect().unwrap();
    assert!(outline.min().x.abs() < 0.01 && outline.min().y.abs() < 0.01, "lower-left corner is (0, 0)");
    // the copper sits inside the board
    let copper = board.top.as_ref().unwrap().bounding_rect().unwrap();
    assert!(copper.min().x > 0.0 && copper.max().x < 30.0 && copper.min().y > 0.0 && copper.max().y < 20.0);
    assert_eq!(board.drills.holes.len(), 3);
    assert_eq!(board.drills.slots.len(), 1);
    assert!((board.drills.holes[0].at.x - 10.0).abs() < 1e-9 && (board.drills.holes[0].at.y - 15.0).abs() < 1e-9);
}

#[test]
fn copper_area_matches_the_pads_tracks_and_zone() {
    let board = load_board(&demo()).unwrap();
    let area = board.top.as_ref().unwrap().unsigned_area();
    // zone 7 x 2 = 14, two rounded pads ~0.85 each, square pad 2.89, round pad 2.27, via 0.5, tracks ~6 + arc ~2
    assert!(area > 25.0 && area < 32.0, "copper area {area}");
}

#[test]
fn jobs_for_a_double_sided_board() {
    let board = load_board(&demo()).unwrap();
    let settings = Settings { double_sided: true, ..Default::default() };
    let (jobs, notes) = generate(&board, &settings);
    let kinds: Vec<JobKind> = jobs.iter().map(|j| j.kind).collect();
    assert_eq!(kinds, [JobKind::Alignment, JobKind::Isolation, JobKind::Drill, JobKind::Drill, JobKind::Drill,
                       JobKind::Isolation, JobKind::Cutout], "{notes:?}");
    assert!(jobs[1].title.starts_with("Isolation, top"));
    assert!(jobs[5].title.starts_with("Isolation, bottom, mirrored") && jobs[5].hint.contains("Flip"));
    assert!(jobs[4].title.contains("1 slot"), "{}", jobs[4].title);

    // the whole program becomes Cirqoid G-code inside the machine's travel
    let text = program(&jobs);
    let converted = convert(&text, 20.0, 30.0, -15.0, Options::default()).unwrap();
    assert_eq!(converted.sections.len(), jobs.len());
    for (section, job) in converted.sections.iter().zip(&jobs) {
        assert!(section.note.starts_with("Job: "), "{}", section.note);
        assert_eq!(section.note.trim_start_matches("Job: "), job.title);
        let mut tracker = PositionTracker::new(soft_cnc::firmware::CIRQOID.soft_limits);
        tracker.apply("G28 Y0.835").unwrap();
        for line in &section.lines {
            tracker.apply(line).unwrap_or_else(|e| panic!("{}: {line}: {e}", job.title));
        }
    }
}

#[test]
fn single_sided_board_without_bottom_layer() {
    let files = ["demo-F_Cu.gbr", "demo-Edge_Cuts.gbr", "demo.drl"].map(sample);
    let board = load_board(&files).unwrap();
    let (jobs, _) = generate(&board, &Settings::default());
    assert_eq!(jobs.first().unwrap().kind, JobKind::Isolation);
    assert_eq!(jobs.last().unwrap().kind, JobKind::Cutout);
    assert!(!jobs.iter().any(|j| j.kind == JobKind::Alignment));
}

#[test]
fn the_users_kicad_test_files_load() {
    // KiCad export where the board frame was drawn on F.Cu and Edge_Cuts is empty
    let files = ["Custom_Arduino-F_Cu.gbr", "Custom_Arduino-Edge_Cuts.gbr"].map(sample);
    let board = load_board(&files).unwrap();
    assert!(board.outline.is_none());
    assert!(board.notes.iter().any(|n| n.contains("outline layer is empty")), "{:?}", board.notes);
    assert!((board.width - 31.5).abs() < 0.01 && (board.height - 21.0).abs() < 0.01, "{} x {}", board.width, board.height);
    let (jobs, notes) = generate(&board, &Settings::default());
    assert_eq!(jobs.len(), 1, "only isolation: no holes, no outline");
    assert!(notes.iter().any(|n| n.contains("no cutout")));
}

#[test]
fn non_pcb_files_are_rejected_with_a_clear_message() {
    // alone, a G-code file gives nothing to mill; picked along with Gerbers it is skipped with a note
    let gcode = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/cirqoid_square.gcode"));
    let err = load_board(std::slice::from_ref(&gcode)).err().unwrap();
    assert!(err.contains("No copper layer or drill holes found"), "{err}");
    let board = load_board(&[sample("demo-F_Cu.gbr"), gcode]).unwrap();
    assert!(board.notes.iter().any(|n| n.contains("cirqoid_square.gcode skipped")));
}
