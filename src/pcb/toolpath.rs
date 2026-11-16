//! Jobs from a board, like FlatCAM: isolation routing, drilling, board cutout with tabs, and the extra steps for
//! double-sided boards (bottom layer mirrored, alignment holes).
//!
//! Output is CAM-style G-code (X0 Y0 = board's lower-left corner, Z0 = copper surface); the app's converter
//! then places it on the machine like any other CAM file.

use super::excellon::Drills;
use super::geom::{mirror_x, rings};
use super::Board;
use crate::gcode::fmt;
use geo::{Buffer, Coord, MultiPolygon, Polygon};

/// Tool and cutting settings (mm, mm/min). Defaults suit a V-bit for isolation and a 1 mm end mill for cutout.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub travel_z: f64,
    pub spindle: i64,
    pub plunge_feed: f64,
    // isolation
    pub iso_tool: f64, // width the V-bit cuts at the cut depth
    pub iso_passes: u32,
    pub iso_overlap: f64, // 0..1, how much each pass overlaps the previous one
    pub iso_depth: f64,
    pub iso_feed: f64,
    // drilling
    pub drill_depth: f64,
    pub drill_feed: f64,
    // cutout
    pub cut_tool: f64,
    pub cut_depth: f64,
    pub cut_step: f64, // depth per pass
    pub cut_feed: f64,
    pub tabs: u32,
    pub tab_width: f64,
    pub tab_height: f64, // material left standing in the tabs
    // double-sided
    pub double_sided: bool,
    pub align_diameter: f64,
    pub align_margin: f64, // distance of the alignment holes from the board edge
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            travel_z: 2.0,
            spindle: 1000,
            plunge_feed: 50.0,
            iso_tool: 0.2,
            iso_passes: 2,
            iso_overlap: 0.4,
            iso_depth: 0.1,
            iso_feed: 120.0,
            drill_depth: 1.8,
            drill_feed: 60.0,
            cut_tool: 1.0,
            cut_depth: 1.8,
            cut_step: 0.6,
            cut_feed: 100.0,
            tabs: 4,
            tab_width: 2.0,
            tab_height: 0.6,
            double_sided: false,
            align_diameter: 1.0,
            align_margin: 5.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobKind {
    Isolation,
    Drill,
    Cutout,
    Alignment,
}

pub struct Job {
    pub kind: JobKind,
    /// Shown in the jobs list, e.g. "Isolation, top · V-bit 0.20 mm · 2 passes".
    pub title: String,
    /// What to do before running it (flip the board, change the bit…).
    pub hint: String,
    /// Tool paths, for the preview.
    pub paths: Vec<Vec<Coord<f64>>>,
    gcode: Vec<String>,
}

/// G-code writer for one job, in CAM coordinates.
struct Writer<'a> {
    s: &'a Settings,
    lines: Vec<String>,
    paths: Vec<Vec<Coord<f64>>>,
}

impl<'a> Writer<'a> {
    fn new(s: &'a Settings) -> Self {
        let lines = vec![format!("M3 S{}", s.spindle.clamp(50, 1400)), format!("G0 Z{}", fmt(s.travel_z))];
        Self { s, lines, paths: Vec::new() }
    }

    fn up(&mut self) {
        self.lines.push(format!("G0 Z{}", fmt(self.s.travel_z)));
    }

    fn rapid(&mut self, p: Coord<f64>) {
        self.lines.push(format!("G0 X{} Y{}", fmt(p.x), fmt(p.y)));
    }

    fn plunge(&mut self, z: f64) {
        self.lines.push(format!("G1 Z{} F{}", fmt(z), fmt(self.s.plunge_feed)));
    }

    fn cut(&mut self, p: Coord<f64>, feed: f64) {
        self.lines.push(format!("G1 X{} Y{} F{}", fmt(p.x), fmt(p.y), fmt(feed)));
    }

    fn finish(mut self, kind: JobKind, title: String, hint: String) -> Job {
        self.up();
        self.lines.push("M5".into());
        Job { kind, title, hint, paths: self.paths, gcode: self.lines }
    }
}

fn dist(a: Coord<f64>, b: Coord<f64>) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

/// Visit rings nearest-first, each starting at its vertex closest to where the last one ended (less travel).
fn order_rings(mut rings: Vec<Vec<Coord<f64>>>) -> Vec<Vec<Coord<f64>>> {
    let mut at = Coord { x: 0.0, y: 0.0 };
    let mut ordered = Vec::with_capacity(rings.len());
    while !rings.is_empty() {
        let (ring_index, vertex) = rings
            .iter()
            .enumerate()
            .flat_map(|(r, ring)| ring.iter().enumerate().map(move |(v, p)| (r, v, dist(*p, at))))
            .min_by(|a, b| a.2.total_cmp(&b.2))
            .map(|(r, v, _)| (r, v))
            .unwrap();
        let mut ring = rings.swap_remove(ring_index);
        if ring.first() == ring.last() {
            ring.pop();
        }
        let last = ring.len().saturating_sub(1);
        ring.rotate_left(vertex.min(last));
        let first = ring[0];
        ring.push(first); // closed again, starting at the chosen vertex
        at = ring[0];
        ordered.push(ring);
    }
    ordered
}

/// Isolation routing: outlines around the copper, one per pass, each further out by the tool width × (1 − overlap).
fn isolation(copper: &MultiPolygon<f64>, s: &Settings, side: &str) -> Option<Job> {
    if copper.0.is_empty() {
        return None;
    }
    let r = s.iso_tool / 2.0;
    let step = s.iso_tool * (1.0 - s.iso_overlap.clamp(0.0, 0.9));
    let mut all = Vec::new();
    for pass in 0..s.iso_passes.max(1) {
        all.extend(rings(&copper.buffer(r + pass as f64 * step)));
    }
    let mut w = Writer::new(s);
    for ring in order_rings(all) {
        w.rapid(ring[0]);
        w.plunge(-s.iso_depth);
        for p in &ring[1..] {
            w.cut(*p, s.iso_feed);
        }
        w.up();
        w.paths.push(ring);
    }
    let passes = s.iso_passes.max(1);
    let title = format!("Isolation, {side} · V-bit {} mm · {passes} pass{}", fmt(s.iso_tool), if passes == 1 { "" } else { "es" });
    Some(w.finish(JobKind::Isolation, title, String::new()))
}

/// One drilling job per hole size; slots are milled by moving the drill along them.
fn drilling(drills: &Drills, s: &Settings) -> Vec<Job> {
    let mut sizes: Vec<f64> = drills.holes.iter().map(|h| h.diameter).chain(drills.slots.iter().map(|s| s.diameter)).collect();
    sizes.sort_by(f64::total_cmp);
    sizes.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    sizes
        .into_iter()
        .map(|size| {
            let mut points: Vec<Coord<f64>> =
                drills.holes.iter().filter(|h| (h.diameter - size).abs() < 1e-6).map(|h| h.at).collect();
            let slots: Vec<_> = drills.slots.iter().filter(|s| (s.diameter - size).abs() < 1e-6).collect();
            let mut w = Writer::new(s);
            let mut at = Coord { x: 0.0, y: 0.0 };
            while !points.is_empty() {
                // nearest hole next
                let i = (0..points.len()).min_by(|a, b| dist(points[*a], at).total_cmp(&dist(points[*b], at))).unwrap();
                let p = points.swap_remove(i);
                w.rapid(p);
                w.plunge(-s.drill_depth);
                w.up();
                w.paths.push(vec![p]);
                at = p;
            }
            for slot in &slots {
                w.rapid(slot.from);
                w.plunge(-s.drill_depth);
                w.cut(slot.to, s.drill_feed);
                w.up();
                w.paths.push(vec![slot.from, slot.to]);
            }
            let holes = drills.holes.iter().filter(|h| (h.diameter - size).abs() < 1e-6).count();
            let mut title = format!("Drill {} mm · {holes} hole{}", fmt(size), if holes == 1 { "" } else { "s" });
            if !slots.is_empty() {
                title.push_str(&format!(" + {} slot{}", slots.len(), if slots.len() == 1 { "" } else { "s" }));
            }
            // drills plunge at the drill feed, not the general plunge feed
            let feed = format!("F{}", fmt(s.plunge_feed));
            for line in &mut w.lines {
                if line.starts_with("G1 Z") {
                    *line = line.replace(&feed, &format!("F{}", fmt(s.drill_feed)));
                }
            }
            w.finish(JobKind::Drill, title, format!("Fit a {} mm drill and set the surface Z.", fmt(size)))
        })
        .collect()
}

/// Board cutout: the outline moved out by the tool radius, cut in depth passes, with tabs left standing.
fn cutout(board: &Polygon<f64>, s: &Settings) -> Option<Job> {
    let path = board.buffer(s.cut_tool / 2.0);
    let outer = path.0.iter().max_by(|a, b| {
        use geo::Area;
        a.unsigned_area().total_cmp(&b.unsigned_area())
    })?;
    let mut ring: Vec<Coord<f64>> = outer.exterior().0.clone();
    if ring.len() < 3 {
        return None;
    }
    if ring.first() != ring.last() {
        ring.push(ring[0]);
    }
    // tab gaps along the path (wider than the tab by the tool diameter, so the tab itself keeps its width)
    let perimeter: f64 = ring.windows(2).map(|w| dist(w[0], w[1])).sum();
    let gap = s.tab_width + s.cut_tool;
    let tabs: Vec<(f64, f64)> = if s.tabs > 0 && s.tab_height > 0.0 && gap * (s.tabs as f64) < perimeter * 0.5 {
        (0..s.tabs)
            .map(|k| {
                let centre = perimeter * (k as f64 + 0.5) / s.tabs as f64;
                (centre - gap / 2.0, centre + gap / 2.0)
            })
            .collect()
    } else {
        Vec::new()
    };
    // split the ring at the tab edges; each piece is either inside a tab or not
    let mut points = vec![(ring[0], false)];
    let mut walked = 0.0;
    for w in ring.windows(2) {
        let length = dist(w[0], w[1]);
        let mut cuts: Vec<f64> = tabs
            .iter()
            .flat_map(|(a, b)| [*a, *b])
            .filter(|d| *d > walked && *d < walked + length)
            .collect();
        cuts.sort_by(f64::total_cmp);
        let mut previous = walked;
        for d in cuts.into_iter().chain(std::iter::once(walked + length)) {
            let t = (d - walked) / length.max(1e-12);
            let p = Coord { x: w[0].x + (w[1].x - w[0].x) * t, y: w[0].y + (w[1].y - w[0].y) * t };
            let middle = (previous + d) / 2.0;
            let in_tab = tabs.iter().any(|(a, b)| middle > *a && middle < *b);
            points.push((p, in_tab));
            previous = d;
        }
        walked += length;
    }
    let mut depths = Vec::new();
    let mut z = 0.0;
    while z > -s.cut_depth + 1e-9 {
        z = (z - s.cut_step.max(0.05)).max(-s.cut_depth);
        depths.push(z);
    }
    let tab_top = -(s.cut_depth - s.tab_height).max(0.0);
    let mut w = Writer::new(s);
    w.rapid(points[0].0);
    for &depth in &depths {
        let mut current = depth;
        w.plunge(depth);
        for &(p, in_tab) in &points[1..] {
            let target = if in_tab { depth.max(tab_top) } else { depth };
            if (target - current).abs() > 1e-9 {
                w.plunge(target);
                current = target;
            }
            w.cut(p, s.cut_feed);
        }
    }
    w.paths.push(points.iter().map(|(p, _)| *p).collect());
    let title = format!(
        "Cutout · end mill {} mm · {} pass{}, {} tab{}",
        fmt(s.cut_tool), depths.len(), if depths.len() == 1 { "" } else { "es" }, tabs.len(), if tabs.len() == 1 { "" } else { "s" }
    );
    Some(w.finish(JobKind::Cutout, title, format!("Fit the {} mm end mill and set the surface Z.", fmt(s.cut_tool))))
}

fn mirror_drills(drills: &Drills, axis: f64) -> Drills {
    let m = |c: Coord<f64>| Coord { x: 2.0 * axis - c.x, y: c.y };
    let mut out = drills.clone();
    out.holes.iter_mut().for_each(|h| h.at = m(h.at));
    out.slots.iter_mut().for_each(|s| {
        s.from = m(s.from);
        s.to = m(s.to);
    });
    out
}

/// All jobs for the board, in the order they should be run, plus notes for the user.
pub fn generate(board: &Board, s: &Settings) -> (Vec<Job>, Vec<String>) {
    let mut jobs = Vec::new();
    let mut notes = Vec::new();
    let axis = board.width / 2.0; // flipping the board left-right about its middle keeps it in place
    let top = board.top.as_ref().filter(|c| !c.0.is_empty());
    let bottom = board.bottom.as_ref().filter(|c| !c.0.is_empty());
    // Bottom-only board: you work on the bottom face, so everything is seen mirrored.
    let bottom_only = top.is_none() && bottom.is_some();
    let double = s.double_sided && top.is_some() && bottom.is_some();
    if s.double_sided && !double {
        notes.push("Double-sided needs both a top and a bottom copper layer; making a single-sided job list.".into());
    }
    if double {
        let holes = [Coord { x: axis, y: -s.align_margin }, Coord { x: axis, y: board.height + s.align_margin }];
        let mut w = Writer::new(s);
        for p in holes {
            w.rapid(p);
            w.plunge(-(s.drill_depth + 1.0)); // through the board into the bed, for the pins
            w.up();
            w.paths.push(vec![p]);
        }
        let feed = format!("F{}", fmt(s.plunge_feed));
        w.lines.iter_mut().filter(|l| l.starts_with("G1 Z")).for_each(|l| *l = l.replace(&feed, &format!("F{}", fmt(s.drill_feed))));
        jobs.push(w.finish(
            JobKind::Alignment,
            format!("Alignment holes · {} mm drill · 2 holes", fmt(s.align_diameter)),
            format!("Drill these first, {} mm below and above the board, then put pins in them.", fmt(s.align_margin)),
        ));
    }
    if let Some(copper) = top {
        jobs.extend(isolation(copper, s, "top"));
    }
    let drills = if bottom_only { mirror_drills(&board.drills, axis) } else { board.drills.clone() };
    jobs.extend(drilling(&drills, s));
    if let Some(copper) = bottom {
        if let Some(mut job) = isolation(&mirror_x(copper, axis), s, "bottom, mirrored") {
            if double {
                job.hint = "Flip the board left-to-right onto the alignment pins first.".into();
            } else if bottom_only {
                notes.push("Bottom-only board: all jobs are mirrored, because you work on its bottom face.".into());
            }
            jobs.push(job);
        }
    }
    match &board.outline {
        Some(outline) => {
            let outline = if bottom_only {
                use geo::MapCoords;
                outline.map_coords(|c| Coord { x: 2.0 * axis - c.x, y: c.y })
            } else {
                outline.clone()
            };
            jobs.extend(cutout(&outline, s));
        }
        None => notes.push("No board outline (Edge_Cuts): no cutout job.".into()),
    }
    if double {
        if let Some(cut) = jobs.iter_mut().find(|j| j.kind == JobKind::Cutout) {
            cut.hint.push_str(" Run it with the board top side up (flip it back).");
        }
    }
    if s.iso_depth > 0.3 {
        notes.push(format!("Isolation depth {} mm is deep for a V-bit; 0.05-0.1 mm is usual.", fmt(s.iso_depth)));
    }
    (jobs, notes)
}

/// One CAM program with a tool section per job (T1, T2, …), each with a comment naming the job.
pub fn program(jobs: &[Job]) -> String {
    let mut out = vec!["(Soft-CNC PCB jobs)".to_string(), "G21".into(), "G90".into()];
    for (index, job) in jobs.iter().enumerate() {
        out.push(format!("T{}", index + 1));
        out.push(format!("(Job: {})", job.title.replace(['(', ')'], "")));
        out.push("M6".into());
        out.extend(job.gcode.iter().cloned());
    }
    out.push("M2".into());
    out.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::excellon::Hole;
    use crate::pcb::geom::circle;
    use geo::Rect;

    fn square_board() -> Board {
        let copper = circle(Coord { x: 10.0, y: 10.0 }, 2.0);
        Board {
            files: Vec::new(),
            top: Some(copper.clone()),
            bottom: Some(crate::pcb::geom::translate(&copper, 5.0, 0.0)),
            outline: Some(Rect::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 30.0, y: 20.0 }).to_polygon()),
            drills: Drills {
                holes: vec![Hole { diameter: 0.8, at: Coord { x: 10.0, y: 10.0 } }, Hole { diameter: 1.0, at: Coord { x: 20.0, y: 5.0 } }],
                slots: Vec::new(),
                warnings: Vec::new(),
            },
            width: 30.0,
            height: 20.0,
            notes: Vec::new(),
        }
    }

    #[test]
    fn isolation_passes_ring_the_copper_at_the_right_distance() {
        let s = Settings::default();
        let (jobs, _) = generate(&square_board(), &s);
        let iso = jobs.iter().find(|j| j.title.starts_with("Isolation, top")).unwrap();
        assert_eq!(iso.paths.len(), 2, "one ring per pass around one round pad");
        let radii: Vec<f64> = iso.paths.iter().map(|ring| dist(ring[0], Coord { x: 10.0, y: 10.0 })).collect();
        // pad radius 1 + tool radius 0.1, then + 0.2 × (1 - 0.4)
        assert!((radii.iter().cloned().fold(f64::MAX, f64::min) - 1.1).abs() < 0.01, "{radii:?}");
        assert!((radii.iter().cloned().fold(0.0, f64::max) - 1.22).abs() < 0.01, "{radii:?}");
        assert!(iso.gcode.contains(&"G1 Z-0.1 F50.0".to_string()));
    }

    #[test]
    fn drilling_makes_one_job_per_size() {
        let (jobs, _) = generate(&square_board(), &Settings::default());
        let drills: Vec<&Job> = jobs.iter().filter(|j| j.kind == JobKind::Drill).collect();
        assert_eq!(drills.len(), 2);
        assert!(drills[0].title.starts_with("Drill 0.8 mm · 1 hole"));
        assert!(drills[0].gcode.contains(&"G1 Z-1.8 F60.0".to_string()), "{:?}", drills[0].gcode);
    }

    #[test]
    fn cutout_has_depth_passes_and_tabs() {
        let s = Settings::default();
        let (jobs, _) = generate(&square_board(), &s);
        let cut = jobs.iter().find(|j| j.kind == JobKind::Cutout).unwrap();
        assert!(cut.title.contains("3 passes, 4 tabs"), "{}", cut.title);
        let depths: Vec<&String> = cut.gcode.iter().filter(|l| l.starts_with("G1 Z")).collect();
        assert!(depths.iter().any(|l| l.starts_with("G1 Z-1.8")));
        assert!(depths.iter().any(|l| l.starts_with("G1 Z-1.2")), "tabs keep 0.6 mm: {depths:?}");
        // the path runs 0.5 mm outside the 30 x 20 board
        let xs: Vec<f64> = cut.paths[0].iter().map(|p| p.x).collect();
        assert!((xs.iter().cloned().fold(f64::MAX, f64::min) + 0.5).abs() < 0.01);
    }

    #[test]
    fn double_sided_mirrors_the_bottom_and_adds_alignment_holes() {
        let s = Settings { double_sided: true, ..Default::default() };
        let (jobs, _) = generate(&square_board(), &s);
        assert_eq!(jobs[0].kind, JobKind::Alignment);
        assert_eq!(jobs[0].paths, [vec![Coord { x: 15.0, y: -5.0 }], vec![Coord { x: 15.0, y: 25.0 }]]);
        let bottom = jobs.iter().find(|j| j.title.contains("bottom")).unwrap();
        // bottom pad centre at x = 15 mirrors about x = 15 onto itself... use its ring centre instead
        let cx = bottom.paths[0].iter().map(|p| p.x).sum::<f64>() / bottom.paths[0].len() as f64;
        assert!((cx - 15.0).abs() < 0.05, "pad at x=15 mirrors onto x=15: {cx}");
        assert!(bottom.hint.contains("Flip"));
    }

    #[test]
    fn program_has_a_section_per_job() {
        let (jobs, _) = generate(&square_board(), &Settings::default());
        let text = program(&jobs);
        assert_eq!(text.matches("\nT").count(), jobs.len());
        let converted = crate::cam::convert(&text, 10.0, 20.0, -15.0, crate::cam::Options::default()).unwrap();
        assert_eq!(converted.sections.len(), jobs.len());
    }
}

#[cfg(test)]
mod edge_tests {
    use super::*;
    use crate::pcb::excellon::{Hole, Slot};
    use crate::pcb::geom::circle;
    use geo::Rect;

    fn board(top: bool, bottom: bool, outline: Option<(f64, f64)>) -> Board {
        let pad = circle(Coord { x: 5.0, y: 5.0 }, 2.0);
        Board {
            files: Vec::new(),
            top: top.then(|| pad.clone()),
            bottom: bottom.then(|| crate::pcb::geom::translate(&pad, 2.0, 0.0)),
            outline: outline.map(|(w, h)| Rect::new(Coord { x: 0.0, y: 0.0 }, Coord { x: w, y: h }).to_polygon()),
            drills: Drills { holes: vec![Hole { diameter: 1.0, at: Coord { x: 3.0, y: 5.0 } }], slots: Vec::new(), warnings: Vec::new() },
            width: outline.map_or(10.0, |o| o.0),
            height: outline.map_or(10.0, |o| o.1),
            notes: Vec::new(),
        }
    }

    fn cutout_job(b: &Board, s: &Settings) -> Job {
        generate(b, s).0.into_iter().find(|j| j.kind == JobKind::Cutout).unwrap()
    }

    #[test]
    fn nearest_ring_first_and_each_ring_closed() {
        let far = vec![Coord { x: 10.0, y: 10.0 }, Coord { x: 11.0, y: 10.0 }, Coord { x: 11.0, y: 11.0 }, Coord { x: 10.0, y: 10.0 }];
        let near = vec![Coord { x: 1.0, y: 1.0 }, Coord { x: 0.5, y: 0.0 }, Coord { x: 1.0, y: 0.0 }, Coord { x: 1.0, y: 1.0 }];
        let ordered = order_rings(vec![far.clone(), near]);
        assert_eq!(ordered[0][0], Coord { x: 0.5, y: 0.0 }, "starts at the vertex closest to the origin");
        for ring in &ordered {
            assert_eq!(ring.first(), ring.last(), "closed");
            assert_eq!(ring.len(), 4);
        }
    }

    #[test]
    fn last_cutout_pass_reaches_the_exact_depth() {
        let s = Settings { cut_depth: 1.7, cut_step: 0.6, tabs: 0, ..Default::default() };
        let job = cutout_job(&board(true, false, Some((20.0, 10.0))), &s);
        let depths: Vec<&String> = job.gcode.iter().filter(|l| l.starts_with("G1 Z")).collect();
        assert_eq!(depths.len(), 3, "{depths:?}");
        assert!(depths[2].starts_with("G1 Z-1.7 "), "{depths:?}");
        assert!(job.title.contains("3 passes, 0 tabs"), "{}", job.title);
    }

    #[test]
    fn tabs_are_dropped_when_they_cannot_fit_or_have_no_height() {
        let small = board(true, false, Some((4.0, 3.0)));
        let crowded = Settings { tabs: 8, tab_width: 3.0, ..Default::default() };
        assert!(cutout_job(&small, &crowded).title.contains("0 tabs"), "8 wide tabs on a tiny board");
        let flat = Settings { tab_height: 0.0, ..Default::default() };
        assert!(cutout_job(&board(true, false, Some((20.0, 10.0))), &flat).title.contains("0 tabs"));
    }

    #[test]
    fn tabs_lift_only_the_deep_passes() {
        let s = Settings { cut_depth: 1.8, cut_step: 0.6, tabs: 2, tab_height: 0.6, ..Default::default() };
        let job = cutout_job(&board(true, false, Some((40.0, 20.0))), &s);
        let zs: Vec<f64> = job.gcode.iter().filter_map(|l| l.strip_prefix("G1 Z")?.split(' ').next()?.parse().ok()).collect();
        assert!(zs.contains(&-1.2), "tabs stay at 1.2 mm deep, leaving 0.6 mm: {zs:?}");
        assert!(zs.iter().all(|z| *z >= -1.8));
    }

    #[test]
    fn single_pass_wording_and_overlap_limit() {
        let s = Settings { iso_passes: 1, ..Default::default() };
        let (jobs, _) = generate(&board(true, false, None), &s);
        assert!(jobs[0].title.ends_with("1 pass"), "{}", jobs[0].title);
        // even 100 % overlap still moves each pass outwards a little
        let s = Settings { iso_passes: 3, iso_overlap: 1.0, ..Default::default() };
        let (jobs, _) = generate(&board(true, false, None), &s);
        let radii: Vec<f64> = jobs[0].paths.iter().map(|r| dist(r[0], Coord { x: 5.0, y: 5.0 })).collect();
        let mut sorted = radii.clone();
        sorted.sort_by(f64::total_cmp);
        sorted.dedup_by(|a, b| (*a - *b).abs() < 1e-3);
        assert_eq!(sorted.len(), 3, "three different rings: {radii:?}");
    }

    #[test]
    fn bottom_only_board_mirrors_everything() {
        let b = board(false, true, Some((10.0, 10.0)));
        let (jobs, notes) = generate(&b, &Settings::default());
        assert!(notes.iter().any(|n| n.contains("Bottom-only board")));
        let drill = jobs.iter().find(|j| j.kind == JobKind::Drill).unwrap();
        assert_eq!(drill.paths[0][0], Coord { x: 7.0, y: 5.0 }, "hole at x=3 mirrors to x=7 on a 10 mm board");
        let iso = jobs.iter().find(|j| j.kind == JobKind::Isolation).unwrap();
        let cx = iso.paths[0].iter().map(|p| p.x).sum::<f64>() / iso.paths[0].len() as f64;
        assert!((cx - 3.0).abs() < 0.05, "pad at x=7 mirrors to x=3: {cx}");
    }

    #[test]
    fn double_sided_without_a_bottom_layer_says_so() {
        let s = Settings { double_sided: true, ..Default::default() };
        let (jobs, notes) = generate(&board(true, false, Some((10.0, 10.0))), &s);
        assert!(!jobs.iter().any(|j| j.kind == JobKind::Alignment));
        assert!(notes.iter().any(|n| n.contains("Double-sided needs both")));
    }

    #[test]
    fn notes_for_missing_outline_and_deep_isolation() {
        let s = Settings { iso_depth: 0.5, ..Default::default() };
        let (jobs, notes) = generate(&board(true, false, None), &s);
        assert!(!jobs.iter().any(|j| j.kind == JobKind::Cutout));
        assert!(notes.iter().any(|n| n.contains("no cutout job")));
        assert!(notes.iter().any(|n| n.contains("deep for a V-bit")));
    }

    #[test]
    fn slots_are_milled_at_the_drill_feed() {
        let mut b = board(true, false, None);
        b.drills.holes.clear();
        b.drills.slots.push(Slot { diameter: 1.2, from: Coord { x: 1.0, y: 1.0 }, to: Coord { x: 3.0, y: 1.0 } });
        let (jobs, _) = generate(&b, &Settings::default());
        let drill = jobs.iter().find(|j| j.kind == JobKind::Drill).unwrap();
        assert_eq!(drill.title, "Drill 1.2 mm · 0 holes + 1 slot");
        assert!(drill.gcode.contains(&"G1 X3.0 Y1.0 F60.0".to_string()), "{:?}", drill.gcode);
        assert!(drill.gcode.contains(&"G1 Z-1.8 F60.0".to_string()));
    }

    #[test]
    fn spindle_speed_is_kept_in_the_machine_range() {
        let s = Settings { spindle: 5000, ..Default::default() };
        let (jobs, _) = generate(&board(true, false, None), &s);
        assert_eq!(jobs[0].gcode[0], "M3 S1400");
    }

    #[test]
    fn empty_copper_makes_no_isolation_job() {
        let mut b = board(true, false, None);
        b.top = Some(MultiPolygon::new(vec![]));
        let (jobs, _) = generate(&b, &Settings::default());
        assert!(!jobs.iter().any(|j| j.kind == JobKind::Isolation));
    }
}
