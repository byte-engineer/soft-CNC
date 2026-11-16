//! PCB files straight to toolpaths, the way FlatCAM does it: read Gerber + Excellon files, work out copper,
//! board outline and holes, then make isolation, drilling and cutout jobs (`toolpath`).

pub mod excellon;
pub mod geom;
pub mod gerber;
pub mod toolpath;

use excellon::Drills;
use geo::{Area, Buffer, BoundingRect, Coord, MapCoords, MultiPolygon, Polygon};
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layer {
    CopperTop,
    CopperBottom,
    Outline,
    Drills,
    Ignored,
}

impl Layer {
    pub fn label(self) -> &'static str {
        match self {
            Layer::CopperTop => "Top copper",
            Layer::CopperBottom => "Bottom copper",
            Layer::Outline => "Board outline",
            Layer::Drills => "Drill holes",
            Layer::Ignored => "Not used",
        }
    }
}

pub struct SourceFile {
    pub name: String,
    pub layer: Layer,
    /// Why this layer was chosen, e.g. "X2 file function: Copper,L1,Top" or "file name".
    pub reason: String,
}

/// Everything read from the files, moved so the board's lower-left corner is (0, 0). Units: mm.
pub struct Board {
    pub files: Vec<SourceFile>,
    pub top: Option<MultiPolygon<f64>>,
    pub bottom: Option<MultiPolygon<f64>>,
    /// The board's shape (inside of the Edge_Cuts line).
    pub outline: Option<Polygon<f64>>,
    pub drills: Drills,
    pub width: f64,
    pub height: f64,
    pub notes: Vec<String>,
}

/// True if the text looks like a Gerber or Excellon file (not G-code).
pub fn is_pcb_file(path: &Path) -> bool {
    let Ok(text) = std::fs::read_to_string(path) else { return false };
    is_gerber(&text) || is_excellon(&text)
}

fn is_gerber(text: &str) -> bool {
    text.contains("%FS") || text.contains("%MO")
}

fn is_excellon(text: &str) -> bool {
    text.lines().any(|l| l.trim() == "M48") || text.lines().any(|l| l.trim().starts_with("T1C") || l.trim().starts_with("T01C"))
}

fn layer_from_name(name: &str) -> Layer {
    let n = name.to_lowercase();
    let ext = n.rsplit('.').next().unwrap_or("");
    if n.contains("f_cu") || n.contains("f.cu") || ext == "gtl" || n.contains("toplayer") || n.contains("copper_top") {
        Layer::CopperTop
    } else if n.contains("b_cu") || n.contains("b.cu") || ext == "gbl" || n.contains("bottomlayer") || n.contains("copper_bottom") {
        Layer::CopperBottom
    } else if n.contains("edge") || matches!(ext, "gko" | "gm1" | "gml" | "gbr_outline") || n.contains("outline") || n.contains("profile") {
        Layer::Outline
    } else if matches!(ext, "drl" | "xln" | "exc" | "drd") {
        Layer::Drills
    } else {
        Layer::Ignored
    }
}

fn layer_from_function(function: &str) -> Layer {
    if let Some(copper) = function.strip_prefix("Copper,") {
        if copper.contains("Top") {
            Layer::CopperTop
        } else if copper.contains("Bot") {
            Layer::CopperBottom
        } else {
            Layer::Ignored // inner layers: a 2-layer mill can't make them
        }
    } else if function.starts_with("Profile") {
        Layer::Outline
    } else {
        Layer::Ignored // solder mask, legend, paste…
    }
}

/// Board shape from the outline layer: lines are drawn centred on the board edge, so the inside of a stroked
/// outline is its outer edge moved in by half the line width.
fn board_shape(outline: &gerber::GerberLayer) -> Option<Polygon<f64>> {
    let largest = outline.image.0.iter().max_by(|a, b| {
        Polygon::new(a.exterior().clone(), vec![]).unsigned_area()
            .total_cmp(&Polygon::new(b.exterior().clone(), vec![]).unsigned_area())
    })?;
    let filled = Polygon::new(largest.exterior().clone(), vec![]);
    if outline.stroke_width <= 0.0 {
        return Some(filled);
    }
    let shrunk = filled.buffer(-outline.stroke_width / 2.0);
    shrunk.0.into_iter().max_by(|a, b| a.unsigned_area().total_cmp(&b.unsigned_area()))
}

pub fn load_board(paths: &[PathBuf]) -> Result<Board, String> {
    let mut files = Vec::new();
    let mut top: Vec<MultiPolygon<f64>> = Vec::new();
    let mut bottom: Vec<MultiPolygon<f64>> = Vec::new();
    let mut outline: Option<Polygon<f64>> = None;
    let mut drills = Drills::default();
    let mut notes = Vec::new();

    for path in paths {
        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let bytes = std::fs::read(path).map_err(|e| format!("Can't read {name}: {e}"))?;
        let text = String::from_utf8_lossy(&bytes);
        if is_gerber(&text) {
            let layer_data = gerber::parse(&text).map_err(|e| format!("{name}: {e}"))?;
            let (layer, reason) = match &layer_data.function {
                Some(f) => (layer_from_function(f), format!("X2 file function: {f}")),
                None => (layer_from_name(&name), "file name".to_string()),
            };
            for warning in &layer_data.warnings {
                notes.push(format!("{name}: {warning}"));
            }
            match layer {
                Layer::CopperTop => top.push(layer_data.image),
                Layer::CopperBottom => bottom.push(layer_data.image),
                Layer::Outline => match board_shape(&layer_data) {
                    Some(shape) => outline = Some(shape),
                    None => notes.push(format!("{name}: the board outline layer is empty")),
                },
                _ => {}
            }
            files.push(SourceFile { name, layer, reason });
        } else if is_excellon(&text) {
            let d = excellon::parse(&text).map_err(|e| format!("{name}: {e}"))?;
            notes.extend(d.warnings.iter().map(|w| format!("{name}: {w}")));
            drills.extend(d);
            files.push(SourceFile { name, layer: Layer::Drills, reason: "drill file".into() });
        } else {
            // e.g. KiCad's *-job.gbrjob, picked along with the Gerbers: skip it, don't fail
            notes.push(format!("{name} skipped: not a Gerber or drill (Excellon) file"));
            files.push(SourceFile { name, layer: Layer::Ignored, reason: "not a Gerber or drill file".into() });
        }
    }

    let top = (!top.is_empty()).then(|| geom::union_all(top));
    let bottom = (!bottom.is_empty()).then(|| geom::union_all(bottom));
    let has_copper = top.as_ref().or(bottom.as_ref()).is_some_and(|c| !c.0.is_empty());
    if !has_copper && drills.is_empty() {
        return Err("No copper layer or drill holes found. Select the copper Gerber (e.g. *-F_Cu.gbr), \
                    and optionally Edge_Cuts and the .drl file."
            .into());
    }

    // Lower-left corner of the board (outline, else everything) becomes (0, 0).
    let mut shapes: Vec<MultiPolygon<f64>> = Vec::new();
    match &outline {
        Some(o) => shapes.push(MultiPolygon::new(vec![o.clone()])),
        None => {
            shapes.extend(top.iter().cloned());
            shapes.extend(bottom.iter().cloned());
            for h in &drills.holes {
                shapes.push(geom::circle(h.at, h.diameter));
            }
            notes.push("No board outline (Edge_Cuts) found: the board size is taken from the copper and holes, \
                        and there is no cutout job.".into());
        }
    }
    let area = geom::bounds(shapes.iter()).ok_or("the files contain nothing to mill")?;
    let (dx, dy) = (-area.min().x, -area.min().y);
    let shift = |c: Coord<f64>| Coord { x: c.x + dx, y: c.y + dy };
    let top = top.map(|m| geom::translate(&m, dx, dy));
    let bottom = bottom.map(|m| geom::translate(&m, dx, dy));
    let outline = outline.map(|o| o.map_coords(shift));
    for h in &mut drills.holes {
        h.at = shift(h.at);
    }
    for s in &mut drills.slots {
        s.from = shift(s.from);
        s.to = shift(s.to);
    }
    let size = outline.as_ref().and_then(|o| o.bounding_rect()).unwrap_or(area);
    Ok(Board {
        files,
        top,
        bottom,
        outline,
        drills,
        width: size.width(),
        height: size.height(),
        notes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layers_from_file_names() {
        let cases = [
            ("board-F_Cu.gbr", Layer::CopperTop),
            ("board.GTL", Layer::CopperTop),
            ("Gerber_TopLayer.GTL", Layer::CopperTop),
            ("board-B_Cu.gbr", Layer::CopperBottom),
            ("board.gbl", Layer::CopperBottom),
            ("board-Edge_Cuts.gbr", Layer::Outline),
            ("board.GKO", Layer::Outline),
            ("board.gm1", Layer::Outline),
            ("board.drl", Layer::Drills),
            ("board.XLN", Layer::Drills),
            ("board-F_Mask.gbr", Layer::Ignored),
            ("board-F_Silkscreen.gbr", Layer::Ignored),
        ];
        for (name, layer) in cases {
            assert_eq!(layer_from_name(name), layer, "{name}");
        }
    }

    #[test]
    fn layers_from_x2_file_functions() {
        assert_eq!(layer_from_function("Copper,L1,Top"), Layer::CopperTop);
        assert_eq!(layer_from_function("Copper,L2,Bot"), Layer::CopperBottom);
        assert_eq!(layer_from_function("Copper,L2,Inr"), Layer::Ignored, "inner layers can't be milled");
        assert_eq!(layer_from_function("Profile,NP"), Layer::Outline);
        assert_eq!(layer_from_function("Soldermask,Top"), Layer::Ignored);
    }

    fn sample(name: &str) -> PathBuf {
        PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/samples/pcb")).join(name)
    }

    #[test]
    fn pcb_files_are_told_apart_from_gcode() {
        assert!(is_pcb_file(&sample("demo-F_Cu.gbr")));
        assert!(is_pcb_file(&sample("demo.drl")));
        assert!(!is_pcb_file(&sample("Custom_Arduino-job.gbrjob")));
        assert!(!is_pcb_file(&PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/cirqoid_square.gcode"))));
        assert!(!is_pcb_file(&sample("does-not-exist.gbr")));
    }

    #[test]
    fn kicad_job_file_selected_with_the_gerbers_is_skipped() {
        let board = load_board(&[sample("demo-F_Cu.gbr"), sample("Custom_Arduino-job.gbrjob")]).unwrap();
        assert_eq!(board.files[1].layer, Layer::Ignored);
        assert!(board.notes.iter().any(|n| n.contains("Custom_Arduino-job.gbrjob skipped")));
    }

    #[test]
    fn nothing_usable_is_an_error() {
        let err = load_board(&[sample("Custom_Arduino-job.gbrjob")]).err().unwrap();
        assert!(err.contains("No copper layer or drill holes found"), "{err}");
    }

    #[test]
    fn an_outline_drawn_as_a_filled_region_is_used_as_is() {
        let gerber = "%TF.FileFunction,Profile,NP*%\n%FSLAX46Y46*%\n%MOMM*%\nG36*\nX0Y0D02*\nX10000000Y0D01*\n\
                      X10000000Y5000000D01*\nX0Y5000000D01*\nX0Y0D01*\nG37*\nM02*\n";
        let layer = gerber::parse(gerber).unwrap();
        let shape = board_shape(&layer).unwrap();
        assert!((shape.unsigned_area() - 50.0).abs() < 1e-6);
    }

    #[test]
    fn a_stroked_outline_is_the_centre_of_the_line() {
        let board = load_board(&[sample("demo-F_Cu.gbr"), sample("demo-Edge_Cuts.gbr")]).unwrap();
        let area = board.outline.unwrap().unsigned_area();
        assert!((area - 600.0).abs() < 0.05, "30 x 20, not 30.05 x 20.05: {area}");
    }

    #[test]
    fn a_missing_file_is_reported() {
        let err = load_board(&[sample("does-not-exist.gbr")]).err().unwrap();
        assert!(err.contains("Can't read does-not-exist.gbr"), "{err}");
    }
}
