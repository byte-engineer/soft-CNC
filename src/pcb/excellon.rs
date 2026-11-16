//! Excellon drill-file reader (KiCad, EasyEDA, Eagle…): tools, holes and slots, in mm.

use geo::Coord;
use regex::Regex;
use std::collections::BTreeMap;
use std::sync::LazyLock;

static TOOL_DEF_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"^T(\d+)[^C]*C([\d.]+)").unwrap());
static TOOL_SEL_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"^T(\d+)$").unwrap());
static XY_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"([XY])([+-]?[\d.]+)").unwrap());

#[derive(Debug, Clone, PartialEq)]
pub struct Hole {
    pub diameter: f64,
    pub at: Coord<f64>,
}

/// An oval hole: the drill moves from `from` to `to`.
#[derive(Debug, Clone, PartialEq)]
pub struct Slot {
    pub diameter: f64,
    pub from: Coord<f64>,
    pub to: Coord<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct Drills {
    pub holes: Vec<Hole>,
    pub slots: Vec<Slot>,
    pub warnings: Vec<String>,
}

impl Drills {
    pub fn is_empty(&self) -> bool {
        self.holes.is_empty() && self.slots.is_empty()
    }

    pub fn extend(&mut self, other: Drills) {
        self.holes.extend(other.holes);
        self.slots.extend(other.slots);
        self.warnings.extend(other.warnings);
    }
}

struct Format {
    inch: bool,
    leading_zeros_kept: bool, // "LZ": leading zeros present, trailing ones dropped
    int_digits: usize,
    dec_digits: usize,
}

impl Format {
    fn unit(&self) -> f64 {
        if self.inch { 25.4 } else { 1.0 }
    }

    fn coord(&self, text: &str) -> Option<f64> {
        let value = if text.contains('.') {
            text.parse::<f64>().ok()?
        } else {
            let negative = text.starts_with('-');
            let mut digits = text.trim_start_matches(['+', '-']).to_string();
            if self.leading_zeros_kept {
                while digits.len() < self.int_digits + self.dec_digits {
                    digits.push('0');
                }
            }
            let v = digits.parse::<f64>().ok()? / 10f64.powi(self.dec_digits as i32);
            if negative { -v } else { v }
        };
        Some(value * self.unit())
    }
}

pub fn parse(text: &str) -> Result<Drills, String> {
    let mut format = Format { inch: false, leading_zeros_kept: false, int_digits: 3, dec_digits: 3 };
    let mut tools: BTreeMap<u32, f64> = BTreeMap::new();
    let mut drills = Drills::default();
    let mut current: Option<f64> = None;
    let mut pos = Coord { x: 0.0, y: 0.0 };
    let mut routing = false; // between M15 (tool down) and M16 (tool up) in route mode
    let mut in_header = false;

    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with(';') {
            if line.contains("FORMAT") && line.contains("inch") {
                format.inch = true;
            }
            continue;
        }
        match line {
            "M48" => {
                in_header = true;
                continue;
            }
            "%" | "M95" => {
                in_header = false;
                continue;
            }
            "M30" | "M00" => break,
            "M71" | "METRIC" => {
                format.inch = false;
                continue;
            }
            "M72" | "INCH" => {
                format.inch = true;
                continue;
            }
            "M15" => {
                routing = true;
                continue;
            }
            "M16" | "M17" => {
                routing = false;
                continue;
            }
            _ => {}
        }
        if line.starts_with("METRIC") || line.starts_with("INCH") {
            format.inch = line.starts_with("INCH");
            format.leading_zeros_kept = line.contains("LZ");
            let (i, d) = if format.inch { (2, 4) } else { (3, 3) };
            format.int_digits = i;
            format.dec_digits = d;
            if let Some(f) = line.split(',').find(|p| p.contains('.') && p.chars().all(|c| c == '0' || c == '.')) {
                let (int, dec) = f.split_once('.').unwrap();
                format.int_digits = int.len();
                format.dec_digits = dec.len();
            }
            continue;
        }
        if let Some(c) = TOOL_DEF_RE.captures(line) {
            let diameter: f64 = c[2].parse().map_err(|_| format!("bad tool size in '{line}'"))?;
            tools.insert(c[1].parse().unwrap(), diameter * format.unit());
            continue;
        }
        if in_header {
            continue;
        }
        if let Some(c) = TOOL_SEL_RE.captures(line) {
            let number: u32 = c[1].parse().unwrap();
            current = if number == 0 { None } else { tools.get(&number).copied() };
            if number != 0 && current.is_none() {
                drills.warnings.push(format!("tool T{number} is used but has no size; its holes are skipped"));
            }
            continue;
        }
        if line.starts_with('X') || line.starts_with('Y') || line.starts_with("G00") || line.starts_with("G01")
            || line.starts_with("G0X") || line.starts_with("G1X") {
            let Some(diameter) = current else { continue };
            // "X1Y2G85X3Y4" is a slot from (1,2) to (3,4)
            let (first, second) = match line.split_once("G85") {
                Some((a, b)) => (a, Some(b)),
                None => (line, None),
            };
            let read = |part: &str, base: Coord<f64>| -> Coord<f64> {
                let mut at = base;
                for c in XY_RE.captures_iter(part) {
                    if let Some(v) = format.coord(&c[2]) {
                        if &c[1] == "X" { at.x = v } else { at.y = v }
                    }
                }
                at
            };
            let at = read(first, pos);
            if let Some(second) = second {
                let to = read(second, at);
                drills.slots.push(Slot { diameter, from: at, to });
                pos = to;
            } else if line.starts_with("G01") || line.starts_with("G1X") {
                if routing {
                    drills.slots.push(Slot { diameter, from: pos, to: at });
                }
                pos = at;
            } else if line.starts_with("G00") || line.starts_with("G0X") {
                pos = at; // route mode: move to the start of a slot
            } else {
                drills.holes.push(Hole { diameter, at });
                pos = at;
            }
        }
    }
    if drills.is_empty() {
        return Err("no holes found in the drill file".into());
    }
    Ok(drills)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kicad_decimal_metric_file() {
        let text = "M48\n; DRILL file {KiCad 10} date\n;FORMAT={-:-/ absolute / metric / decimal}\nFMAT,2\nMETRIC\n\
                    T1C0.800\nT2C1.000\n%\nG90\nG05\nT1\nX125.73Y-80.01\nX128.27Y-80.01\nT2\nX130.0Y-82.0\n\
                    X131.0Y-85.0G85X133.0Y-85.0\nT0\nM30\n";
        let d = parse(text).unwrap();
        assert_eq!(d.holes.len(), 3);
        assert_eq!(d.holes[0], Hole { diameter: 0.8, at: Coord { x: 125.73, y: -80.01 } });
        assert_eq!(d.slots, [Slot { diameter: 1.0, from: Coord { x: 131.0, y: -85.0 }, to: Coord { x: 133.0, y: -85.0 } }]);
    }

    #[test]
    fn inch_file_without_decimal_points() {
        // INCH with leading zeros kept, 2.4 format: X012345 = 1.2345 in
        let text = "M48\nINCH,LZ\nT01C0.0315\n%\nT01\nX012345Y005\nM30\n";
        let d = parse(text).unwrap();
        let h = &d.holes[0];
        assert!((h.diameter - 0.8001).abs() < 1e-3);
        assert!((h.at.x - 1.2345 * 25.4).abs() < 1e-6 && (h.at.y - 0.5 * 25.4).abs() < 1e-6, "{h:?}");
    }

    #[test]
    fn route_mode_slots() {
        let text = "M48\nMETRIC\nT3C1.2\n%\nT3\nG00X10.0Y10.0\nM15\nG01X14.0Y10.0\nM16\nG05\nM30\n";
        let d = parse(text).unwrap();
        assert_eq!(d.slots.len(), 1);
        assert_eq!(d.slots[0].to, Coord { x: 14.0, y: 10.0 });
    }
}

#[cfg(test)]
mod edge_tests {
    use super::*;

    #[test]
    fn tool_zero_and_unknown_tools() {
        let d = parse("M48\nMETRIC\nT1C0.8\n%\nT1\nX1.0Y1.0\nT0\nX2.0Y2.0\nT5\nX3.0Y3.0\nT1\nX4.0Y4.0\nM30\n").unwrap();
        let xs: Vec<f64> = d.holes.iter().map(|h| h.at.x).collect();
        assert_eq!(xs, [1.0, 4.0], "holes after T0 or an undefined tool are skipped");
        assert!(d.warnings.iter().any(|w| w.contains("T5")));
    }

    #[test]
    fn a_file_without_holes_is_an_error() {
        assert!(parse("M48\nMETRIC\nT1C0.8\n%\nM30\n").unwrap_err().contains("no holes"));
    }

    #[test]
    fn units_from_comments_and_m71_m72() {
        let d = parse("M48\n;FORMAT={-:-/ absolute / inch / decimal}\nT1C0.0315\n%\nT1\nX1.0Y0.5\nM30\n").unwrap();
        assert!((d.holes[0].at.x - 25.4).abs() < 1e-9 && (d.holes[0].diameter - 0.8001).abs() < 1e-3);
        let d = parse("M48\nM72\nT1C0.04\n%\nT1\nX1.0Y1.0\nM71\nX10.0Y10.0\nM30\n").unwrap();
        assert!((d.holes[0].at.x - 25.4).abs() < 1e-9, "inch");
        assert!((d.holes[1].at.x - 10.0).abs() < 1e-9, "metric after M71");
    }

    #[test]
    fn trailing_zero_format_and_tool_parameters() {
        // INCH,TZ: leading zeros dropped, 2.4 digits: X12345 = 1.2345 in
        let d = parse("M48\nINCH,TZ\nT01F00S00C0.0400\n%\nT01\nX12345Y5000\nM30\n").unwrap();
        assert!((d.holes[0].at.x - 1.2345 * 25.4).abs() < 1e-6, "{:?}", d.holes[0]);
        assert!((d.holes[0].at.y - 0.5 * 25.4).abs() < 1e-6);
        assert!((d.holes[0].diameter - 1.016).abs() < 1e-6, "C after F/S parameters");
    }

    #[test]
    fn explicit_digit_format_in_the_header() {
        // METRIC,LZ,000.000: leading zeros kept, 3.3 digits: X001500 = 1.5 mm
        let d = parse("M48\nMETRIC,LZ,000.000\nT1C1.0\n%\nT1\nX001500Y0025\nM30\n").unwrap();
        assert!((d.holes[0].at.x - 1.5).abs() < 1e-9 && (d.holes[0].at.y - 2.5).abs() < 1e-9, "{:?}", d.holes[0]);
    }

    #[test]
    fn coordinates_are_modal() {
        let d = parse("M48\nMETRIC\nT1C1.0\n%\nT1\nX1.0Y2.0\nX3.0\nY5.0\nM30\n").unwrap();
        let points: Vec<(f64, f64)> = d.holes.iter().map(|h| (h.at.x, h.at.y)).collect();
        assert_eq!(points, [(1.0, 2.0), (3.0, 2.0), (3.0, 5.0)]);
    }

    #[test]
    fn route_moves_without_plunge_are_not_slots() {
        let d = parse("M48\nMETRIC\nT1C1.0\n%\nT1\nX1.0Y1.0\nG00X5.0Y5.0\nG01X9.0Y5.0\nM30\n").unwrap();
        assert!(d.slots.is_empty(), "G01 without M15 only moves");
    }
}
