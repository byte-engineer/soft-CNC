//! Small G-code text helpers shared by the app, the position tracker and the CAM converter.

use regex::Regex;
use std::sync::LazyLock;

/// Axis words in an upper-cased line: `X12.5`, `Y-3`, `Z.5`.
pub static AXIS_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"([XYZ])\s*(-?\d*\.?\d+)").unwrap());
static COMMENT_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\(.*?\)").unwrap());
static G28_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"(?i)\bG28\b").unwrap());
static Z_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"(?i)Z\s*([-+]?\d*\.?\d+)").unwrap());

pub const AXES: [char; 3] = ['X', 'Y', 'Z'];

pub fn axis_index(axis: char) -> usize {
    match axis.to_ascii_uppercase() {
        'X' => 0,
        'Y' => 1,
        _ => 2,
    }
}

/// X/Y/Z words of an upper-cased line, in order.
pub fn axis_words(upper: &str) -> Vec<(usize, f64)> {
    AXIS_RE
        .captures_iter(upper)
        .filter_map(|c| Some((axis_index(c[1].chars().next()?), c[2].parse().ok()?)))
        .collect()
}

/// 3 decimals, trailing zeros trimmed, but always with a decimal point (the Cirqoid needs one).
pub fn fmt(value: f64) -> String {
    let mut text = format!("{value:.3}");
    while text.ends_with('0') {
        text.pop();
    }
    if text.ends_with('.') {
        text.push('0');
    }
    if text == "-0.0" {
        "0.0".into()
    } else {
        text
    }
}

/// Like Python's `:g` for the values we show: `fmt` without a trailing `.0` (`120`, `-26.9`, `0.835`).
pub fn short(value: f64) -> String {
    let text = fmt(value);
    text.strip_suffix(".0").map(str::to_string).unwrap_or(text)
}

/// The Cirqoid rejects whole numbers on coordinates/feed: turn `X5 F1200` into `X5.0 F1200.0`.
/// G/M numbers and S (spindle speed, read as an integer) are left alone.
pub fn decimal_words(line: &str) -> String {
    let chars: Vec<char> = line.chars().collect();
    let mut out = String::with_capacity(line.len() + 8);
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        out.push(c);
        i += 1;
        if !"XYZAIJKRFxyzaijkrf".contains(c) {
            continue;
        }
        let mut j = i;
        while j < chars.len() && chars[j].is_whitespace() {
            j += 1;
        }
        if j < chars.len() && chars[j] == '-' {
            j += 1;
        }
        let digits = j;
        while j < chars.len() && chars[j].is_ascii_digit() {
            j += 1;
        }
        if j == digits || (j < chars.len() && chars[j] == '.') {
            continue; // no number here, or it already has a decimal point
        }
        out.extend(&chars[i..j]);
        out.push_str(".0");
        i = j;
    }
    out
}

/// Strip comments and blank lines from a G-code program.
pub fn clean_gcode(text: &str) -> Vec<String> {
    text.lines()
        .filter_map(|raw| {
            let before_semicolon = raw.split(';').next().unwrap_or("");
            let line = COMMENT_RE.replace_all(before_semicolon, "").trim().to_string();
            (!line.is_empty() && line != "%").then_some(line)
        })
        .collect()
}

/// True for G-code from a CAM tool (Z0 = copper surface): it goes above Z0 and never homes the Cirqoid.
pub fn looks_like_cam(lines: &[String]) -> bool {
    let homes = lines.iter().any(|line| G28_RE.is_match(line));
    let above = lines.iter().any(|line| {
        Z_RE.captures_iter(line)
            .any(|c| c[1].parse::<f64>().is_ok_and(|z| z > 0.0))
    });
    above && !homes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fmt_always_has_a_decimal_point() {
        let got: Vec<String> = [5.0, 1200.0, -0.0, 0.8351, -1.8, 12.3456].iter().map(|v| fmt(*v)).collect();
        assert_eq!(got, ["5.0", "1200.0", "0.0", "0.835", "-1.8", "12.346"]);
        assert_eq!(short(120.0), "120");
        assert_eq!(short(-26.9), "-26.9");
    }

    #[test]
    fn decimal_words_only_touches_whole_numbers_on_coordinates_and_feed() {
        let cases = [
            ("G0 X1 F500", "G0 X1.0 F500.0"),
            ("G0 Y-10 F500", "G0 Y-10.0 F500.0"),
            ("G92 X0 Y0 Z0", "G92 X0.0 Y0.0 Z0.0"),
            ("G28 Y0.835", "G28 Y0.835"),
            ("G0 Z25.0 F300.0", "G0 Z25.0 F300.0"),
            ("G90", "G90"),
            ("M3 S1000", "M3 S1000"),
            ("G2 X10 Y0 I5 J0 F300", "G2 X10.0 Y0.0 I5.0 J0.0 F300.0"),
            ("g0 x5 f100", "g0 x5.0 f100.0"),
            ("G0 X10.5", "G0 X10.5"),
        ];
        for (src, want) in cases {
            assert_eq!(decimal_words(src), want, "{src}");
        }
    }

    #[test]
    fn clean_gcode_strips_comments() {
        let lines = clean_gcode("G90 ; abs\n(comment)\nG1 X1 (inline) Y1\n\n%\n");
        assert_eq!(lines, ["G90", "G1 X1  Y1"]);
    }

    #[test]
    fn cam_files_are_recognised() {
        let cam: Vec<String> = ["G21", "G0 Z2.0", "G1 Z-0.2"].map(String::from).to_vec();
        let ready: Vec<String> = ["G90", "G28 Y0.835", "G0 Z0.0"].map(String::from).to_vec();
        assert!(looks_like_cam(&cam));
        assert!(!looks_like_cam(&ready));
    }
}

#[cfg(test)]
mod edge_tests {
    use super::*;

    #[test]
    fn axis_words_read_signs_and_bare_decimals() {
        assert_eq!(axis_words("G0 X-1.5 Y.5 Z2"), [(0, -1.5), (1, 0.5), (2, 2.0)]);
        assert_eq!(axis_words("M3 S1000 F200"), []);
    }

    #[test]
    fn rounding_and_negative_zero() {
        assert_eq!(fmt(-0.0004), "0.0");
        assert_eq!(fmt(2.9999), "3.0");
        assert_eq!(short(0.0), "0");
        assert_eq!(short(224.835), "224.835");
    }

    #[test]
    fn decimal_words_spacing_and_already_decimal_numbers() {
        assert_eq!(decimal_words("G1 X 5 Y-2.5"), "G1 X 5.0 Y-2.5");
        assert_eq!(decimal_words("G1X5Y6F100"), "G1X5.0Y6.0F100.0");
        assert_eq!(decimal_words("G1 X-0"), "G1 X-0.0");
        assert_eq!(decimal_words(""), "");
    }

    #[test]
    fn cam_detection_edge_cases() {
        let lines = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert!(!looks_like_cam(&lines(&["G0 Z0", "G1 Z-1"])), "never above zero: machine coordinates");
        assert!(!looks_like_cam(&lines(&["G28", "G0 Z5"])), "homes itself: a machine file");
        assert!(looks_like_cam(&lines(&["g0 z+2.5"])), "lower case and + sign");
        assert!(!looks_like_cam(&[]));
    }
}
