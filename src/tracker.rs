//! Follows the position from the absolute moves we send, and enforces soft limits.

use crate::gcode::{axis_words, short, AXES};
use regex::Regex;
use std::sync::LazyLock;

/// (min, max) per axis X, Y, Z in mm, measured from home.
pub type Limits = [(f64, f64); 3];

static MOVE_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\bG0?[0-3]\b|^[XYZ]").unwrap());
static G28_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\bG28\b").unwrap());
static G92_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\bG92\b").unwrap());

/// Positions are machine coordinates (from home). G92 sets a work offset: work = machine - offset.
#[derive(Clone, Debug, Default)]
pub struct PositionTracker {
    pub limits: Option<Limits>,
    pub machine: Option<[f64; 3]>, // unknown until homed
    pub offset: [f64; 3],
}

impl PositionTracker {
    pub fn new(limits: Option<Limits>) -> Self {
        Self { limits, machine: None, offset: [0.0; 3] }
    }

    pub fn work(&self) -> Option<[f64; 3]> {
        self.machine.map(|m| [m[0] - self.offset[0], m[1] - self.offset[1], m[2] - self.offset[2]])
    }

    pub fn work_range(&self, axis: usize) -> Option<(f64, f64)> {
        self.limits.map(|l| (l[axis].0 - self.offset[axis], l[axis].1 - self.offset[axis]))
    }

    /// Update from one line. Returns an error message, and changes nothing, if the line isn't allowed.
    pub fn apply(&mut self, line: &str) -> Result<(), String> {
        let upper = line.to_uppercase();
        let words = axis_words(&upper);
        if G28_RE.is_match(&upper) {
            let mut home = [0.0; 3];
            for (axis, value) in &words {
                home[*axis] = *value;
            }
            self.machine = Some(home);
            self.offset = [0.0; 3];
        } else if G92_RE.is_match(&upper) {
            let machine = self.machine.ok_or("home the machine first (G92 needs a known position)")?;
            for (axis, value) in &words {
                self.offset[*axis] = machine[*axis] - value;
            }
        } else if !words.is_empty() && MOVE_RE.is_match(&upper) {
            let mut machine = self.machine.ok_or("position unknown - home the machine first")?;
            for (axis, value) in &words {
                let pos = value + self.offset[*axis];
                if let Some(limits) = self.limits {
                    let (lo, hi) = limits[*axis];
                    if !(lo - 1e-6..=hi + 1e-6).contains(&pos) {
                        return Err(format!(
                            "{} would go to {} mm from home; soft limit is {} to {}",
                            AXES[*axis], short(pos), short(lo), short(hi)
                        ));
                    }
                }
                machine[*axis] = pos;
            }
            self.machine = Some(machine);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::firmware::CIRQOID;

    fn homed() -> PositionTracker {
        let mut t = PositionTracker::new(CIRQOID.soft_limits);
        t.apply("G28 Y0.835").unwrap();
        t
    }

    #[test]
    fn moves_need_homing_first() {
        let mut t = PositionTracker::new(CIRQOID.soft_limits);
        assert!(t.apply("G0 X10.0").unwrap_err().contains("home"));
        assert_eq!(homed().work(), Some([0.0, 0.835, 0.0]));
    }

    #[test]
    fn soft_limits_refuse_and_keep_the_position() {
        let mut t = homed();
        let err = t.apply("G0 Z1.0").unwrap_err();
        assert_eq!(err, "Z would go to 1 mm from home; soft limit is -26.9 to 0");
        assert!(t.apply("G0 X120.0").unwrap_err().contains("X would go to 120"));
        t.apply("G0 X103.0 Y224.835 Z-26.9").unwrap();
        assert_eq!(t.work(), Some([103.0, 224.835, -26.9]));
    }

    #[test]
    fn g92_offsets_still_check_the_real_position() {
        let mut t = homed();
        t.apply("G0 X50.0").unwrap();
        t.apply("G92 X0.0").unwrap();
        assert_eq!(t.work().unwrap()[0], 0.0);
        assert_eq!(t.work_range(0), Some((-50.0, 53.0)));
        assert!(t.apply("G0 X60.0").is_err()); // 110 mm from home
    }
}

#[cfg(test)]
mod edge_tests {
    use super::*;

    #[test]
    fn homing_with_words_sets_those_positions() {
        let mut t = PositionTracker::new(None);
        t.apply("G28 X1 Y2.5").unwrap();
        assert_eq!(t.work(), Some([1.0, 2.5, 0.0]));
    }

    #[test]
    fn non_moves_change_nothing() {
        let mut t = PositionTracker::new(None);
        t.apply("G28").unwrap();
        for line in ["G90", "G21", "M3 S1000", "M5", "G4 P1"] {
            t.apply(line).unwrap();
        }
        assert_eq!(t.work(), Some([0.0, 0.0, 0.0]));
    }

    #[test]
    fn modal_axis_lines_and_lower_case_count_as_moves() {
        let mut t = PositionTracker::new(None);
        t.apply("G28").unwrap();
        t.apply("X5 Y6").unwrap();
        t.apply("g1 z-2").unwrap();
        assert_eq!(t.work(), Some([5.0, 6.0, -2.0]));
    }

    #[test]
    fn without_limits_anything_goes_and_there_is_no_range() {
        let mut t = PositionTracker::new(None);
        t.apply("G28").unwrap();
        t.apply("G0 X5000 Z50").unwrap();
        assert_eq!(t.work_range(0), None);
    }

    #[test]
    fn g92_before_homing_is_refused() {
        let mut t = PositionTracker::new(None);
        assert!(t.apply("G92 X0").unwrap_err().contains("home the machine first"));
    }

    #[test]
    fn a_refused_move_changes_nothing_even_if_another_axis_was_fine() {
        let mut t = PositionTracker::new(crate::firmware::CIRQOID.soft_limits);
        t.apply("G28 Y0.835").unwrap();
        assert!(t.apply("G0 X10.0 Z1.0").is_err());
        assert_eq!(t.work(), Some([0.0, 0.835, 0.0]), "X must not have moved either");
    }
}
