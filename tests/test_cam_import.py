import math
import re
from pathlib import Path

import pytest

import cnc_ui
from cam_import import ConvertError, Options, convert, fmt

SAMPLES = Path(__file__).parent / "samples"
ORIGIN_X, ORIGIN_Y, SURFACE_Z = 20.0, 30.0, -20.0
MOVE_RE = re.compile(r"^G([01]) (.*) F(-?\d+\.\d+)$")


def moves(lines):
    """Parse converted lines into (kind, {axis: value}, feed)."""
    out = []
    for line in lines:
        match = MOVE_RE.match(line)
        if match:
            words = {a: float(v) for a, v in re.findall(r"([XYZ])(-?\d+\.\d+)", match.group(2))}
            out.append((int(match.group(1)), words, float(match.group(3))))
    return out


def convert_sample(name, **kw):
    return convert((SAMPLES / name).read_text(), ORIGIN_X, ORIGIN_Y, SURFACE_Z, Options(**kw))


def test_fmt_always_has_a_decimal_point():
    assert [fmt(v) for v in (5, 1200, -0.0, 0.8351, -1.8, 12.3456)] == \
        ["5.0", "1200.0", "0.0", "0.835", "-1.8", "12.346"]


def test_words_without_spaces_and_origin_surface_mapping():
    result = convert("G21\nG90\nG00 X10.0000Y10.0000\nG01 F100\nG01 Z-0.0500\n", ORIGIN_X, ORIGIN_Y, SURFACE_Z)
    lines = result.sections[0].lines
    assert "G0 X30.0 Y40.0 F1200.0" in lines
    assert "G1 Z-20.05 F100.0" in lines


def test_every_number_has_a_decimal_point_and_every_move_a_feed():
    result = convert_sample("flatcam_isolation.nc")
    for section in result.sections:
        for line in section.lines:
            assert not re.search(r"[XYZF]-?\d+(?![\d.])", line), line
            if line.startswith(("G0 ", "G1 ")):
                assert " F" in line, line


def test_z_above_machine_top_is_clamped_with_warning():
    result = convert_sample("flatcam_isolation.nc")  # Z15 tool-change height maps to -5; fine
    assert all(m[1].get("Z", -1) <= 0 for s in result.sections for m in moves(s.lines))
    high = convert("G0 Z30\nG1 F100 X1 Z-0.1\n", 0, 0, -20.0)
    assert any("clamped" in w for w in high.warnings)
    assert "G0 Z0.0 F300.0" in high.sections[0].lines


def test_air_offset_lifts_cutting_depth():
    normal = convert("G1 F100 X1 Z-0.1\n", 0, 0, -20.0)
    air = convert("G1 F100 X1 Z-0.1\n", 0, 0, -20.0, Options(air_offset=5.0))
    cut = lambda result: [w for kind, w, _ in moves(result.sections[0].lines) if kind == 1][-1]
    assert cut(normal)["Z"] == pytest.approx(-20.1)
    assert cut(air)["Z"] == pytest.approx(-15.1)


def test_inches_converted_to_mm():
    result = convert("G20\nG90\nG1 F10 X1 Y0.5\n", 0, 0, -20.0)
    assert moves(result.sections[0].lines)[-1] == (1, {"X": 25.4, "Y": 12.7}, 254.0)


def test_z_moves_are_capped_at_z_feed():
    result = convert("G1 F1000 X5\nG1 Z-0.2\nG0 Z5\nG0 X0 Y0\n", 0, 0, -20.0)
    feeds = {tuple(sorted(w)): f for _, w, f in moves(result.sections[0].lines)}
    assert feeds[("X",)] == 1000.0
    assert feeds[("Z",)] == 300.0
    assert feeds[("X", "Y")] == 1200.0


def test_arc_is_linearized_within_tolerance_and_ends_exactly():
    result = convert_sample("flatcam_isolation.nc")
    lines = result.sections[0].lines
    assert not any(re.match(r"G0?[23] ", line) for line in lines)
    # Arc from (30,20) to (20,30) around centre (20,20), r = 10, in CAM coordinates.
    cx, cy = 20 + ORIGIN_X, 20 + ORIGIN_Y
    pts = [w for _, w, _ in moves(lines) if "X" in w and "Y" in w]
    start = pts.index({"X": 50.0, "Y": 50.0})
    end = pts.index({"X": 40.0, "Y": 60.0})
    arc = pts[start:end + 1]
    assert len(arc) > 5
    for p in arc:
        assert math.hypot(p["X"] - cx, p["Y"] - cy) == pytest.approx(10.0, abs=0.002)
    for a, b in zip(arc, arc[1:]):  # chord sagitta within the 0.01 tolerance, plus output rounding to 0.001
        mid = ((a["X"] + b["X"]) / 2, (a["Y"] + b["Y"]) / 2)
        assert 10.0 - math.hypot(mid[0] - cx, mid[1] - cy) <= 0.011


def test_arc_with_radius_word_matches_ij_arc():
    ij = convert("G0 X10 Y0\nG1 F100\nG3 X0 Y10 I-10 J0\n", 0, 0, -20.0)
    r = convert("G0 X10 Y0\nG1 F100\nG3 X0 Y10 R10\n", 0, 0, -20.0)
    assert moves(ij.sections[0].lines) == moves(r.sections[0].lines)


def test_tool_changes_split_sections_and_drill_cycles_expand():
    result = convert_sample("pcb2gcode_drill.ngc")
    assert [s.tool for s in result.sections] == ["T1", "T2"]
    assert "0.8" in result.sections[0].note
    t1 = moves(result.sections[0].lines)
    holes = [w for _, w, _ in t1 if w.keys() == {"X", "Y"}]
    assert holes == [{"X": 25.0, "Y": 35.0}, {"X": 30.0, "Y": 35.0}]
    assert (1, {"Z": -21.8}, 150.0) in t1  # drilled to Z-1.8 below the surface
    # G83 with Q0.6 from R1 to Z-1.8: pecks at 0.4, -0.2, -0.8, -1.4, -1.8
    t2_depths = [w["Z"] for kind, w, _ in moves(result.sections[1].lines) if kind == 1]
    assert t2_depths == pytest.approx([-19.6, -20.2, -20.8, -21.4, -21.8])
    assert result.sections[1].lines[:3] == ["G90", "G21", "G0 Z0.0 F300.0"]
    assert result.sections[1].lines[-2:] == ["M5", "G0 Z0.0 F300.0"]


def test_spindle_speed_is_clamped():
    result = convert_sample("pcb2gcode_drill.ngc")
    assert "M3 S1400" in result.sections[0].lines
    assert any("S10000 limited to S1400" in w for w in result.warnings)


def test_section_note_prefers_the_tool_diameter():
    assert convert_sample("flatcam_isolation.nc").sections[0].note == "Tool Diameter: 0.1 mm"


def test_relative_moves_are_refused():
    with pytest.raises(ConvertError, match="G91"):
        convert("G91\nG1 X1 F100\n", 0, 0, -20.0)


def test_unknown_g_code_is_refused():
    with pytest.raises(ConvertError, match="G53"):
        convert("G53 G0 X1\n", 0, 0, -20.0)


def test_output_passes_the_apps_soft_limits():
    for name in ("flatcam_isolation.nc", "pcb2gcode_drill.ngc"):
        for section in convert_sample(name).sections:
            tracker = cnc_ui.PositionTracker(cnc_ui.SOFT_LIMITS["Cirqoid"])
            tracker.apply("G28 Y0.835")
            for line in section.lines:
                assert tracker.apply(cnc_ui.decimal_words(line)) is None, line
                assert cnc_ui.decimal_words(line) == line  # already in the Cirqoid dialect


def test_out_of_range_origin_is_caught_by_soft_limits():
    section = convert_sample("flatcam_isolation.nc").sections[0]
    far = convert((SAMPLES / "flatcam_isolation.nc").read_text(), 90.0, 30.0, SURFACE_Z).sections[0]
    tracker = cnc_ui.PositionTracker(cnc_ui.SOFT_LIMITS["Cirqoid"])
    tracker.apply("G28 Y0.835")
    errors = [tracker.apply(line) for line in far.lines]
    assert any(e and "X would go to" in e for e in errors)
    assert section.lines != far.lines
