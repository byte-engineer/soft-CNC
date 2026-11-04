"""Convert G-code from PCB CAM tools (FlatCAM, pcb2gcode) into the dialect the Cirqoid accepts.

CAM files put Z0 on the copper surface (positive = up) and X0/Y0 on the board corner. The Cirqoid needs
machine coordinates (Z 0 at the top, negative going down), absolute moves only, a decimal point on every
number and an F on every move. Tool changes split the file into sections, because each bit has to be
touched off on the copper again before its part of the job runs.
"""

import math
import re
from dataclasses import dataclass, field

AXES = "XYZ"


class ConvertError(Exception):
    """The file uses something the Cirqoid can't do (e.g. relative moves)."""


@dataclass
class Options:
    air_offset: float = 0.0  # lift the whole job by this much (mm) for a dry run in the air
    rapid_xy_feed: float = 1200.0  # feed written on G0 moves
    rapid_z_feed: float = 300.0  # feed for G0 moves that only change Z
    max_z_feed: float = 300.0  # any move that changes Z is capped at this
    max_spindle: int = 1400
    arc_tolerance: float = 0.01  # mm, largest gap between an arc and its straight chords
    default_feed: float = 300.0  # G1 feed when the file never sets one


@dataclass
class Section:
    tool: str
    lines: list = field(default_factory=list)
    note: str = ""  # e.g. a tool-diameter comment from the CAM file
    moves: int = 0
    bounds: dict = field(default_factory=dict)  # axis -> (min, max), machine coordinates


@dataclass
class ConvertResult:
    sections: list
    warnings: list
    bounds: dict  # axis -> (min, max) in machine coordinates, over all sections


def fmt(value):
    """3 decimals, trailing zeros trimmed, but always with a decimal point (the Cirqoid needs one)."""
    text = f"{value:.3f}".rstrip("0")
    text = text + "0" if text.endswith(".") else text
    return "0.0" if text == "-0.0" else text


WORD_RE = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def _note_rank(comment):
    """How well a comment describes the tool: 3 = diameter, 2 = tool/drill with a size in mm, 1 = tool/drill."""
    if re.search(r"dia", comment, re.IGNORECASE):
        return 3
    if re.search(r"tool|drill|bit", comment, re.IGNORECASE):
        return 2 if re.search(r"\d\s*mm", comment, re.IGNORECASE) and "change" in comment.lower() else 1
    return 0


def _words(line):
    return [(letter.upper(), float(value)) for letter, value in WORD_RE.findall(line)]


class _Converter:
    def __init__(self, origin_x, origin_y, surface_z, options):
        self.offset = {"X": origin_x, "Y": origin_y, "Z": surface_z + options.air_offset}
        self.opt = options
        self.scale = 1.0  # 25.4 after G20
        self.motion = None  # 0, 1, 2, 3, 81, 82, 83
        self.feed = None  # mm/min, already scaled
        self.cam = {axis: None for axis in AXES}  # current position in CAM coordinates (mm)
        self.machine = {axis: None for axis in AXES}  # last position written, machine coordinates
        self.arc_absolute = False  # G90.1
        self.retract_to_r = True  # G99 (default) vs G98
        self.cycle = {}  # modal canned-cycle words: Z, R, P, Q
        self.speed = None
        self.spindle_on = False
        self.warnings = {}
        self.bounds = {}
        self.sections = []
        self.section = None
        self._new_section("(no tool)")

    # ---------- helpers ----------
    def warn(self, message):
        self.warnings[message] = self.warnings.get(message, 0) + 1

    def emit(self, line):
        self.section.lines.append(line)

    def _new_section(self, tool):
        """Start a new section, or just rename the current one if it hasn't moved yet."""
        if self.section and self.section.moves == 0:
            self.section.tool = tool
            return
        if self.section:
            self._close_section()
        self.section = Section(tool)
        self.sections.append(self.section)
        self.machine = {axis: None for axis in AXES}  # each section runs on its own; the user may jog in between
        self.emit("G90")
        self.emit("G21")
        self._write_move(0, {"Z": 0.0}, machine=True)

    def _close_section(self):
        if self.section.lines[-1] != "M5":
            self.emit("M5")
        self.spindle_on = False
        self._write_move(0, {"Z": 0.0}, machine=True)

    def _to_machine(self, axis, value):
        mapped = self.offset[axis] + value
        if axis == "Z" and mapped > 0.0:
            self.warn("Z above the machine top (Z0) clamped to 0 - travel/tool-change height only")
            mapped = 0.0
        return mapped

    def _write_move(self, kind, target, machine=False):
        """Write one G0/G1 move. target: axis -> CAM coordinate (or machine coordinate if machine=True)."""
        out = {}
        for axis in AXES:
            if axis in target:
                out[axis] = target[axis] if machine else self._to_machine(axis, target[axis])
        if not out or all(self.machine[a] is not None and abs(self.machine[a] - v) < 1e-9 for a, v in out.items()):
            return  # nothing to move
        z_changes = "Z" in out and (self.machine["Z"] is None or abs(out["Z"] - self.machine["Z"]) > 1e-9)
        if kind == 0:
            feed = self.opt.rapid_z_feed if set(out) == {"Z"} else self.opt.rapid_xy_feed
        else:
            if self.feed is None:
                self.warn(f"no feed set before the first G1; using F{fmt(self.opt.default_feed)}")
                self.feed = self.opt.default_feed
            feed = self.feed
        if z_changes:
            feed = min(feed, self.opt.max_z_feed)
        words = " ".join(f"{axis}{fmt(out[axis])}" for axis in AXES if axis in out)
        self.emit(f"G{kind} {words} F{fmt(feed)}")
        for axis, value in out.items():
            self.machine[axis] = value
            for bounds in (self.bounds, self.section.bounds):
                lo, hi = bounds.get(axis, (value, value))
                bounds[axis] = (min(lo, value), max(hi, value))
        if not machine and ("X" in out or "Y" in out):  # Z-only travel alone doesn't make a section worth running
            self.section.moves += 1

    def _move(self, kind, target):
        z = target.get("Z", self.cam["Z"])
        if kind == 1 and z is not None and z < 0 and not self.spindle_on:  # cutting below the copper surface
            self.warn("cut below the surface with the spindle off; spindle started automatically")
            self.speed = self.speed or min(1000, self.opt.max_spindle)
            self.spindle_on = True
            self.emit(f"M3 S{self.speed}")
        self._write_move(kind, target)
        self.cam.update(target)

    # ---------- arcs ----------
    def _arc(self, clockwise, target, words, number):
        sx, sy = self.cam["X"], self.cam["Y"]
        if sx is None or sy is None:
            raise ConvertError(f"line {number}: arc before the X/Y position is known")
        ex, ey = target.get("X", sx), target.get("Y", sy)
        if "R" in words:
            cx, cy = self._center_from_radius(sx, sy, ex, ey, words["R"] * self.scale, clockwise, number)
        else:
            i, j = words.get("I", 0.0) * self.scale, words.get("J", 0.0) * self.scale
            cx, cy = (i, j) if self.arc_absolute else (sx + i, sy + j)
        radius = math.hypot(sx - cx, sy - cy)
        a0 = math.atan2(sy - cy, sx - cx)
        a1 = math.atan2(ey - cy, ex - cx)
        sweep = (a0 - a1) if clockwise else (a1 - a0)
        sweep %= 2 * math.pi
        if sweep < 1e-9:
            sweep = 2 * math.pi  # same start and end point: full circle
        if radius > self.opt.arc_tolerance:
            step = 2 * math.acos(max(-1.0, 1 - self.opt.arc_tolerance / radius))
            segments = max(1, math.ceil(sweep / step))
        else:
            segments = 1
        z0 = self.cam["Z"]
        z1 = target.get("Z", z0)
        direction = -1 if clockwise else 1
        for k in range(1, segments + 1):
            if k == segments:
                point = {"X": ex, "Y": ey}
            else:
                angle = a0 + direction * sweep * k / segments
                point = {"X": cx + radius * math.cos(angle), "Y": cy + radius * math.sin(angle)}
            if "Z" in target and z0 is not None:
                point["Z"] = z0 + (z1 - z0) * k / segments
            self._move(1, point)
        if "Z" in target:
            self.cam["Z"] = z1

    @staticmethod
    def _center_from_radius(sx, sy, ex, ey, r, clockwise, number):
        chord = math.hypot(ex - sx, ey - sy)
        if chord < 1e-9 or abs(r) < chord / 2 - 1e-9:
            raise ConvertError(f"line {number}: arc radius R{fmt(r)} can't reach its end point")
        mx, my = (sx + ex) / 2, (sy + ey) / 2
        h = math.sqrt(max(0.0, r * r - (chord / 2) ** 2))
        ux, uy = -(ey - sy) / chord, (ex - sx) / chord  # unit normal, left of the chord
        # Clockwise with a positive R puts the centre right of the chord; a negative R means the long way round.
        side = -1 if clockwise else 1
        if r < 0:
            side = -side
        return mx + side * h * ux, my + side * h * uy

    # ---------- canned drill cycles ----------
    def _drill(self, target, number):
        z_bottom = self.cycle.get("Z")
        r_plane = self.cycle.get("R")
        if z_bottom is None or r_plane is None:
            raise ConvertError(f"line {number}: drill cycle without Z or R")
        start_z = self.cam["Z"]
        if start_z is not None and start_z < r_plane:
            self._move(0, {"Z": r_plane})
        xy = {axis: target[axis] for axis in "XY" if axis in target}
        if xy:
            self._move(0, xy)
        self._move(0, {"Z": r_plane})
        if self.motion == 83 and self.cycle.get("Q"):
            depth = r_plane
            while depth > z_bottom + 1e-9:
                depth = max(depth - self.cycle["Q"], z_bottom)
                self._move(1, {"Z": depth})
                self._move(0, {"Z": r_plane})
                if depth > z_bottom + 1e-9:
                    self._move(0, {"Z": min(r_plane, depth + 0.1)})
        else:
            self._move(1, {"Z": z_bottom})
            if self.motion == 82 and self.cycle.get("P"):
                self.warn("drill dwell (G82 P) skipped - dwell isn't verified on this machine")
            if self.motion == 83:
                self.warn("G83 without Q drilled in one go")
        retract = r_plane if self.retract_to_r or start_z is None else max(start_z, r_plane)
        self._move(0, {"Z": retract})

    # ---------- main loop ----------
    def line(self, raw, number):
        comments = re.findall(r"\((.*?)\)", raw)
        text = re.sub(r"\(.*?\)", "", raw).split(";", 1)[0].split("*", 1)[0].strip()
        for comment in comments:  # remember the tool size for the section picker; the most specific comment wins
            rank = _note_rank(comment)
            if rank > _note_rank(self.section.note):
                self.section.note = comment.strip()
        if not text or text == "%":
            return
        words_list = _words(text)
        gcodes = [round(v, 1) for letter, v in words_list if letter == "G"]
        mcodes = [int(v) for letter, v in words_list if letter == "M"]
        words = {letter: v for letter, v in words_list if letter not in "GMNO"}

        for g in gcodes:
            if g == 20:
                self.scale = 25.4
            elif g == 21:
                self.scale = 1.0
            elif g == 90:
                pass  # absolute is the only mode we write
            elif g == 91:
                raise ConvertError(f"line {number}: relative moves (G91) aren't supported by the Cirqoid - "
                                   "export absolute coordinates from the CAM tool")
            elif g == 90.1:
                self.arc_absolute = True
            elif g == 91.1:
                self.arc_absolute = False
            elif g == 98:
                self.retract_to_r = False
            elif g == 99:
                self.retract_to_r = True
            elif g in (0, 1, 2, 3, 81, 82, 83):
                self.motion = int(g)
            elif g == 80:
                self.motion = None
            elif g == 4:
                self.warn("dwell (G4) skipped - not verified on this machine")
            elif g == 28:
                self.warn("homing (G28) inside the file skipped - home with the app's Home button")
            elif g in (17, 40, 49, 54, 61, 64, 94):
                pass  # modal settings that don't change anything here
            else:
                raise ConvertError(f"line {number}: G{fmt(g).replace('.0', '')} isn't supported")

        if "T" in words:
            self._new_section(f"T{int(words['T'])}")
        if "S" in words:
            speed = int(round(words["S"]))
            if speed > self.opt.max_spindle:
                self.warn(f"spindle speed S{speed} limited to S{self.opt.max_spindle}")
                speed = self.opt.max_spindle
            self.speed = speed
            if self.spindle_on and not ({3, 4} & set(mcodes)):
                self.emit(f"M3 S{speed}")
        if "F" in words:
            self.feed = words["F"] * self.scale
        if 3 in mcodes or 4 in mcodes:
            if 4 in mcodes:
                self.warn("M4 (counter-clockwise spindle) changed to M3")
            if not self.speed:
                self.warn("spindle started without a speed; using S1000")
                self.speed = min(1000, self.opt.max_spindle)
            self.spindle_on = True
            self.emit(f"M3 S{self.speed}")

        target = {axis: words[axis] * self.scale for axis in AXES if axis in words}
        if self.motion in (81, 82, 83):
            for key in ("Z", "R", "Q"):
                if key in words:
                    self.cycle[key] = words[key] * self.scale
            if "P" in words:
                self.cycle["P"] = words["P"]
            if "X" in target or "Y" in target:
                self._drill(target, number)
        elif target:
            if self.motion is None:
                raise ConvertError(f"line {number}: move without G0/G1")
            if self.motion in (2, 3):
                self._arc(self.motion == 2, target, words, number)
            else:
                self._move(self.motion, target)

        if 5 in mcodes:
            self.spindle_on = False
            self.emit("M5")
        if 6 in mcodes and self.section.moves:
            self._new_section(self.section.tool)
        if (0 in mcodes or 1 in mcodes) and self.section.moves:
            self._new_section(f"{self.section.tool} (after pause)")
        for m in mcodes:
            if m in (7, 8, 9):
                pass  # coolant: the Cirqoid has none
            elif m not in (0, 1, 2, 3, 4, 5, 6, 30):
                self.warn(f"M{m} skipped")

    def finish(self):
        self._close_section()
        sections = [s for s in self.sections if s.moves]
        warnings = [f"{message} (x{count})" if count > 1 else message for message, count in self.warnings.items()]
        return ConvertResult(sections, warnings, dict(self.bounds))


def convert(text, origin_x, origin_y, surface_z, options=None):
    """Convert a CAM G-code program. origin/surface are machine coordinates of the board corner and copper top."""
    converter = _Converter(origin_x, origin_y, surface_z, options or Options())
    for number, raw in enumerate(text.splitlines(), 1):
        converter.line(raw, number)
    result = converter.finish()
    if not result.sections:
        raise ConvertError("no moves found in the file")
    return result
