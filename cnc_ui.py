"""Small G-code control panel for a USB-connected CNC (test program).

Run with:  uv run cnc_ui.py
Pick the port "SIMULATOR" to try the UI without a machine attached.
"""

import json
import queue
import re
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

from cam_import import ConvertError, Options as CamOptions, convert as convert_cam, fmt

SIM_PORT = "SIMULATOR"
BAUDS = ["9600", "19200", "38400", "57600", "115200", "250000"]
ACK_TIMEOUT = 10  # seconds to wait for a reply before warning and sending the next line
REPLIES = ("ok", "error", "ack", "nack")  # line starts that count as the controller's answer

# Commands that differ between controller firmwares.
# bytes = real-time command written straight to the port; str = normal G-code line.
FIRMWARE = {
    "GRBL": {"home": "$H", "unlock": "$X", "position": b"?", "estop": b"\x18",
             "hold": b"!", "resume": b"~"},
    "Marlin": {"home": "G28", "unlock": "M999", "position": "M114", "estop": b"M112\n",
               "hold": None, "resume": None},
    "Generic": {"home": "G28", "unlock": None, "position": "M114", "estop": b"M112\n",
                "hold": None, "resume": None},
    # Cirqoid PCB mill: replies "ack"/"nack", absolute moves only (no G91), no position query
    # and no software e-stop, so E-STOP can only turn the spindle off (M5).
    # Cirqoid only homes properly with this exact sequence.
    "Cirqoid": {"home": ("G90", "G21", "G28 Y0.835"), "unlock": None, "position": None, "estop": b"M5\n",
                "hold": None, "resume": None},
}
DEFAULT_BAUD = {"GRBL": "115200", "Marlin": "250000", "Generic": "115200", "Cirqoid": "38400"}
# Firmwares that stay silent when they accept a line: wait this long for a possible "nack", then send the next.
SILENT_ACCEPT = {"Cirqoid": 0.5}
ABSOLUTE_ONLY = {"Cirqoid"}  # firmwares without G91: the app tracks the position and jogs with absolute moves
# Software travel limits in mm, measured from home (machine coordinates).
# Cirqoid Z is 0 at the top (home) and negative going down; -26.9 is the lowest point.
SOFT_LIMITS = {"Cirqoid": {"X": (0.0, 103.0), "Y": (0.0, 224.835), "Z": (-26.9, 0.0)}}
# Lines the Cirqoid answers with "ok"/"error" (moves and homing): wait for that answer, up to MOTION_TIMEOUT.
REPLY_EXPECTED = {"Cirqoid": re.compile(r"\bG0?[0-3]\b|\bG28\b|^[XYZ]", re.IGNORECASE)}
MOTION_TIMEOUT = 120  # seconds; homing can take a while
# Highest jog feed per axis (mm/min); Cirqoid's own programs move Z at F300.
JOG_MAX_FEED = {"Cirqoid": {"Z": 300.0}}
SETTINGS_FILE = Path.home() / ".soft-cnc.json"  # remembers small UI preferences such as the theme
# "app" is the window background; "bg" is the cards (sections) drawn on it.
THEMES = {
    "light": {"app": "#e4e7ec", "bg": "#ffffff", "fg": "#1f2328", "muted": "#6b7280", "field": "#f6f7f9",
              "button": "#eceef2", "hover": "#dde1e7", "border": "#d0d5dd", "select": "#d6e6fb",
              "accent": "#1f6feb", "accent_hover": "#3b82f6", "accent_fg": "#ffffff", "good": "#1a7f37",
              "disabled": "#a3a8b0", "console_bg": "#fafbfc", "console_fg": "#2d333b",
              "tx": "#1f5fbf", "err": "#cf222e", "info": "#1a7f37", "ok": "#6a9a6a"},
    "dark": {"app": "#141414", "bg": "#202124", "fg": "#e6e6e6", "muted": "#8b9099", "field": "#2a2b2e",
             "button": "#303134", "hover": "#3c3d41", "border": "#36373b", "select": "#264f78",
             "accent": "#2f81f7", "accent_hover": "#4b93f8", "accent_fg": "#ffffff", "good": "#57ab5a",
             "disabled": "#5c5f66", "console_bg": "#151618", "console_fg": "#d4d4d4",
             "tx": "#7fc7ff", "err": "#ff6b6b", "info": "#9ccc65", "ok": "#5f8f5f"},
}
SPINDLE_MAX = 1400  # above this the spindle doesn't get any faster
SPINDLE_STEP = 50

AXIS_RE = re.compile(r"([XYZ])\s*(-?\d*\.?\d+)")
MARLIN_POS_RE = re.compile(r"X:(-?[\d.]+)\s*Y:(-?[\d.]+)\s*Z:(-?[\d.]+)")

# Auto-detect: baud rates to try (most common first) and how to recognise each firmware.
DETECT_BAUDS = [115200, 250000, 57600, 38400, 19200, 9600, 230400, 74880, 500000, 1000000]
DETECT_QUERIES = [b"\r\n", b"$I\r\n", b"?", b"\r\n", b"M115\r\n", b"M114\r\n"]  # info queries only, no motion
FINE_QUERIES = [b"\r\n", b"M115\r\n"]
FINE_OFFSETS = (-4, -2, 2, 4)  # percent, for controllers whose clock is slightly off
SILENT_SAMPLE = "(no reply at this rate, complaints at the others)"
SIGNATURES = [
    ("GRBL", re.compile(r"Grbl|\[VER:|<(Idle|Alarm|Run|Hold|Jog|Home)")),
    ("Marlin", re.compile(r"FIRMWARE_NAME|Marlin|echo:")),
    ("Cirqoid", re.compile(r"(^|\n)n?ack\b")),
    ("Generic", re.compile(r"(^|\n)ok\b|X:-?\d")),
]


def decimal_words(line):
    """Cirqoid rejects whole numbers on coordinates/feed: turn "X5 F1200" into "X5.0 F1200.0".

    G/M numbers and S (spindle speed, read as an integer) are left alone.
    """
    return re.sub(r"([XYZAIJKRF])(\s*-?\d+)(?![\d.])", r"\1\2.0", line, flags=re.IGNORECASE)


class PositionTracker:
    """Follows the position from the absolute moves we send and enforces soft limits.

    Positions are machine coordinates (from home). G92 sets a work offset: work = machine - offset.
    """

    MOVE_RE = re.compile(r"\bG0?[0-3]\b|^[XYZ]")

    def __init__(self, limits=None):
        self.limits = limits or {}
        self.machine = None  # unknown until homed
        self.offset = {axis: 0.0 for axis in "XYZ"}

    def copy(self):
        twin = PositionTracker(self.limits)
        twin.machine = dict(self.machine) if self.machine else None
        twin.offset = dict(self.offset)
        return twin

    def work(self):
        return {axis: self.machine[axis] - self.offset[axis] for axis in "XYZ"} if self.machine else None

    def work_range(self, axis):
        lo, hi = self.limits[axis]
        return lo - self.offset[axis], hi - self.offset[axis]

    def apply(self, line):
        """Update from one line. Returns an error message, and changes nothing, if the line isn't allowed."""
        upper = line.upper()
        words = {axis: float(v) for axis, v in AXIS_RE.findall(upper)}
        if re.search(r"\bG28\b", upper):
            self.machine = {axis: words.get(axis, 0.0) for axis in "XYZ"}
            self.offset = {axis: 0.0 for axis in "XYZ"}
        elif re.search(r"\bG92\b", upper):
            if self.machine is None:
                return "home the machine first (G92 needs a known position)"
            for axis, v in words.items():
                self.offset[axis] = self.machine[axis] - v
        elif words and self.MOVE_RE.search(upper):
            if self.machine is None:
                return "position unknown - home the machine first"
            target = {axis: v + self.offset[axis] for axis, v in words.items()}
            for axis, pos in target.items():
                if axis in self.limits:
                    lo, hi = self.limits[axis]
                    if not lo - 1e-6 <= pos <= hi + 1e-6:
                        return f"{axis} would go to {pos:g} mm from home; soft limit is {lo:g} to {hi:g}"
            self.machine.update(target)
        return None


def looks_like_cam(lines):
    """True for G-code from a CAM tool (Z0 = copper surface): it goes above Z0 and never homes the Cirqoid."""
    homes = any(re.search(r"\bG28\b", line, re.IGNORECASE) for line in lines)
    above = any(float(z) > 0 for line in lines for z in re.findall(r"Z\s*([-+]?\d*\.?\d+)", line, re.IGNORECASE))
    return above and not homes


def clean_gcode(text):
    """Strip comments and blank lines from a G-code program."""
    lines = []
    for raw in text.splitlines():
        line = re.sub(r"\(.*?\)", "", raw.split(";", 1)[0]).strip()
        if line and line != "%":
            lines.append(line)
    return lines


class SimulatedPort:
    """Pretends to be a GRBL-style controller so the UI can be tried without hardware."""

    BANNER = "Grbl 1.1h ['$' for help] (simulator)"

    def __init__(self):
        self.out = queue.Queue()
        self.pos = [0.0, 0.0, 0.0]
        self.relative = False
        self.state = "Idle"
        self.partial = ""
        self._emit(self.BANNER)

    def _emit(self, text):
        self.out.put((text + "\r\n").encode())

    def write(self, data):
        for ch in data.decode(errors="replace"):
            if ch == "?":
                x, y, z = self.pos
                self._emit(f"<{self.state}|MPos:{x:.3f},{y:.3f},{z:.3f}|FS:0,0>")
            elif ch == "!":
                self.state = "Hold"
            elif ch == "~":
                self.state = "Idle"
            elif ch == "\x18":
                self.state, self.partial = "Idle", ""
                self._emit(self.BANNER)
            elif ch == "\n":
                self._line(self.partial.strip().upper())
                self.partial = ""
            else:
                self.partial += ch

    def _line(self, line):
        time.sleep(0.02)  # pretend the controller is busy for a moment
        jog = line.startswith("$J=")  # GRBL jogs don't change the modal G90/G91 state
        line = line[3:] if jog else line
        relative = ("G91" in line) or (self.relative and "G90" not in line)
        if not jog:
            self.relative = relative
        words = {axis: float(v) for axis, v in AXIS_RE.findall(line)}
        if line in ("$H", "G28"):
            self.pos = [0.0, 0.0, 0.0]
        elif "G92" in line:
            for i, axis in enumerate("XYZ"):
                if axis in words:
                    self.pos[i] = words[axis]
        elif words:
            for i, axis in enumerate("XYZ"):
                if axis in words:
                    self.pos[i] = self.pos[i] + words[axis] if relative else words[axis]
        self._emit("ok")

    def read(self, _size):
        try:
            return self.out.get(timeout=0.1)
        except queue.Empty:
            return b""

    def close(self):
        pass


def open_port(name, baud):
    if name == SIM_PORT:
        return SimulatedPort()
    return serial.Serial(name, baud, timeout=0.1)


def collect(port, seconds):
    """Read everything the port sends for the given time."""
    data, end = b"", time.monotonic() + seconds
    while time.monotonic() < end:
        data += port.read(256)
    return data


def readable(data):
    printable = sum(32 <= b < 127 or b in (9, 10, 13) for b in data)
    return len(data) >= 3 and printable / len(data) >= 0.9


class Detector:
    """Tries baud rates on one port and recognises the firmware from its replies (runs in a thread)."""

    def __init__(self, port_name, events):
        self.port_name = port_name
        self.events = events
        self.cancel = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _log(self, text, tag="info"):
        self.events.put(("log", text, tag))

    def _probe(self, baud, settle, queries):
        port = open_port(self.port_name, baud)
        try:
            data = collect(port, settle)  # startup banner (many boards reset when the port opens)
            for q in queries:
                if self.cancel.is_set():
                    break
                port.write(q)
                data += collect(port, 0.5)
        finally:
            port.close()
        return data

    def _try(self, bauds, settle, queries):
        """Returns (baud, firmware, sample) for the first recognised firmware, else the first readable baud."""
        fallback = None
        for baud in bauds:
            if self.cancel.is_set():
                self._log("Auto-detect cancelled")
                return fallback
            data = self._probe(baud, settle, queries)
            text = data.decode("ascii", errors="replace")
            sample = " | ".join(line.strip() for line in text.splitlines() if line.strip())[:80]
            if not data:
                self._log(f"  {baud:>7}: no reply", "rx")
                self.silent.append(baud)
            elif not readable(data):
                self._log(f"  {baud:>7}: garbled ({len(data)} bytes) - wrong rate", "rx")
                self.garbled += 1
            else:
                firmware = next((fw for fw, sig in SIGNATURES if sig.search(text)), None)
                if firmware:
                    self._log(f"  {baud:>7}: {firmware} detected  \"{sample}\"", "info")
                    return baud, firmware, sample
                self._log(f"  {baud:>7}: readable, firmware not recognised  \"{sample}\"", "err")
                fallback = fallback or (baud, None, sample)
        return fallback

    def _run(self):
        found = None
        self.silent, self.garbled = [], 0
        try:
            self._log("Trying standard baud rates...")
            found = self._try(DETECT_BAUDS, 2.0, DETECT_QUERIES)
            if not found and self.silent and self.garbled:
                # Alive (it answered the garbage we sent at wrong rates) but quiet at this rate: it accepted the
                # queries without replying, which is how a Cirqoid behaves.
                found = (self.silent[0], "Cirqoid", SILENT_SAMPLE)
            elif not found and not self.cancel.is_set():
                self._log("Nothing readable; trying rates a few % off the standard ones...")
                fine = [round(b * (1 + p / 100)) for b in DETECT_BAUDS if b <= 250000 for p in FINE_OFFSETS]
                found = self._try(fine, 0.5, FINE_QUERIES)
        except serial.SerialException as exc:
            self._log(f"Auto-detect stopped: {exc}", "err")
        self.events.put(("detect_done", found))


class Link:
    """Serial connection. Lines are sent one at a time, each waiting for "ok"/"error".

    With silent_wait set (controllers that don't answer accepted lines), each line waits only that long
    for a reply and silence counts as accepted.
    """

    def __init__(self, port, events, silent_wait=None, reply_expected=None):
        self.port = port
        self.events = events  # queue read by the UI thread
        self.silent_wait = silent_wait
        self.reply_expected = reply_expected  # regex: lines that get a real answer even in silent mode
        self.last_reply = None
        self.lines = queue.Queue()
        self.ack = threading.Event()
        self.paused = threading.Event()
        self.write_lock = threading.Lock()
        self.generation = 0  # bumped by clear() so queued lines become stale
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._writer, daemon=True).start()

    def send(self, line, job=False, tag=None):
        self.lines.put((self.generation, line, job, tag))

    def realtime(self, data):
        with self.write_lock:
            self.port.write(data)

    def clear(self):
        """Drop everything still waiting to be sent."""
        self.generation += 1
        self.paused.clear()
        self.ack.set()

    def close(self):
        self.alive = False
        self.clear()
        self.port.close()

    def _writer(self):
        while self.alive:
            try:
                gen, line, job, tag = self.lines.get(timeout=0.2)
            except queue.Empty:
                continue
            while self.paused.is_set() and self.alive and gen == self.generation:
                time.sleep(0.05)
            if gen != self.generation or not self.alive:
                continue
            self.ack.clear()
            self.last_reply = None
            try:
                with self.write_lock:
                    self.port.write((line + "\n").encode())
            except Exception as exc:  # port unplugged, etc.
                self.events.put(("error", f"write failed: {exc}"))
                return
            self.events.put(("tx", line, tag))
            expects = not self.silent_wait or bool(self.reply_expected and self.reply_expected.search(line))
            wait = (MOTION_TIMEOUT if self.silent_wait else ACK_TIMEOUT) if expects else self.silent_wait
            deadline = time.monotonic() + wait
            while not self.ack.wait(0.05):
                if not self.alive or gen != self.generation:
                    break
                if time.monotonic() > deadline:
                    if expects:
                        self.events.put(("error", f"no reply after {wait}s for: {line} "
                                                  "(check baud rate / firmware)"))
                    break
            if gen == self.generation:
                reply = self.last_reply
                if job and reply and reply.lower().startswith(("error", "nack")):
                    self.generation += 1  # drop the rest of the job here, before another line can go out
                self.events.put(("line_done", tag, reply, job))

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.port.read(256)
            except Exception as exc:
                if self.alive:
                    self.events.put(("error", f"read failed: {exc}"))
                return
            if not chunk:
                if buf:  # data with no newline (often a baud mismatch): show it rather than hide it
                    self._emit(buf)
                    buf = b""
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._emit(raw)

    def _emit(self, raw):
        # Non-printable bytes are shown as \xNN so garbage from a wrong baud rate is visible.
        text = raw.decode("ascii", errors="backslashreplace").strip()
        if not text:
            return
        # GRBL answers "ok" / "error:N"; Marlin answers "ok" (its "Error:" is followed by ok).
        if text.startswith(REPLIES):
            self.last_reply = text
            self.ack.set()
        self.events.put(("rx", text))


class App:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.link = None
        self.detector = None
        self.job_total = 0
        self.job_done = 0
        self.job_lines = []
        self.history = []
        self.history_pos = 0
        self.tracker = None  # PositionTracker for absolute-only firmwares
        self.pending = {}  # tag -> (tracker snapshot before the line, line) for lines not yet answered
        self.sent = set()  # tags already written to the port
        self.next_tag = 0
        self.spindle_on = False
        self.last_msg_tag = None
        self.display = None

        root.title("Soft-CNC — G-code control (test)")
        root.geometry("1040x720")
        root.minsize(900, 660)
        self.theme = self._load_settings().get("theme", "light")
        self.palette = THEMES.get(self.theme, THEMES["light"])
        self._build()
        self.apply_theme(self.theme if self.theme in THEMES else "light")
        self._set_connected(False)
        self.refresh_ports()
        self._bind_keys()
        root.after(50, self._drain_events)
        root.after(250, self._poll_status)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- layout ----------
    @staticmethod
    def _card(parent, title=None, **pack):
        """A section panel (lighter than the window background) with a small bold header."""
        card = ttk.Frame(parent, style="Card.TFrame", padding=(12, 8, 12, 10))
        card.pack(fill="x", **({"pady": (0, 8)} | pack))
        if title:
            ttk.Label(card, text=title.upper(), style="CardTitle.TLabel").pack(anchor="w", pady=(0, 6))
        return card

    def _build(self):
        outer = ttk.Frame(self.root, style="App.TFrame", padding=10)
        outer.pack(fill="both", expand=True)

        # Top bar: connection
        top = ttk.Frame(outer, style="Card.TFrame", padding=(12, 8))
        top.pack(fill="x", pady=(0, 8))
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_box.pack(side="left", padx=(6, 2))
        ttk.Button(top, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        ttk.Label(top, text="Baud").pack(side="left", padx=(14, 0))
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD["Cirqoid"])
        baud_box = ttk.Combobox(top, textvariable=self.baud_var, values=BAUDS, width=8)
        baud_box.pack(side="left", padx=6)
        ttk.Label(top, text="Firmware").pack(side="left", padx=(8, 0))
        self.fw_var = tk.StringVar(value="Cirqoid")
        fw_box = ttk.Combobox(top, textvariable=self.fw_var, values=list(FIRMWARE), width=8, state="readonly")
        fw_box.pack(side="left", padx=6)
        fw_box.bind("<<ComboboxSelected>>", lambda e: self._firmware_changed())
        self.comboboxes = [self.port_box, baud_box, fw_box]
        self.connect_btn = ttk.Button(top, text="Connect", style="Accent.TButton", width=11,
                                      command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=(10, 4))
        self.auto_btn = ttk.Button(top, text="Auto-detect", command=self.auto_detect)
        self.auto_btn.pack(side="left")
        self.theme_btn = ttk.Button(top, width=8, command=self.toggle_theme)
        self.theme_btn.pack(side="right")
        self.status_var = tk.StringVar(value="○ Disconnected")
        self.status_label = ttk.Label(top, textvariable=self.status_var, font=("Segoe UI", 10, "bold"))
        self.status_label.pack(side="right", padx=12)

        body = ttk.Frame(outer, style="App.TFrame")
        body.pack(fill="both", expand=True)

        # Left: position readout, jog, machine commands, spindle, e-stop
        left = ttk.Frame(body, style="App.TFrame")
        left.pack(side="left", fill="y", padx=(0, 8))
        self.estop_btn = tk.Button(left, text="E-STOP  (Esc)", bg="#c62828", fg="white", relief="flat",
                                   activebackground="#8e0000", activeforeground="white", cursor="hand2",
                                   font=("Segoe UI", 15, "bold"), command=self.estop)
        self.estop_btn.pack(side="bottom", fill="x", ipady=6)  # packed first so it is never clipped

        dro = ttk.Frame(self._card(left, "Position · mm"))
        dro.pack(fill="x")
        dro.columnconfigure(1, weight=1)
        self.pos_vars = {axis: tk.StringVar(value="—") for axis in "XYZ"}
        for row, axis in enumerate("XYZ"):
            ttk.Label(dro, text=axis, style="Axis.TLabel").grid(row=row, column=0, sticky="w")
            ttk.Label(dro, textvariable=self.pos_vars[axis], style="DRO.TLabel", anchor="e").grid(
                row=row, column=1, sticky="e")
        self.pos_note_var = tk.StringVar(value="disconnected")
        ttk.Label(dro, textvariable=self.pos_note_var, style="Muted.TLabel").grid(
            row=3, column=0, columnspan=2, pady=(2, 0), sticky="w")

        jog = self._card(left, "Jog")
        grid = ttk.Frame(jog)
        grid.pack(anchor="center")
        b = lambda text, axis, sign, r, c: ttk.Button(
            grid, text=text, width=5, command=lambda: self.jog(axis, sign)).grid(row=r, column=c, padx=2, pady=2)
        b("Y+", "Y", 1, 0, 1)
        b("X−", "X", -1, 1, 0)
        b("X+", "X", 1, 1, 2)
        b("Y−", "Y", -1, 2, 1)
        b("Z↑", "Z", 1, 0, 4)
        b("Z↓", "Z", -1, 2, 4)
        self.jog_buttons = grid.winfo_children()
        grid.columnconfigure(3, minsize=14)  # gap between the XY pad and the Z column

        steps = ttk.Frame(jog)
        steps.pack(fill="x", pady=(8, 4))
        ttk.Label(steps, text="Step").pack(side="left", padx=(0, 6))
        self.step_var = tk.StringVar(value="1")
        for value in ("0.1", "1", "5", "10", "50"):
            ttk.Radiobutton(steps, text=value, value=value, variable=self.step_var, style="Toolbutton",
                            width=4).pack(side="left", padx=1)
        ttk.Label(steps, text="mm", style="Muted.TLabel").pack(side="left", padx=(4, 0))
        opts = ttk.Frame(jog)
        opts.pack(fill="x")
        ttk.Label(opts, text="Feed").pack(side="left", padx=(0, 6))
        self.feed_var = tk.StringVar(value="500")
        ttk.Entry(opts, textvariable=self.feed_var, width=7).pack(side="left")
        ttk.Label(opts, text="mm/min", style="Muted.TLabel").pack(side="left", padx=(4, 0))
        ttk.Label(jog, text="Keys  ← → ↑ ↓  PgUp/PgDn   ·   Esc stops", style="Muted.TLabel").pack(
            anchor="w", pady=(6, 0))

        cmds = ttk.Frame(self._card(left, "Machine"))
        cmds.pack(fill="x")
        cmds.columnconfigure((0, 1), weight=1)
        self.machine_btns = {}  # firmware command name (or "zero") -> button; hidden if the firmware lacks it
        for key, text, fn in [
            ("home", "⌂  Home", lambda: self.fw_command("home")),
            ("zero", "Zero XYZ (G92)", lambda: self.send_line("G92 X0 Y0 Z0")),
            ("unlock", "Unlock", lambda: self.fw_command("unlock")),
            ("position", "Position", lambda: self.fw_command("position")),
            ("hold", "Feed hold", lambda: self.fw_command("hold")),
            ("resume", "Resume", lambda: self.fw_command("resume")),
        ]:
            self.machine_btns[key] = ttk.Button(cmds, text=text, command=fn,
                                                style="Accent.TButton" if key == "home" else "TButton")
        self.machine_buttons = list(self.machine_btns.values())
        self._layout_machine_buttons()

        spindle = ttk.Frame(self._card(left, "Spindle"))
        spindle.pack(fill="x")
        spindle.columnconfigure((0, 1, 2), weight=1)
        ttk.Label(spindle, text="Speed S").grid(row=0, column=0, sticky="w")
        self.speed_var = tk.StringVar(value="1000")
        self.speed_box = ttk.Spinbox(spindle, textvariable=self.speed_var, from_=SPINDLE_STEP, to=SPINDLE_MAX,
                                     increment=SPINDLE_STEP, width=7)
        self.speed_box.grid(row=0, column=1, sticky="w", pady=(0, 6))
        self.spindle_state_var = tk.StringVar(value="off")
        self.spindle_label = ttk.Label(spindle, textvariable=self.spindle_state_var, font=("Segoe UI", 9, "bold"))
        self.spindle_label.grid(row=0, column=2, sticky="e")
        self.spindle_buttons = [self.speed_box]
        for i, (text, fn) in enumerate([("Start M3", self.spindle_start), ("Set speed", self.spindle_set),
                                        ("Stop M5", self.spindle_stop)]):
            btn = ttk.Button(spindle, text=text, command=fn)
            btn.grid(row=1, column=i, sticky="ew", padx=(0 if i == 0 else 4, 0))
            self.spindle_buttons.append(btn)

        # Right: tabs
        self.tabs = ttk.Notebook(body)
        self.tabs.pack(side="left", fill="both", expand=True)
        # Clicking a tab would give it focus, and arrow keys would then switch tabs as well as jog.
        self.tabs.bind("<<NotebookTabChanged>>", lambda e: self.root.focus_set())

        console = ttk.Frame(self.tabs, style="Card.TFrame", padding=10)
        self.tabs.add(console, text="  Console  ")
        text_box = ttk.Frame(console)
        text_box.pack(fill="both", expand=True)
        self.log_text = tk.Text(text_box, height=10, font=("Consolas", 10), state="disabled", wrap="word",
                                relief="flat", borderwidth=0, padx=8, pady=6)  # colours: apply_theme
        scroll = ttk.Scrollbar(text_box, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)
        entry_row = ttk.Frame(console)
        entry_row.pack(fill="x", pady=(8, 0))
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(entry_row, textvariable=self.cmd_var, font=("Consolas", 11))
        self.cmd_entry.pack(side="left", fill="x", expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self.send_entry())
        self.cmd_entry.bind("<Up>", lambda e: self.history_step(-1))
        self.cmd_entry.bind("<Down>", lambda e: self.history_step(1))
        self.send_btn = ttk.Button(entry_row, text="Send", command=self.send_entry)
        self.send_btn.pack(side="left", padx=(6, 0))
        ttk.Button(entry_row, text="Clear", command=self.clear_log).pack(side="left", padx=(6, 0))

        # PCB job tab: 1 board position, 2 program, 3 run
        job = ttk.Frame(self.tabs, style="Card.TFrame", padding=(16, 12))
        self.tabs.add(job, text="  PCB job  ")
        job.columnconfigure(0, weight=1)

        ttk.Label(job, text="1 · BOARD POSITION", style="CardTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(job, text="Home, jog the bit over the board's corner, then lower it until it just touches "
                            "the copper. Redo the surface Z after every bit change.",
                  style="Muted.TLabel", wraplength=640).grid(row=1, column=0, sticky="w", pady=(2, 8))
        board = ttk.Frame(job)
        board.grid(row=2, column=0, sticky="w")
        self.origin_x_var, self.origin_y_var, self.surface_var = tk.StringVar(), tk.StringVar(), tk.StringVar()
        self.air_var = tk.BooleanVar(value=False)  # tick for a dry run: the whole job is lifted by the offset
        self.air_offset_var = tk.StringVar(value="5")
        ttk.Label(board, text="Origin  X").grid(row=0, column=0, sticky="w")
        ttk.Entry(board, textvariable=self.origin_x_var, width=9).grid(row=0, column=1, padx=6)
        ttk.Label(board, text="Y").grid(row=0, column=2)
        ttk.Entry(board, textvariable=self.origin_y_var, width=9).grid(row=0, column=3, padx=6)
        self.origin_btn = ttk.Button(board, text="Set origin here", command=self.set_origin_here)
        self.origin_btn.grid(row=0, column=4, padx=(6, 0), sticky="ew")
        ttk.Label(board, text="Surface  Z").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(board, textvariable=self.surface_var, width=9).grid(row=1, column=1, padx=6, pady=(6, 0))
        self.surface_btn = ttk.Button(board, text="Set surface Z here", command=self.set_surface_here)
        self.surface_btn.grid(row=1, column=4, padx=(6, 0), pady=(6, 0), sticky="ew")
        air = ttk.Frame(board)
        air.grid(row=2, column=0, columnspan=5, sticky="w", pady=(8, 0))
        ttk.Checkbutton(air, text="Air test: lift the whole job by", variable=self.air_var).pack(side="left")
        ttk.Entry(air, textvariable=self.air_offset_var, width=4).pack(side="left", padx=6)
        ttk.Label(air, text="mm (dry run)", style="Muted.TLabel").pack(side="left")

        ttk.Separator(job).grid(row=3, column=0, sticky="ew", pady=14)
        ttk.Label(job, text="2 · PROGRAM", style="CardTitle.TLabel").grid(row=4, column=0, sticky="w")
        ttk.Label(job, text="Load… for ready-made Cirqoid G-code.  Import CAM… for FlatCAM / pcb2gcode output "
                            "(converted to the board position above).",
                  style="Muted.TLabel", wraplength=640).grid(row=5, column=0, sticky="w", pady=(2, 8))
        program = ttk.Frame(job)
        program.grid(row=6, column=0, sticky="ew")
        ttk.Button(program, text="Load…", command=self.load_file).pack(side="left")
        ttk.Button(program, text="Import CAM…", command=self.import_cam).pack(side="left", padx=6)
        self.file_var = tk.StringVar(value="no file loaded")
        ttk.Label(program, textvariable=self.file_var, font=("Segoe UI", 10, "bold")).pack(side="left", padx=8)

        ttk.Separator(job).grid(row=7, column=0, sticky="ew", pady=14)
        ttk.Label(job, text="3 · RUN", style="CardTitle.TLabel").grid(row=8, column=0, sticky="w", pady=(0, 8))
        run = ttk.Frame(job)
        run.grid(row=9, column=0, sticky="ew")
        self.start_btn = ttk.Button(run, text="▶  Start", style="Accent.TButton", width=10, command=self.start_job)
        self.start_btn.pack(side="left")
        self.pause_btn = ttk.Button(run, text="Pause", width=8, command=self.pause_job)
        self.pause_btn.pack(side="left", padx=6)
        self.stop_btn = ttk.Button(run, text="■  Stop", width=8, command=self.stop_job)
        self.stop_btn.pack(side="left")
        self.progress_var = tk.StringVar(value="")
        ttk.Label(run, textvariable=self.progress_var, width=12, anchor="e").pack(side="right")
        self.progress = ttk.Progressbar(run)
        self.progress.pack(side="left", fill="x", expand=True, padx=(12, 6))

        ttk.Label(job, text="LAST MESSAGE", style="CardTitle.TLabel").grid(row=10, column=0, sticky="w",
                                                                         pady=(18, 2))
        self.last_msg_var = tk.StringVar(value="—")
        self.last_msg_label = ttk.Label(job, textvariable=self.last_msg_var, wraplength=640)
        self.last_msg_label.grid(row=11, column=0, sticky="w")

    # ---------- helpers ----------
    def log(self, text, tag="rx"):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n", tag)
        if int(self.log_text.index("end-1c").split(".")[0]) > 3000:
            self.log_text.delete("1.0", "500.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
        if tag in ("info", "err"):  # also shown under "Last message" on the PCB job tab
            self.last_msg_var.set(text.strip())
            self.last_msg_tag = tag
            self.last_msg_label.configure(foreground=self.palette[tag])

    def _log_ok(self):
        """Show an "ok" at the end of the command line it answers, instead of on a line of its own."""
        last = "end-2c linestart"
        if "tx" in self.log_text.tag_names(last) and "ok" not in self.log_text.tag_names("end-3c"):
            self.log_text.configure(state="normal")
            self.log_text.insert("end-2c", "   ✓ ok", "ok")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        else:
            self.log("ok", "ok")

    # ---------- theme ----------
    @staticmethod
    def _load_settings():
        try:
            return json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _save_settings(self):
        try:
            SETTINGS_FILE.write_text(json.dumps({"theme": self.theme}), encoding="utf-8")
        except OSError:
            pass  # a preference that isn't saved is not worth an error

    def toggle_theme(self):
        self.apply_theme("dark" if self.theme == "light" else "light")
        self._save_settings()

    def apply_theme(self, name):
        """Recolour every widget. Uses the 'clam' style because Windows' native style can't be recoloured."""
        self.theme, p = name, THEMES[name]
        self.palette = p
        style = ttk.Style(self.root)
        style.theme_use("clam")
        self.root.configure(bg=p["app"])
        style.configure(".", background=p["bg"], foreground=p["fg"], fieldbackground=p["field"],
                        bordercolor=p["border"], lightcolor=p["bg"], darkcolor=p["bg"], troughcolor=p["field"],
                        selectbackground=p["select"], selectforeground=p["fg"], insertcolor=p["fg"],
                        focuscolor=p["accent"], arrowcolor=p["fg"])
        style.map(".", foreground=[("disabled", p["disabled"])])
        style.configure("TButton", background=p["button"], padding=(6, 1), lightcolor=p["button"],
                        darkcolor=p["button"])
        style.map("TButton", background=[("disabled", p["bg"]), ("pressed", p["hover"]), ("active", p["hover"])],
                  lightcolor=[("active", p["hover"])], darkcolor=[("active", p["hover"])])
        style.configure("Toolbutton", background=p["bg"], padding=(4, 2), anchor="center")
        style.map("Toolbutton", background=[("selected", p["select"]), ("active", p["hover"])])
        for widget in ("TEntry", "TCombobox", "TSpinbox"):
            style.configure(widget, fieldbackground=p["field"], foreground=p["fg"], background=p["button"],
                            lightcolor=p["field"], darkcolor=p["field"])
            style.map(widget, fieldbackground=[("disabled", p["bg"]), ("readonly", p["field"])],
                      foreground=[("disabled", p["disabled"])],
                      selectbackground=[("readonly", p["field"])], selectforeground=[("readonly", p["fg"])])
        for widget in ("TCheckbutton", "TRadiobutton"):
            style.configure(widget, indicatorbackground=p["field"], indicatorforeground=p["fg"])
            style.map(widget, background=[("active", p["bg"])],
                      indicatorbackground=[("selected", p["field"]), ("disabled", p["bg"])])
        style.configure("App.TFrame", background=p["app"])
        style.configure("Card.TFrame", background=p["bg"])
        style.configure("CardTitle.TLabel", foreground=p["muted"], font=("Segoe UI", 8, "bold"))
        style.configure("Muted.TLabel", foreground=p["muted"])
        style.configure("Axis.TLabel", foreground=p["muted"], font=("Segoe UI", 14, "bold"))
        style.configure("DRO.TLabel", foreground=p["fg"], font=("Consolas", 18, "bold"))
        style.configure("Accent.TButton", background=p["accent"], foreground=p["accent_fg"],
                        bordercolor=p["accent"], lightcolor=p["accent"], darkcolor=p["accent"],
                        font=("Segoe UI", 9, "bold"))
        style.map("Accent.TButton",
                  background=[("disabled", p["button"]), ("pressed", p["accent_hover"]), ("active", p["accent_hover"])],
                  foreground=[("disabled", p["disabled"])],
                  bordercolor=[("disabled", p["border"])],
                  lightcolor=[("disabled", p["button"]), ("active", p["accent_hover"])],
                  darkcolor=[("disabled", p["button"]), ("active", p["accent_hover"])])
        style.configure("TNotebook", background=p["app"], borderwidth=0, tabmargins=(0, 0, 0, 0))
        style.configure("TNotebook.Tab", background=p["app"], foreground=p["muted"], padding=(14, 6),
                        bordercolor=p["app"], lightcolor=p["app"], font=("Segoe UI", 9, "bold"))
        style.map("TNotebook.Tab", background=[("selected", p["bg"]), ("active", p["hover"])],
                  foreground=[("selected", p["fg"]), ("active", p["fg"])],
                  lightcolor=[("selected", p["bg"])], bordercolor=[("selected", p["bg"])])
        style.configure("TSeparator", background=p["border"])
        style.configure("Horizontal.TProgressbar", background=p["accent"], troughcolor=p["field"],
                        lightcolor=p["accent"], darkcolor=p["accent"])
        style.configure("TScrollbar", background=p["button"], troughcolor=p["bg"], lightcolor=p["button"],
                        darkcolor=p["button"])
        style.map("TScrollbar", background=[("active", p["hover"])])
        # Drop-down lists of the comboboxes are plain Tk listboxes.
        for option, value in (("background", p["field"]), ("foreground", p["fg"]),
                              ("selectBackground", p["select"]), ("selectForeground", p["fg"])):
            self.root.option_add(f"*TCombobox*Listbox.{option}", value)
        for box in self.comboboxes:
            try:
                popdown = self.root.tk.call("ttk::combobox::PopdownWindow", box)
                self.root.tk.call(f"{popdown}.f.l", "configure", "-background", p["field"], "-foreground",
                                  p["fg"], "-selectbackground", p["select"], "-selectforeground", p["fg"])
            except tk.TclError:
                pass
        self.log_text.configure(bg=p["console_bg"], fg=p["console_fg"], insertbackground=p["console_fg"],
                                selectbackground=p["select"])
        for tag in ("tx", "err", "info", "ok"):
            self.log_text.tag_configure(tag, foreground=p[tag])
        self.log_text.tag_configure("rx", foreground=p["console_fg"])
        self.theme_btn.configure(text="☀ Light" if name == "dark" else "☾ Dark")
        self.status_label.configure(foreground=p["good"] if self.link else p["muted"])
        self.spindle_label.configure(foreground=p["good"] if self.spindle_on else p["muted"])
        self.last_msg_label.configure(foreground=p[self.last_msg_tag] if self.last_msg_tag else p["muted"])

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _set_position(self, x=None, y=None, z=None, note=""):
        for axis, value in zip("XYZ", (x, y, z)):
            if value is None:
                text = "—"
            else:
                try:
                    text = f"{float(value):.3f}"
                except ValueError:
                    text = str(value)
            self.pos_vars[axis].set(text)
        self.pos_note_var.set(note)

    def _firmware_changed(self):
        self.baud_var.set(DEFAULT_BAUD[self.fw_var.get()])
        self._layout_machine_buttons()
        self._update_controls()

    def _layout_machine_buttons(self):
        """Show only the machine buttons the selected firmware has a command for."""
        fw = FIRMWARE[self.fw_var.get()]
        visible = [btn for key, btn in self.machine_btns.items() if key == "zero" or fw.get(key) is not None]
        for btn in self.machine_btns.values():
            btn.grid_remove()
        for i, btn in enumerate(visible):
            btn.grid(row=i // 2, column=i % 2, sticky="ew", padx=2, pady=2)

    def _bind_keys(self):
        keys = {"<Left>": ("X", -1), "<Right>": ("X", 1), "<Up>": ("Y", 1), "<Down>": ("Y", -1),
                "<Prior>": ("Z", 1), "<Next>": ("Z", -1)}
        for key, (axis, sign) in keys.items():
            self.root.bind(key, lambda e, a=axis, s=sign: self._key_jog(a, s))
        self.root.bind("<Escape>", lambda e: self.estop())

    def _typing(self):
        try:
            widget = self.root.focus_get()
        except (KeyError, tk.TclError):
            return False
        if isinstance(widget, tk.Text):  # the console is read-only: keys there should still jog
            return widget.cget("state") == "normal"
        return isinstance(widget, (tk.Entry, ttk.Entry, tk.Spinbox))

    def _key_jog(self, axis, sign):
        if not self._typing() and self.link and not self.job_running():
            self.jog(axis, sign)

    def job_running(self):
        return self.job_total > 0 and self.job_done < self.job_total

    def _set_connected(self, connected):
        self.connect_btn.configure(text="Disconnect" if connected else "Connect")
        if connected:
            self.status_var.set("● Connected")
            self.status_label.configure(foreground=self.palette["good"])
            if self.tracker:
                self._set_position(note="not homed — press Home")
            else:
                self._set_position(note="connected")
        else:
            self.status_var.set("○ Disconnected")
            self.status_label.configure(foreground=self.palette["muted"])
            self._set_position(note="disconnected")
        self._update_controls()

    def _update_controls(self):
        connected = self.link is not None
        running = self.job_running()
        manual = "normal" if connected and not running else "disabled"
        for w in (self.jog_buttons + self.machine_buttons + self.spindle_buttons
                  + [self.send_btn, self.cmd_entry, self.origin_btn, self.surface_btn]):
            w.configure(state=manual)
        if connected and self.fw_var.get() == "GRBL":  # real-time commands work mid-job on GRBL
            for key in ("hold", "resume"):
                self.machine_btns[key].configure(state="normal")
        self.estop_btn.configure(state="normal" if connected else "disabled")
        self.start_btn.configure(state="normal" if connected and not running and self.job_lines else "disabled")
        self.pause_btn.configure(state="normal" if running else "disabled")
        self.stop_btn.configure(state="normal" if running else "disabled")

    # ---------- connection ----------
    def refresh_ports(self):
        ports = [f"{p.device} — {p.description}" for p in list_ports.comports()]
        self.port_box.configure(values=ports + [SIM_PORT])
        if not self.port_var.get():
            self.port_var.set(ports[0] if ports else SIM_PORT)

    def toggle_connect(self):
        if self.link:
            self.disconnect()
            return
        choice = self.port_var.get().split(" — ")[0]
        try:
            port = open_port(choice, int(self.baud_var.get()))
        except (serial.SerialException, ValueError) as exc:
            messagebox.showerror("Connect failed", str(exc))
            return
        fw = self.fw_var.get()
        self.link = Link(port, self.events, SILENT_ACCEPT.get(fw), REPLY_EXPECTED.get(fw))
        self.pending.clear()
        self.sent.clear()
        self.tracker = PositionTracker(SOFT_LIMITS.get(fw)) if fw in ABSOLUTE_ONLY else None
        self.display = PositionTracker() if fw in ABSOLUTE_ONLY else None  # follows lines as they go out
        self._set_spindle(False)
        self.log(f"Connected to {choice} @ {self.baud_var.get()} ({fw})", "info")
        if fw in ABSOLUTE_ONLY:
            self.log("Cirqoid: it doesn't reply to accepted commands, so silence means OK and only a 'nack' is an "
                     "error. Press Home (G90, G21, G28 Y0.835) before moving, so the app knows where the machine is.",
                     "info")
        if fw in SOFT_LIMITS:
            self.log("Soft limits (mm from home): " + ", ".join(
                f"{axis} {lo:g}..{hi:g}" for axis, (lo, hi) in SOFT_LIMITS[fw].items()), "info")
        self._set_connected(True)

    def auto_detect(self):
        if self.detector:
            self.detector.cancel.set()
            return
        if self.link:
            self.disconnect()
        name = self.port_var.get().split(" — ")[0]
        self.log(f"Auto-detect on {name} (info queries only, no motion). Takes up to ~2 minutes.", "info")
        self.detector = Detector(name, self.events)
        self.auto_btn.configure(text="Stop scan")
        self.connect_btn.configure(state="disabled")

    def _detect_done(self, found):
        self.detector = None
        self.auto_btn.configure(text="Auto-detect")
        self.connect_btn.configure(state="normal")
        if not found:
            self.log("Auto-detect: no working setting found. Check the cable/power, "
                     "or the controller may not use a serial text protocol.", "err")
            return
        baud, firmware, sample = found
        self.baud_var.set(str(baud))
        if firmware:
            self.fw_var.set(firmware)
            self.log(f"Auto-detect: {firmware} at {baud} baud. Settings applied - click Connect.", "info")
            if sample == SILENT_SAMPLE:
                self.log(f"The controller accepts commands at {baud} baud without replying, as a Cirqoid does.",
                         "info")
            if "communication error" in sample:
                self.log("Warning: the controller answers 'nack: communication error' to every byte, so it is "
                         "not receiving the PC's data correctly. Unplug the USB cable, power-cycle the machine, "
                         "plug it back in and try again.", "err")
        else:
            self.fw_var.set("Generic")
            hint = (" It rejects the commands (nack), so it probably needs its own protocol, not plain G-code."
                    if "nack" in sample.lower() else "")
            self.log(f"Auto-detect: the controller talks at {baud} baud, but its firmware isn't one this app "
                     f"knows.{hint} Baud applied, firmware set to Generic.", "err")

    def disconnect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.job_total = self.job_done = 0
        self.tracker = self.display = None
        self.pending.clear()
        self.sent.clear()
        self._set_spindle(False)
        self.log("Disconnected", "info")
        self._set_connected(False)

    # ---------- commands ----------
    def send_line(self, line):
        """Send one line; returns False if it was refused (soft limit / unknown position)."""
        if not (self.link and line):
            return False
        line = self._dialect(line)
        snapshot = self.tracker.copy() if self.tracker else None
        if self.tracker:
            error = self.tracker.apply(line)
            if error:
                self.log(f"Not sent: {line}  ({error})", "err")
                return False
        self._enqueue(line, snapshot)
        self._show_position()
        return True

    def _enqueue(self, line, snapshot, job=False):
        tag, self.next_tag = self.next_tag, self.next_tag + 1
        self.pending[tag] = (snapshot, line)
        self.link.send(line, job, tag)

    def _line_done(self, tag, reply, job):
        """The machine answered (or the wait ended). On a refusal, put the tracked position back."""
        snapshot, line = self.pending.pop(tag, (None, ""))
        self.sent.discard(tag)
        refused = bool(reply) and reply.lower().startswith(("error", "nack"))
        if refused and job and self.job_running():
            self.link.clear()
            self._discard_unsent(restore=False)
            self.job_total = self.job_done = 0
            self.log(f"Job stopped: the machine refused '{line}'", "err")
            self._update_progress()
            self._update_controls()
        if refused and self.tracker and snapshot:
            self.tracker = snapshot.copy()
            for later, (_, later_line) in list(self.pending.items()):  # replay what is still queued
                self.pending[later] = (self.tracker.copy(), later_line)
                self.tracker.apply(later_line)
            if self.display:  # redo the display from before the refused line, plus what went out after it
                self.display = snapshot.copy()
                self.display.limits = {}
                for later, (_, later_line) in self.pending.items():
                    if later in self.sent:
                        self.display.apply(later_line)
            self._show_position()
            self.log(f"Position kept as before '{line}' (the machine refused it)", "err")
        elif job and self.job_total:
            self.job_done += 1
            self._update_progress()
            if self.job_done >= self.job_total:
                self.log("Job finished", "info")
                self._update_controls()

    def _discard_unsent(self, restore=True):
        """After the queue is cleared: forget lines that never went out, and their effect on the position."""
        unsent = [tag for tag in self.pending if tag not in self.sent]
        if restore and unsent and self.tracker and self.pending[unsent[0]][0]:
            self.tracker = self.pending[unsent[0]][0].copy()
            self._show_position()
        for tag in unsent:
            del self.pending[tag]

    def _dialect(self, line):
        return decimal_words(line) if self.fw_var.get() == "Cirqoid" else line

    def _on_sent(self, line):
        """A line has actually been written to the machine: update spindle state and the position readout."""
        self._note_spindle(line)
        if self.display:
            self.display.apply(line)
            self._show_position()

    def _show_position(self):
        source = self.display or self.tracker  # the readout shows what has been sent, not what is queued
        work = source.work() if source else None
        if work:
            self._set_position(work["X"], work["Y"], work["Z"], "homed · position as sent")

    # ---------- spindle ----------
    def _speed(self):
        try:
            speed = int(float(self.speed_var.get()))
        except ValueError:
            speed = -1
        if speed <= 0:
            self.log("Spindle speed must be a positive whole number", "err")
            return None
        if speed > SPINDLE_MAX:
            self.log(f"Spindle speed limited to the maximum, S{SPINDLE_MAX}", "info")
            speed = SPINDLE_MAX
            self.speed_var.set(str(speed))
        return speed

    def spindle_start(self):
        speed = self._speed()
        if speed:
            self.send_line(f"M3 S{speed}")

    def spindle_set(self):
        speed = self._speed()
        if not speed:
            return
        if self.spindle_on:
            self.send_line(f"M3 S{speed}")
        else:
            self.log(f"Spindle is off; S{speed} will be used when you press Start", "info")

    def spindle_stop(self):
        self.send_line("M5")

    def _note_spindle(self, line):
        upper = line.upper()
        if re.search(r"\bM0?3\b", upper):
            speed = re.search(r"S\s*(\d+)", upper)
            self._set_spindle(True, speed.group(1) if speed else None)
        elif re.search(r"\bM0?5\b", upper):
            self._set_spindle(False)

    def _set_spindle(self, on, speed=None):
        self.spindle_on = on
        self.spindle_state_var.set(f"● on S{speed}" if on and speed else "● on" if on else "off")
        self.spindle_label.configure(foreground=self.palette["good"] if on else self.palette["muted"])

    def send_entry(self):
        line = self.cmd_var.get().strip()
        if not line:
            return
        self.history.append(line)
        self.history_pos = len(self.history)
        self.cmd_var.set("")
        if line in ("?", "!", "~"):  # GRBL real-time characters: no newline, no "ok"
            self.link.realtime(line.encode())
            self.log(line, "tx")
        else:
            self.send_line(line)

    def history_step(self, delta):
        if not self.history:
            return
        self.history_pos = max(0, min(len(self.history), self.history_pos + delta))
        self.cmd_var.set(self.history[self.history_pos] if self.history_pos < len(self.history) else "")
        self.cmd_entry.icursor("end")

    def fw_command(self, name):
        cmd = FIRMWARE[self.fw_var.get()][name]
        if cmd is None:
            self.log(f"'{name}' is not available for {self.fw_var.get()}", "err")
        elif isinstance(cmd, bytes):
            self.link.realtime(cmd)
            if name != "position":
                self.log(f"[{name}]", "tx")
        elif isinstance(cmd, tuple):  # a sequence of lines
            for line in cmd:
                if not self.send_line(line):
                    break
        else:
            self.send_line(cmd)

    def jog(self, axis, sign):
        try:
            step = float(self.step_var.get()) * sign
            feed = float(self.feed_var.get())
        except ValueError:
            self.log("Step and feed must be numbers", "err")
            return
        fw = self.fw_var.get()
        feed = min(feed, JOG_MAX_FEED.get(fw, {}).get(axis, feed))
        if fw == "GRBL":
            self.send_line(f"$J=G91 G21 {axis}{step:g} F{feed:g}")
        elif self.fw_var.get() in ABSOLUTE_ONLY:
            work = self.tracker.work()
            if work is None:
                self.log("Position unknown: press Home first.", "err")
                return
            target = work[axis] + step
            if axis in self.tracker.limits:
                lo, hi = self.tracker.work_range(axis)
                clamped = min(max(target, lo), hi)
                if abs(clamped - work[axis]) < 1e-9:
                    self.log(f"{axis} is already at its soft limit", "err")
                    return
                if clamped != target:
                    self.log(f"{axis} jog shortened to stop at the soft limit", "info")
                target = clamped
            self.send_line(f"G0 {axis}{round(target, 3):g} F{feed:g}")  # same shape as Cirqoid's own programs
        else:
            for line in ("G91", f"G1 {axis}{step:g} F{feed:g}", "G90"):
                self.send_line(line)

    def estop(self):
        if not self.link:
            return
        self.link.clear()
        self._discard_unsent()
        self.link.realtime(FIRMWARE[self.fw_var.get()]["estop"])
        self._set_spindle(False)
        self.job_total = self.job_done = 0
        if self.fw_var.get() == "Cirqoid":
            self.log("*** STOP: queue cleared, spindle off (M5). Cirqoid has no software e-stop: moves already "
                     "sent will finish - use the machine's power switch to halt motion. ***", "err")
        else:
            self.log("*** E-STOP sent — queue cleared ***", "err")
        self._update_controls()

    # ---------- file streaming ----------
    def load_file(self):
        path = filedialog.askopenfilename(
            filetypes=[("G-code", "*.gcode *.nc *.ngc *.tap *.txt"), ("All files", "*.*")])
        if not path:
            return
        with open(path, encoding="utf-8", errors="replace") as fh:
            lines = clean_gcode(fh.read())
        if self.fw_var.get() in ABSOLUTE_ONLY and looks_like_cam(lines) and messagebox.askyesno(
                "Load", "This looks like CAM G-code: its Z is measured from the copper surface "
                        "(Z0 = copper, positive = above it), not from the machine's top.\n\n"
                        "Convert it with Import CAM, using the board origin and surface Z?"):
            self.import_cam(path)
            return
        self.job_lines = lines
        self.file_var.set(f"{path.replace(chr(92), '/').split('/')[-1]} — {len(self.job_lines)} lines")
        self.log(f"Loaded {path}: {len(self.job_lines)} lines", "info")
        self._update_controls()

    # ---------- PCB: import CAM G-code ----------
    def _known_position(self):
        work = self.tracker.work() if self.tracker else None
        if work is None:
            self.log("Position unknown: connect as Cirqoid and press Home first.", "err")
        return work

    def set_origin_here(self):
        work = self._known_position()
        if work:
            self.origin_x_var.set(fmt(work["X"]))
            self.origin_y_var.set(fmt(work["Y"]))
            self.log(f"Board origin set to X{fmt(work['X'])} Y{fmt(work['Y'])}", "info")

    def set_surface_here(self):
        work = self._known_position()
        if work:
            self.surface_var.set(fmt(work["Z"]))
            self.log(f"Copper surface set to Z{fmt(work['Z'])} (re-do this after every bit change)", "info")

    def import_cam(self, path=None):
        """Load G-code from FlatCAM / pcb2gcode, converted to the Cirqoid dialect at the board's position."""
        try:
            origin_x, origin_y, surface = (float(v.get()) for v in
                                           (self.origin_x_var, self.origin_y_var, self.surface_var))
            air = float(self.air_offset_var.get()) if self.air_var.get() else 0.0
        except ValueError:
            messagebox.showerror("Import CAM", "Set the board origin (X, Y) and the surface Z first.\n\n"
                                 "Home, jog the bit over the board corner and press 'Set origin here', then "
                                 "lower it onto the copper and press 'Set surface Z here'.")
            return
        path = path or filedialog.askopenfilename(
            title="G-code from FlatCAM / pcb2gcode",
            filetypes=[("CAM G-code", "*.nc *.ngc *.gcode *.cnc *.tap *.txt"), ("All files", "*.*")])
        if not path:
            return
        options = CamOptions(air_offset=air, max_spindle=SPINDLE_MAX,
                             max_z_feed=JOG_MAX_FEED["Cirqoid"]["Z"], rapid_z_feed=JOG_MAX_FEED["Cirqoid"]["Z"])
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                result = convert_cam(fh.read(), origin_x, origin_y, surface, options)
        except ConvertError as exc:
            messagebox.showerror("Import CAM", f"Can't convert this file:\n\n{exc}")
            return
        index = 0 if len(result.sections) == 1 else self._pick_section(result.sections)
        if index is None:
            return
        section = result.sections[index]
        self.job_lines = list(section.lines)
        name = path.replace(chr(92), "/").split("/")[-1]
        tool = "" if section.tool == "(no tool)" else f" · {section.tool}"
        self.file_var.set(f"{name}{tool} — {len(self.job_lines)} lines, converted")
        self.log(f"Imported {name}: {section.tool}{' - ' + section.note if section.note else ''}, "
                 f"{len(self.job_lines)} lines", "info")
        for warning in result.warnings:
            self.log(f"  note: {warning}", "rx")
        self.log("  moves span " + ", ".join(f"{axis} {fmt(lo)}..{fmt(hi)}"
                                             for axis, (lo, hi) in sorted(section.bounds.items())), "info")
        if len(result.sections) > 1:
            self.log(f"  this file has {len(result.sections)} tools ("
                     + ", ".join(s.tool for s in result.sections)
                     + "): after this one, change the bit, set the surface Z again and import the next.", "info")
        if air:
            self.log(f"  AIR TEST: the whole job is lifted {fmt(air)} mm. Untick 'Air test' and import again "
                     "for the real cut.", "info")
        if self.fw_var.get() != "Cirqoid":
            self.log("  note: converted for the Cirqoid; the current firmware setting is "
                     f"{self.fw_var.get()}", "err")
        self._update_controls()

    def _pick_section(self, sections):
        """Ask which tool section of a multi-tool file to load. Returns its index, or None."""
        win = tk.Toplevel(self.root, bg=self.palette["bg"])
        win.title("Pick the tool to run")
        win.transient(self.root)
        win.grab_set()
        ttk.Label(win, text="This file uses several tools. Load one at a time: change the bit and set the "
                            "surface Z before each.").pack(padx=8, pady=6)
        p = self.palette
        box = tk.Listbox(win, width=80, height=min(10, len(sections)), bg=p["field"], fg=p["fg"],
                         selectbackground=p["select"], selectforeground=p["fg"], highlightthickness=0)
        for section in sections:
            box.insert("end", f"{section.tool}   {section.note}   ({len(section.lines)} lines)")
        box.selection_set(0)
        box.pack(padx=8)
        choice = {}

        def load():
            selected = box.curselection()
            choice["index"] = selected[0] if selected else None
            win.destroy()

        box.bind("<Double-1>", lambda e: load())
        row = ttk.Frame(win)
        row.pack(pady=6)
        ttk.Button(row, text="Load", command=load).pack(side="left", padx=4)
        ttk.Button(row, text="Cancel", command=win.destroy).pack(side="left", padx=4)
        win.wait_window()
        return choice.get("index")

    def start_job(self):
        if not (self.link and self.job_lines):
            return
        lines = [self._dialect(line) for line in self.job_lines]
        snapshots = [None] * len(lines)
        if self.tracker:  # check the whole file against the soft limits before sending anything
            preview = self.tracker.copy()
            for number, line in enumerate(lines, 1):
                snapshots[number - 1] = preview.copy()
                error = preview.apply(line)
                if error:
                    self.log(f"Job not started: line {number} '{line}': {error}", "err")
                    if looks_like_cam(self.job_lines):
                        self.log("  This file's Z is measured from the copper surface (CAM G-code): open it "
                                 "with Import CAM… instead of Load…", "err")
                    return
            self.tracker = preview
        self.job_total, self.job_done = len(lines), 0
        for line, snapshot in zip(lines, snapshots):
            self._enqueue(line, snapshot, job=True)
        self._show_position()
        self.pause_btn.configure(text="Pause")
        self.log("Job started", "info")
        self._update_progress()
        self._update_controls()

    def pause_job(self):
        fw = FIRMWARE[self.fw_var.get()]
        if self.link.paused.is_set():
            self.link.paused.clear()
            if fw["resume"]:
                self.link.realtime(fw["resume"])
            self.pause_btn.configure(text="Pause")
            self.log("Job resumed", "info")
        else:
            self.link.paused.set()
            if fw["hold"]:
                self.link.realtime(fw["hold"])
            self.pause_btn.configure(text="Resume")
            self.log("Job paused (moves already buffered may still finish)" if not fw["hold"]
                     else "Job paused (feed hold)", "info")

    def stop_job(self):
        self.link.clear()
        self._discard_unsent()
        self.job_total = self.job_done = 0
        self.log("Job stopped — no more lines will be sent (use E-STOP to halt motion)", "info")
        self._update_progress()
        self._update_controls()

    def _update_progress(self):
        if self.job_total:
            self.progress.configure(maximum=self.job_total, value=self.job_done)
            self.progress_var.set(f"{self.job_done}/{self.job_total}")
        else:
            self.progress.configure(value=0)
            self.progress_var.set("")

    # ---------- event loop ----------
    def _drain_events(self):
        try:
            while True:
                kind, *rest = self.events.get_nowait()
                if kind == "tx":
                    self.log(rest[0], "tx")
                    self.sent.add(rest[1])
                    self._on_sent(rest[0])
                elif kind == "rx":
                    self._handle_rx(rest[0])
                elif kind == "error":
                    self.log(rest[0], "err")
                elif kind == "log":
                    self.log(*rest)
                elif kind == "detect_done":
                    self._detect_done(rest[0])
                elif kind == "line_done":
                    self._line_done(*rest)
        except queue.Empty:
            pass
        self.root.after(50, self._drain_events)

    def _handle_rx(self, text):
        if text.startswith("<") and text.endswith(">"):  # GRBL status report
            fields = text[1:-1].split("|")
            pos = next((f.split(":", 1)[1] for f in fields if f.startswith(("MPos:", "WPos:"))), "")
            x, y, z = (pos.split(",") + ["?", "?", "?"])[:3]
            self._set_position(x, y, z, fields[0])
            return
        match = MARLIN_POS_RE.search(text)
        if match:
            self._set_position(*match.groups(), "reported by the machine")
        if text == "ok":
            self._log_ok()
            return
        self.log(text, "err" if text.lower().startswith(("error", "alarm", "nack")) else "rx")

    def _poll_status(self):
        if self.link and self.fw_var.get() == "GRBL":
            try:
                self.link.realtime(b"?")
            except Exception as exc:
                self.log(f"status poll failed: {exc}", "err")
                self.disconnect()
        self.root.after(250, self._poll_status)

    def on_close(self):
        if self.detector:
            self.detector.cancel.set()
        if self.link:
            self.link.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
