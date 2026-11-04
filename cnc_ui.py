"""Small G-code control panel for a USB-connected CNC (test program).

Run with:  uv run cnc_ui.py
Pick the port "SIMULATOR" to try the UI without a machine attached.
"""

import queue
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

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

        root.title("Soft-CNC — G-code control (test)")
        root.geometry("1000x740")
        root.minsize(840, 680)
        self._build()
        self._set_connected(False)
        self.refresh_ports()
        root.after(50, self._drain_events)
        root.after(250, self._poll_status)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- layout ----------
    def _build(self):
        pad = {"padx": 4, "pady": 4}

        top = ttk.Frame(self.root)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=34, state="readonly")
        self.port_box.pack(side="left", **pad)
        ttk.Button(top, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        ttk.Label(top, text="Baud").pack(side="left", padx=(12, 0))
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD["Cirqoid"])
        ttk.Combobox(top, textvariable=self.baud_var, values=BAUDS, width=8).pack(side="left", **pad)
        ttk.Label(top, text="Firmware").pack(side="left", padx=(12, 0))
        self.fw_var = tk.StringVar(value="Cirqoid")
        fw_box = ttk.Combobox(top, textvariable=self.fw_var, values=list(FIRMWARE), width=8, state="readonly")
        fw_box.pack(side="left", **pad)
        fw_box.bind("<<ComboboxSelected>>", lambda e: self.baud_var.set(DEFAULT_BAUD[self.fw_var.get()]))
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=(12, 4))
        self.auto_btn = ttk.Button(top, text="Auto-detect", command=self.auto_detect)
        self.auto_btn.pack(side="left", padx=(0, 12))
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var, font=("Consolas", 10)).pack(side="left")

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, **pad)

        # Left: jog + machine commands + e-stop
        left = ttk.Frame(body)
        left.pack(side="left", fill="y")

        jog = ttk.LabelFrame(left, text="Jog")
        jog.pack(fill="x", **pad)
        grid = ttk.Frame(jog)
        grid.pack(**pad)
        b = lambda text, axis, sign, r, c: ttk.Button(
            grid, text=text, width=6, command=lambda: self.jog(axis, sign)).grid(row=r, column=c, padx=2, pady=2)
        b("Y+", "Y", 1, 0, 1)
        b("X−", "X", -1, 1, 0)
        b("X+", "X", 1, 1, 2)
        b("Y−", "Y", -1, 2, 1)
        b("Z↑", "Z", 1, 0, 4)
        b("Z↓", "Z", -1, 2, 4)
        self.jog_buttons = grid.winfo_children()

        opts = ttk.Frame(jog)
        opts.pack(fill="x", **pad)
        ttk.Label(opts, text="Step mm").grid(row=0, column=0, sticky="w")
        self.step_var = tk.StringVar(value="1")
        ttk.Combobox(opts, textvariable=self.step_var, values=["0.1", "1", "5", "10", "50"],
                     width=6).grid(row=0, column=1, **pad)
        ttk.Label(opts, text="Feed mm/min").grid(row=1, column=0, sticky="w")
        self.feed_var = tk.StringVar(value="500")
        ttk.Entry(opts, textvariable=self.feed_var, width=8).grid(row=1, column=1, **pad)

        cmds = ttk.LabelFrame(left, text="Machine")
        cmds.pack(fill="x", **pad)
        self.machine_buttons = []
        for i, (text, fn) in enumerate([
            ("Home", lambda: self.fw_command("home")),
            ("Unlock", lambda: self.fw_command("unlock")),
            ("Zero XYZ (G92)", lambda: self.send_line("G92 X0 Y0 Z0")),
            ("Position", lambda: self.fw_command("position")),
            ("Feed hold", lambda: self.fw_command("hold")),
            ("Resume", lambda: self.fw_command("resume")),
        ]):
            btn = ttk.Button(cmds, text=text, command=fn)
            btn.grid(row=i // 2, column=i % 2, sticky="ew", padx=2, pady=2)
            self.machine_buttons.append(btn)

        spindle = ttk.LabelFrame(left, text="Spindle")
        spindle.pack(fill="x", **pad)
        ttk.Label(spindle, text="Speed (S)").grid(row=0, column=0, sticky="w", padx=4)
        self.speed_var = tk.StringVar(value="1000")
        self.speed_box = ttk.Spinbox(spindle, textvariable=self.speed_var, from_=SPINDLE_STEP, to=SPINDLE_MAX,
                                     increment=SPINDLE_STEP, width=8)
        self.speed_box.grid(row=0, column=1, **pad)
        self.spindle_state_var = tk.StringVar(value="off")
        ttk.Label(spindle, textvariable=self.spindle_state_var, width=10).grid(row=0, column=2, padx=4)
        self.spindle_buttons = [self.speed_box]
        for i, (text, fn) in enumerate([("Start (M3)", self.spindle_start), ("Set speed", self.spindle_set),
                                        ("Stop (M5)", self.spindle_stop)]):
            btn = ttk.Button(spindle, text=text, command=fn)
            btn.grid(row=1, column=i, sticky="ew", padx=2, pady=2)
            self.spindle_buttons.append(btn)

        self.estop_btn = tk.Button(left, text="E-STOP", bg="#c62828", fg="white",
                                   activebackground="#8e0000", activeforeground="white",
                                   font=("Segoe UI", 16, "bold"), command=self.estop)
        self.estop_btn.pack(fill="x", padx=4, pady=12, ipady=10)

        # Right: console
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)
        console = ttk.LabelFrame(right, text="Console")
        console.pack(fill="both", expand=True, **pad)
        self.log_text = tk.Text(console, height=10, font=("Consolas", 10), state="disabled",
                                bg="#111", fg="#ddd", insertbackground="#ddd")
        scroll = ttk.Scrollbar(console, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_configure("tx", foreground="#7fc7ff")
        self.log_text.tag_configure("rx", foreground="#cfcfcf")
        self.log_text.tag_configure("err", foreground="#ff6b6b")
        self.log_text.tag_configure("info", foreground="#9ccc65")

        entry_row = ttk.Frame(right)
        entry_row.pack(fill="x", **pad)
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(entry_row, textvariable=self.cmd_var, font=("Consolas", 11))
        self.cmd_entry.pack(side="left", fill="x", expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self.send_entry())
        self.cmd_entry.bind("<Up>", lambda e: self.history_step(-1))
        self.cmd_entry.bind("<Down>", lambda e: self.history_step(1))
        self.send_btn = ttk.Button(entry_row, text="Send", command=self.send_entry)
        self.send_btn.pack(side="left", padx=4)

        # Bottom: G-code file streaming
        job = ttk.LabelFrame(self.root, text="G-code file")
        job.pack(fill="x", **pad)
        ttk.Button(job, text="Load…", command=self.load_file).pack(side="left", **pad)
        self.file_var = tk.StringVar(value="(no file)")
        ttk.Label(job, textvariable=self.file_var, width=36).pack(side="left", **pad)
        self.start_btn = ttk.Button(job, text="Start", command=self.start_job)
        self.start_btn.pack(side="left", **pad)
        self.pause_btn = ttk.Button(job, text="Pause", command=self.pause_job)
        self.pause_btn.pack(side="left", **pad)
        self.stop_btn = ttk.Button(job, text="Stop", command=self.stop_job)
        self.stop_btn.pack(side="left", **pad)
        self.progress = ttk.Progressbar(job, length=200)
        self.progress.pack(side="left", fill="x", expand=True, **pad)
        self.progress_var = tk.StringVar(value="")
        ttk.Label(job, textvariable=self.progress_var, width=14).pack(side="left", **pad)

    # ---------- helpers ----------
    def log(self, text, tag="rx"):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n", tag)
        if int(self.log_text.index("end-1c").split(".")[0]) > 3000:
            self.log_text.delete("1.0", "500.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def job_running(self):
        return self.job_total > 0 and self.job_done < self.job_total

    def _set_connected(self, connected):
        self.connect_btn.configure(text="Disconnect" if connected else "Connect")
        if not connected:
            self.status_var.set("Disconnected")
        self._update_controls()

    def _update_controls(self):
        connected = self.link is not None
        running = self.job_running()
        manual = "normal" if connected and not running else "disabled"
        for w in self.jog_buttons + self.machine_buttons + self.spindle_buttons + [self.send_btn, self.cmd_entry]:
            w.configure(state=manual)
        if connected and self.fw_var.get() == "GRBL":  # real-time commands work mid-job on GRBL
            for w in self.machine_buttons[4:]:
                w.configure(state="normal")
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
        self.tracker = None
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
        self._note_spindle(line)
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

    def _show_position(self):
        work = self.tracker.work() if self.tracker else None
        if work:
            self.status_var.set(f"X {work['X']:.3f}  Y {work['Y']:.3f}  Z {work['Z']:.3f}  (as sent)")

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
        self.spindle_state_var.set(f"on S{speed}" if on and speed else "on" if on else "off")

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
            self.job_lines = clean_gcode(fh.read())
        self.file_var.set(f"{path.replace(chr(92), '/').split('/')[-1]} ({len(self.job_lines)} lines)")
        self.log(f"Loaded {path}: {len(self.job_lines)} lines", "info")
        self._update_controls()

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
            self.status_var.set(f"{fields[0]:<6}  X {x}  Y {y}  Z {z}")
            return
        match = MARLIN_POS_RE.search(text)
        if match:
            self.status_var.set("X {}  Y {}  Z {}".format(*match.groups()))
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
