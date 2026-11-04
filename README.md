# Soft-CNC

Small G-code control panel for a USB-connected CNC (test program). Tkinter + pyserial, run through uv.

```
uv run cnc_ui.py
```

(uv is configured to use its managed Python, because the system Python here has no tkinter.)

## Use

1. Pick the **Port** (the machine shows up as `COM8 — USB Serial Port`), the **Baud** and the **Firmware**, then **Connect**.
   Choose `SIMULATOR` to try everything without hardware.
   **Auto-detect** tries every common baud rate, then rates 2–4% off them, using info queries only
   (`$I`, `?`, `M115`, `M114`, no motion). It fills in Baud and Firmware for you (takes up to ~2 minutes).
2. **Jog**: X/Y/Z buttons move by *Step* mm at *Feed* mm/min (GRBL uses `$J=` jog commands; others use `G91 / G1 / G90`).
3. **Console**: type any G-code line and press Enter (↑/↓ for history). `?`, `!`, `~` are sent as GRBL real-time characters.
4. **G-code file**: Load → Start. Lines go out one at a time, each waiting for `ok`. Pause / Stop stop sending more lines.
5. **Spindle**: set the speed (S, whole number, steps of 50, max 1400) and press **Start (M3)**, which sends `M3 S<speed>`.
   **Set speed** changes the speed while it runs. **Stop (M5)** sends `M5`. The label shows the state the app last sent.
6. **E-STOP** clears the queue and sends `Ctrl-X` (GRBL soft reset) or `M112` (Marlin/Generic).
   It's a software stop. Keep the machine's hardware e-stop within reach.

| Button   | Cirqoid (default) | GRBL | Marlin / Generic |
|----------|-------------------|------|------------------|
| Home     | `G90`, `G21`, `G28 Y0.835` | `$H` | `G28` |
| Unlock   | – | `$X` | `M999` / – |
| Position | – (app shows the position it last sent) | `?` (also polled every 250 ms) | `M114` |
| Feed hold / Resume | – | `!` / `~` | – |
| E-STOP   | `M5` (spindle off only) | `Ctrl-X` | `M112` |

**Cirqoid notes** (38400 baud, Newline):
- It sends **no reply** for commands it accepts. The app waits 0.5 s per line for a possible `nack`,
  treats silence as accepted, and shows any `nack` in red.
- Example programs are in `examples/` (see **Examples** below).
  Make sure nothing on the bed is high enough to be hit at that depth.
- Its controller only takes absolute moves (no `G91`). Press **Home** (`G90`, `G21`, `G28 Y0.835`) before moving.
  The app then tracks the position from what it sends and jogs with absolute moves like `G0 X1.0 F500.0`
  (every number gets a decimal point). Moves and homing are answered with `ok` or `error: ...`; the app waits for that answer (up to 2 minutes for homing). If the machine refuses a move, the tracked position stays where it was and a running job stops.
- **Soft limits**, measured from home: X 0 to 103.0, Y 0 to 224.835, Z -26.9 to 0 mm (`SOFT_LIMITS` in `cnc_ui.py`).
  Z is 0 at the top (home) and negative going down; -26.9 is the lowest point (the controller refuses positive Z).
  Z jogs are capped at F300, like Cirqoid's own programs.
  A jog that would cross a limit is shortened to stop at it. A typed move outside a limit is not sent.
  A loaded file is checked line by line before **Start**, and nothing is sent if any move goes outside.
  If you set a work zero with **Zero XYZ** (`G92`), the limits still apply to the real position from home.
  Arcs (`G2`/`G3`) are checked at their end point only.
- There's no software e-stop. E-STOP clears the queue and turns the spindle off, but moves already sent still finish.
  Use the machine's power switch to halt motion.
- If every byte you send gets `nack: communication error`, the USB-serial adapter is in a bad state.
  Unplug the USB cable, power-cycle the machine, and plug the cable back in. (Rapid baud-rate switching
  on an open port caused this once.)

## Examples

These are ready-made Cirqoid G-code, so open them with **Load…**. **Import CAM…** is only for G-code from
FlatCAM / pcb2gcode, which still needs converting. Both examples home first and stay at the top (`Z0.0`), so
they move in the air.
- `examples/cirqoid_small_square.gcode`: a 10 mm square at X10–20, Y10–20. Run this first.
- `examples/cirqoid_square.gcode`: a 90 mm square at X5–95, Y5–95, traced four times.
- `examples/etch_square_circle.gcode`: spindle on, then a 20 × 20 mm square with an 8 mm-radius circle inside,
  etched 0.2 mm deep. It's written CAM-style (X0 Y0 = square corner, Z0 = copper surface), so open it with
  **Import CAM…** after **Set origin here** and **Set surface Z here**, not with **Load…**.

## Making a PCB (FlatCAM / pcb2gcode → Cirqoid)

Make the toolpaths in a CAM tool. **FlatCAM** is recommended: it has a GUI, and its *Double-Sided PCB* tool
handles mirroring and alignment holes. **pcb2gcode** works too. Then use **Import CAM…** in this app.
It converts the file to the Cirqoid dialect (`cam_import.py`):
- every number gets a decimal point and every move an F;
- Z is moved from "0 = copper surface" to the machine (0 = top, negative = down);
- arcs become short straight moves;
- `G81`/`G83` drill cycles are expanded;
- inches become mm;
- spindle speed is capped at S1400 and Z moves at F300;
- tool changes split the file into one part per tool.

**CAM settings:** units mm, absolute coordinates, board corner at X0 Y0.
Start with the values below and adjust them to your bits:

| Job | Cut Z | Travel Z | Feed XY / Z |
|-----|-------|----------|-------------|
| Isolation (V-bit) | −0.05 to −0.1 | 2 | 100–150 / 50–60 mm/min |
| Drilling | −1.8 (1.6 mm board + margin) | 2 | — / 60–150 mm/min |
| Board outline (end mill) | −1.8, in several passes | 2 | 100 / 50 mm/min |

Spindle up to 1400. Export one G-code file per job, or one file with tool changes.

**For each tool:**
1. Fix the copper board to the bed and insert the bit.
2. Connect and press **Home**.
3. Jog the bit over the board's corner (CAM X0 Y0) and press **Set origin here**.
4. Lower Z in small steps until the bit just touches the copper (paper test) and press **Set surface Z here**.
   Do this again after every bit change, because bits differ in length.
5. Optional dry run: tick **Air test** (off by default) and press **Import CAM…**. The whole job is lifted
   by the offset (5 mm), so **Start** cuts only air.
6. With **Air test** unticked, press **Import CAM…** and **Start** the real cut.

A file with several tools asks which one to load. Run them one at a time, repeating steps 1 and 4 for each.
Before anything is sent, the converted job is checked against the soft limits.
Nothing runs if any move would leave the machine's travel, and the console names the offending line.

**Double-sided boards:** in FlatCAM's Double-Sided tool, add alignment holes and mirror the bottom layer about
the axis it shows.
1. Drill the alignment holes and mill the top side.
2. Flip the board onto pins in those holes.
3. Run the mirrored bottom files with the **same origin**.

**Not yet checked on the machine:** whether it accepts `G4` dwell (so the converter skips dwells) and `G2`/`G3`
arcs (so they're converted to straight lines).
# soft-CNC
