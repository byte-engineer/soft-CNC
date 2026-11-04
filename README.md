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
- `cirqoid_square.gcode` homes, keeps Z at the top (`Z0.0`) and traces a 90 mm square in the air.
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

`test_square.gcode` is a 10 mm square at Z=0, starting from the current zero. Set zero with the bit lifted clear of the work before you run it.
