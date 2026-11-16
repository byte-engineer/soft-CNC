# Soft-CNC

Small G-code control panel for the Cirqoid PCB mill (and GRBL / Marlin machines) over USB serial.
Written in Rust with [egui](https://github.com/emilk/egui). The earlier Python/Tkinter version is at git tag
`v1.0-python`.

```
cargo run --release
```

The app is then also at `target/release/soft-cnc.exe` (a single file you can copy anywhere).
Run the tests with `cargo test`.

## Use

1. Pick the **Port** (the machine shows up as `COM8 — USB Serial Port`), the **Baud** and the **Firmware**, then **Connect**.
   Without hardware, pick `SIMULATOR-CIRQOID` (behaves like the Cirqoid) or `SIMULATOR` (GRBL-like).
   **Auto-detect** tries every common baud rate, then rates 2–4% off them, using info queries only
   (`$I`, `?`, `M115`, `M114`, no motion). It fills in Baud and Firmware for you (takes up to ~2 minutes).
2. **Jog**: X/Y/Z buttons move by *Step* mm at *Feed* mm/min (GRBL uses `$J=` jog commands; others use `G91 / G1 / G90`).
   Keys: ← → jog X, ↑ ↓ jog Y, PgUp/PgDn jog Z, **Esc = E-STOP** (not while typing in a box).
3. **Console** tab: type any G-code line and press Enter (↑/↓ for history). `?`, `!`, `~` are sent as GRBL real-time
   characters. An `ok` reply is shown as ✓ ok on the line it answers.
4. **PCB job** tab: board position, **Load…** (a G-code file, or Gerber + drill files — see below), the **Jobs**
   list and **Start / Pause / Stop** with progress.
   Lines go out one at a time, each waiting for the controller's answer. The latest message is shown there too.
5. **Spindle**: set the speed (S, whole number, steps of 50, max 1400) and press **Start M3**, which sends `M3 S<speed>`.
   **Set speed** changes the speed while it runs. **Stop M5** sends `M5`. The label shows the state last sent.
6. **E-STOP** clears the queue and sends `Ctrl-X` (GRBL soft reset) or `M112` (Marlin/Generic).
   It's a software stop. Keep the machine's hardware e-stop within reach.
7. **☾ Dark / ☀ Light** switches the theme; the choice is remembered in `~/.soft-cnc.json`.

| Button   | Cirqoid (default) | GRBL | Marlin / Generic |
|----------|-------------------|------|------------------|
| Home     | `G90`, `G21`, `G28 Y0.835` | `$H` | `G28` |
| Unlock   | – | `$X` | `M999` / – |
| Position | – (the readout shows the position last sent) | `?` (also polled every 250 ms) | `M114` |
| Feed hold / Resume | – | `!` / `~` | – |
| E-STOP   | `M5` (spindle off only) | `Ctrl-X` | `M112` |

**Cirqoid notes** (38400 baud, Newline):
- It sends **no reply** for setting lines it accepts (`G90`, `G21`, `M3`…). The app waits 0.5 s per such line for a
  possible `nack`, treats silence as accepted, and shows any `nack` in red.
- Its controller only takes absolute moves (no `G91`). Press **Home** (`G90`, `G21`, `G28 Y0.835`) before moving.
  The app then tracks the position from what it sends and jogs with absolute moves like `G0 X1.0 F500.0`
  (every number gets a decimal point). Moves and homing are answered with `ok` or `error: ...`; the app waits for that
  answer (up to 2 minutes for homing). If the machine refuses a move, the position stays where it was and a running
  job stops.
- **Soft limits**, measured from home: X 0 to 103.0, Y 0 to 224.835, Z -26.9 to 0 mm (`CIRQOID` in `src/firmware.rs`).
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

## Loading G-code

There's one **Load…** button, and any G-code file can be loaded. With the Cirqoid selected, every file is
rewritten into G-code the Cirqoid understands (`src/cam.rs`):
- every number gets a decimal point and every move an F;
- relative moves (`G91`) become absolute positions;
- arcs (`G2`/`G3`) become short straight moves;
- `G81`/`G83` drill cycles are spelled out;
- inches become mm;
- spindle speed is capped at S1400 and Z moves at F300;
- tool changes split the file into one part per tool;
- codes the Cirqoid can't use (dwell, coolant…) are skipped, with a note in the console.

How the positions are read is decided from the file itself, and the console says which it chose (`read as …`):
- **CAM G-code** (from FlatCAM / pcb2gcode: it goes above Z0 and never homes) measures Z from the copper surface,
  so it is placed at the **board origin** and **surface Z** set on the PCB job tab. If those aren't set yet, the app
  says so instead of loading.
- **Anything else** is taken to be in machine coordinates already (Z0 = machine top, like the examples): positions
  are kept, including its homing line.

Other firmwares (GRBL, Marlin, Generic) understand ordinary G-code, so their files are sent as written.

## Examples

Open them with **Load…**. The two squares are in machine coordinates: they home first and stay at the top
(`Z0.0`), so they move in the air.
- `examples/cirqoid_small_square.gcode`: a 10 mm square at X10–20, Y10–20. Run this first.
- `examples/cirqoid_square.gcode`: a 90 mm square at X5–95, Y5–95, traced four times.
- `examples/etch_square_circle.gcode`: spindle on, then a 20 × 20 mm square with an 8 mm-radius circle inside,
  etched 0.2 mm deep. It's written CAM-style (X0 Y0 = square corner, Z0 = copper surface), so press
  **Set origin here** and **Set surface Z here** before loading it.

## Making a PCB straight from Gerber files

No separate CAM program is needed: the app does what FlatCAM does (`src/pcb/`).

1. **Export from your PCB program** (KiCad: *File → Plot* for **F.Cu**, **B.Cu** if double-sided, and
   **Edge.Cuts**; then *Generate Drill Files*, Excellon, millimetres). KiCad's X2 attributes say which layer each
   file is; other programs' files are recognised by name (`.gtl`, `.gbl`, `.gko`, `.drl`…).
2. **Load…** and select all of those files together. The **PCB setup** opens:
   - the layers it found, the board size, and any notes;
   - the tool settings (remembered for next time):

     | Job | Defaults |
     |-----|----------|
     | Isolation (V-bit) | cut width 0.2 mm, 2 passes, 40 % overlap, 0.1 mm deep, F120 |
     | Drilling | 1.8 mm deep (1.6 mm board + margin), plunge F60, one job per drill size |
     | Cutout (end mill) | 1 mm tool, 1.8 mm deep in 0.6 mm passes, F100, 4 tabs 2 mm wide, 0.6 mm thick |
     | General | travel height 2 mm, plunge F50, spindle S1000 |

   - a **preview** of copper and toolpaths (hover a job to highlight it), and the jobs in the order to run them.
3. **Create jobs**. They appear in the **Jobs** list on the PCB job tab. X0 Y0 is the board's lower-left
   corner, Z0 the copper surface.
4. **For each job:** fit the bit it names, jog to the board corner and **Set origin here** (once), lower onto the
   copper and **Set surface Z here** (after every bit change), press **Load** next to the job, then **Start**.
   Each job is converted when you load it, so it always uses the current surface Z.

**Double-sided boards** (needs top and bottom copper; tick *Mirror the bottom layer, add alignment holes*):
1. **Alignment holes** first: two holes on the board's centre line, 5 mm below and above it, drilled through into
   the bed. Put pins in them.
2. Top isolation, then the drills.
3. Flip the board **left-to-right** onto the pins, then run the **bottom isolation** (mirrored about the centre
   line, so the board corner stays at the same origin).
4. Flip back and run the **cutout**.

A board with only bottom copper is milled face-down: all its jobs are mirrored.

**Not yet checked on the machine:** whether it accepts `G4` dwell (so the converter skips dwells) and `G2`/`G3`
arcs (so they're converted to straight lines).

**Not yet checked on the machine:** whether it accepts `G4` dwell (so the converter skips dwells) and `G2`/`G3`
arcs (so they're converted to straight lines).

## Code

| File | What it does |
|------|--------------|
| `src/app.rs` | The egui window: cards, tabs, theme, keyboard, Jobs list |
| `src/app/pcb_setup.rs` | The PCB setup dialog: layers, tool settings, preview |
| `src/pcb/gerber.rs` | Gerber reader (apertures, KiCad macros, tracks, arcs, zones, polarity) |
| `src/pcb/excellon.rs` | Drill-file reader (holes, slots, mm/inch) |
| `src/pcb/mod.rs` | Recognises layers, finds the board shape, moves the board to X0 Y0 |
| `src/pcb/toolpath.rs` | Isolation, drilling, cutout with tabs, double-sided jobs |
| `src/controller.rs` | Everything the app does with the machine, without UI (tested headless) |
| `src/link.rs` | Serial threads: send a line, wait for its answer; read replies |
| `src/firmware.rs` | Cirqoid / GRBL / Marlin / Generic profiles and limits |
| `src/tracker.rs` | Position tracking and soft limits |
| `src/cam.rs` | Any G-code (incl. FlatCAM / pcb2gcode output) → Cirqoid converter |
| `src/gcode.rs` | G-code text helpers (decimal points, comments, CAM detection) |
| `src/detect.rs` | Auto-detect of baud rate and firmware |
| `src/sim.rs` | Built-in simulators |
| `tests/controller.rs` | End-to-end tests against the simulators |
| `tests/pcb.rs` | Gerber + drill files → jobs, with KiCad-style sample files |
