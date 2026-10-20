# How to test Buddy 3 (Line and Barcode), step by step

This guide tells you exactly what to do for every Buddy 3 test in
`TEST_LOG.md`, in the order to do them. Parts 1–4 need only the PC and the
Pico W's own USB to the computer (the wheels never turn). Part 5 needs the real car with Buddy
2's Motion Control. How to record a result is in Part 6 and in
`../README.md`.

---

## Safety first

- **Power: two USB sockets, one rule.** The Pico W's own USB socket may
  go to the computer (console and flashing). The Robo Pico's power input
  goes to the power bank. **Never both at the same time**, and **never
  plug the Robo Pico's power input into the computer.**
  - Parts 1–4: Pico W USB to the computer, Robo Pico power unplugged.
  - Part 5: Robo Pico power from the power bank, Pico W USB unplugged.
- In Parts 1–4 the motors are never driven (`SYS_SIM_CAR` is 1: a
  simulated car moves instead), so the car can sit on the desk.
- **GP19 conflict.** The course template's subsystem 1 example still
  blinks an LED on **GP19**, the barcode sensor's pin. While that example
  is in the build, the Pico drives GP19 as an output against the sensor,
  and barcodes cannot be read. Until subsystem 1's owner removes or moves
  it, do this **for your bench build only, and do not commit it**:
  in `app_program/subsystem1/subsystem1.h` change `S1_LED_PIN` from `19`
  to `14`. Afterwards undo it with
  `git checkout app_program/subsystem1/subsystem1.h`.

## What you need

| Item | For |
|---|---|
| Car with Pico W on the Robo Pico; USB cable from the **Pico W's** socket to the computer | all |
| 3 IR modules wired: **left GP6 (Grove 5)**, **right GP7 (Grove 7)**, **barcode GP19 (header pin 25)**, each to 3V3 and GND | all |
| HC-SR04+ on Grove 4, pointing at **open space (nothing within 60 cm)** | Buddy 5's task runs too: if the sensor is missing it reports a fault and the run stops |
| Black electrical tape (about 18–19 mm wide, like the line) and white A4 paper | line sensors |
| Code 39 barcodes `*A*`, `*B*`, `*C*`, `*D*`, printed like Barcode_Sample.pdf: narrow 3 mm, wide 9 mm, 141 mm long, printed at **100 % / Actual size** | barcodes |
| Ruler, phone for photos and video | evidence |

Check the print with a ruler: each barcode must measure 141 mm end to end.

## Settings used in this guide

All in `app_program/system/sys_config.h`. Change, rebuild, reflash.

| Switch | Parts 1–4 | Part 5 (car) |
|---|---|---|
| `SYS_SIM_CAR` | 1 | 0 (when Buddy 2's Motion Control is in) |
| `SYS_SIM_LINE` | **0** (the real Buddy 3 task) | 0 |
| `SYS_CONSOLE_SENSORS` | 1 for Part 2 only, else 0 | 0 |

Build and flash (details in the top-level `README.md`, section 2):

```sh
cd build_make
make -j8            # normal image
make TEST=1 -j8     # self-test image (Part 3)
```

Hold BOOTSEL, plug the Pico W's USB into the computer, copy the `.uf2`
onto the drive that appears.

Read the console (see `../README.md`, step 4). On macOS:

```sh
ls /dev/cu.usbmodem*                      # find the port
cat /dev/cu.usbmodem1101 | tee ../testing/subsystem3/logs/2026-10-21_part2.txt
```

Start `cat` first, then press the Pico's reset (or unplug and replug USB).

**The bench trick.** In Parts 2–4 the simulated car "drives" as soon as a
run starts (2 s after power-up). If neither front sensor sees black for
250 mm of simulated travel, Buddy 3 reports the line lost and the run
stops after a search. If both see black for 400 mm it reports a stuck
sensor. So before power-up, stick a strip of black tape on white paper
and put it under the **left sensor only**. The car then "follows" a line
under its left sensor forever.

---

## Part 1: PC tests, no Pico (S3-U08, S3-U09)

```sh
make -C testing/host 2>&1 | tee testing/subsystem3/logs/2026-10-21_host.txt
```

Pass: `test_s3` prints `58 passed, 0 failed` (S3-U08) and `test_s3_task`
prints `74 passed, 0 failed` (S3-U09). Then `make -C testing/host clean`.

## Part 2: wiring and sensor polarity (S3-M01)

1. Set `SYS_CONSOLE_SENSORS` to 1, build (`make -j8`), flash, open the
   console.
2. Hold each sensor about 5 mm above white paper, then above black tape.
   Every change prints a line like:
   ```
   [line] sensors  L1 R0 B0   (1 = black)
   ```
3. Each sensor must read **1 over black, 0 over white**. If one is the
   wrong way round, turn its potentiometer until it switches cleanly; if
   all three are inverted, set `S3_IR_BLACK_LEVEL` to 0 in
   `app_program/subsystem3/s3_config.h`.
4. Record the height you used. Photo of the mounting.
5. Set `SYS_CONSOLE_SENSORS` back to 0.

Pass: all three read 1 over black and 0 over white, and switch cleanly at
the mounting height.

## Part 2b: calibrate on the bench (S3-C01 … S3-C03)

Record each value in `TEST_LOG.md` → *Calibration record*, and commit the
config change with it.

**S3-C01: front sensor spacing and height.** Measure the gap between the
two front sensors, centre to centre. It must be **25–40 mm**: wider than
the 18 mm line, so both read white when the car is centred (narrower and
the plain line looks like a junction everywhere); not much wider, or the
car wanders far before it corrects and can miss a sharp curve. Measure the
height of the sensors above the floor (these modules usually work best at
about 3–8 mm), then set the potentiometers at that height (Part 2).

**S3-C02: barcode sensor position.** Measure from the line centre to the
barcode sensor: **40–50 mm**. The barcode starts 20 mm beyond the line's
edge, 29 mm from its centre; closer than that the sensor sees the line
itself. Same height as the front sensors.

**S3-C03: pivot point.** With a ruler, measure straight back from the
point midway between the two front sensors to the wheel axle (the line
through the centres of the two drive wheels). Set `S3_PIVOT_OFFSET_MM` in
`app_program/subsystem3/s3_config.h` to that value and rebuild. Photo of
the ruler.

## Part 3: self-test image (S3-U01 … S3-U07)

1. Tape under the left sensor only (the bench trick), HC-SR04+ facing open
   space.
2. `make TEST=1 -j8`, flash, start logging, reset.
3. Look for:
   ```
   [TEST] S3-U01 PASS ...
   ...
   [TEST] S3-U07 PASS ...
   ```
Pass: all seven PASS. S3-U06 fails if `SYS_SIM_LINE` is 1 (the real task
is not started then).

## Part 4: barcodes and turns on the bench

Normal image (`make -j8`), tape under the left sensor, console logging.
After power-up you should see:

```
[bench] starting a run
[nav] Running, motion owner: line
[line] Line Following  (sensors L1 R0 B0)
```

### 4a. Read a barcode by hand (first check for S3-M03)

1. Take the printed `*C*` (go straight). Hold the page flat, 5–10 mm under
   the **barcode sensor**, with the barcode's **left end** (the start)
   nearest the sensor.
2. Slide the page **steadily** so the sensor crosses the whole barcode in
   about **1 second** (no stops). Direction: the start must reach the
   sensor first. If the console says `backwards 1`, slide the other way.
3. The console prints:
   ```
   [line] barcode 'C': result 0, command 3 (ok 1, bad 0, backwards 0)
   ```
   `result 0` = read OK; command 1 left, 2 right, 3 straight, 4 U-turn.
4. Wait 1 s between passes (the reader re-arms after 40 mm of white).
5. Do 10 passes with `*C*`. Count OK, refused (`bad` goes up) and wrong
   letters. A wrong letter is a FAIL; a refused one is not (try a steadier
   speed). Also try one pass the wrong way: `backwards` must go up and no
   command is sent.

Use only `*C*` here. A, B and D start a turn (4b): A and B wait for a
junction and ignore further barcodes until then, and D starts a spin
straight away.

### 4b. A barcode turn on the bench (bench part of S3-M07)

A turn waits for a junction, creeps to the pivot point, then the car spins
until the **leading** front sensor finds the new line (left sensor for A,
right sensor for B and D).

1. Pass `*A*` under the barcode sensor as in 4a.
2. **Junction:** for about half a second put black tape under **both**
   front sensors, then back to the left sensor only. (Not longer than 2 s,
   or it counts as a stuck sensor.)
3. About a second later (the creep):
   ```
   [line] barcode 'A': result 0, command 1 ...
   [line] Await Navigation  ...
   [nav] Running, motion owner: barcode
     [motion] turn -360 deg
   ```
4. **The spin** (the simulated car turns 90° per second, so you have about
   4 s): move the tape so the **left** sensor sees white for a moment, then
   black again. The console shows:
   ```
   [nav] Running, motion owner: line
   [line] Line Following ...
   ```
5. Do nothing in step 4 instead: after a full turn the coordinator stops
   the run (`[nav] Safe Stop`, line-lost fault). That is the safety net.
   Press **GP21** to reset and start again.
6. Repeat with `*B*` (right sensor leads) and `*D*` (U-turn: posted with no
   junction; the first 1.5 s of the spin ignore the line, then the right
   sensor leads). `*C*` never spins: the run just continues.

Video steps 1–4 for A and for D.

### 4c. Side sensor over the printed barcode (S3-M06)

Still on the Pico W's USB, motors off. Lay a strip of black tape (the line) on white
paper and a printed barcode **20 mm beside it**, as on the course. Put the
car on the tape, left sensor on the tape edge (bench trick), and **push the
car by hand** steadily along the line over the barcode (about 1 s for the
barcode). Use `*C*` again.

Pass: the console prints `barcode 'C' … result 0`, five passes out of
five. If it is refused, move the barcode sensor further out (at least 29 mm from the
line centre, about 40–50 mm is best).

---

## Part 5: on the car (needs Buddy 2's Motion Control)

Set `SYS_SIM_CAR` to 0 once Buddy 2's code is in the build. Power from the
**power bank only** (Robo Pico power input; Pico W USB unplugged), so there
is no console: until WiFi telemetry (Buddy 1) works, the evidence is
**video**. Lay a test track with black
tape on white: a straight, a curve, a + crossing and a T, with printed
barcodes 20 mm beside the line before each junction.

**Ask Buddy 2 first** for calibrated distance (encoder mm per tick: the
barcode bars are measured with it) and turns that spin on the spot (the
pivot lands on the junction only if the car turns about its axle centre).

### Calibrate on the car first (S3-C04 … S3-C07)

Change one value at a time, 3 runs each, and note what you saw.

- **S3-C04 speeds** (`S3_BASE_MM_S`, `S3_TURN_*`, `S3_SOFT_*` in
  `s3_config.h`). Start at base 150 mm/s. On the straight, if the car
  zig-zags hard, bring the two `S3_SOFT_*` speeds closer (e.g. 160 / 135).
  On the tightest curve, if it runs off, widen the `S3_TURN_*` pair
  (e.g. 200 / 40) or lower the base speed.
- **S3-C05 line lost** (`S3_LOST_MM`). 5 runs on a 2 m straight: the car
  must never stop to search for the line. Then lift it off the line during
  a run: it must stop and search. If it searches without reason, raise
  `S3_LOST_MM` by 100 or widen the `S3_SOFT_*` pair.
- **S3-C06 junction** (`S3_JUNCTION_MM`): run S3-M04 below. If the
  car spins at the curve, raise `S3_JUNCTION_MM` (it must stay below the
  crossing line's width, 18 mm); if it never spins at the crossing,
  lower it.
- **S3-C07 turn speeds** (`S3_CREEP_MM_S`, `NAV_BC_TURN_MM_S` in
  `app_program/system/nav_config.h`). From the S3-M07 videos: if the car
  pivots X mm past the junction centre, lower `S3_PIVOT_OFFSET_MM` by X
  (this includes its stopping distance); if it overshoots the new line by
  more than 10 mm, lower `NAV_BC_TURN_MM_S` by 10.

### Then the tests

| Test | What to do | Pass |
|---|---|---|
| S3-M02 | Follow the track for 30 s at 150 mm/s in daylight, room light and dim light. One video each. | no manual help in all three |
| S3-M03 | 20 passes over barcodes (5 each of A, B, C, D; a junction after each A and B). Count correct, missed, wrong, duplicates. | ≥ 19 correct, 0 wrong, 0 duplicates |
| S3-M04 | Junctions are not visible without a console, so use a turn: put an `*A*` before the sharpest curve, then one before the crossing. | before the curve: no stop and spin at the curve (it waits for the crossing); before the crossing: spins there, once |
| S3-M05 | Scope GP6 and a motor PWM pin: time from the sensor edge to the wheel change. Then suspend the line task: the car must stop. | ≤ 20 ms; stop within 50 ms + braking |
| S3-M07 | (S3-C03 done.) A, B, D and C through the + crossing and the T, 5 runs each. Film from above. | pivots within ±15 mm of the junction centre, stops with the new line under the leading sensor, takes the right branch every run; C goes straight across |

Note for S3-M07: a branch that goes off to **one side only** while the
line also continues straight (├ or ┤) puts only one sensor on black, so it
looks like a curve, not a junction. Test the junction shapes the course
really has, and write down what happens.

---

## Part 6: record the result

For each test, in `TEST_LOG.md` → *Results*, add a row:

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S3-M01 | `8e63e12` | n1sh0 | PASS | `logs/2026-10-21_part2.txt`, photo | 5 mm height |

- **Commit:** `git rev-parse --short HEAD` (commit before testing).
- **Logs** go in `logs/` (`YYYY-MM-DD_what.txt`), photos too; videos as
  unlisted YouTube links listed in `videos/README.md`.
- A FAIL stays in the table. Fix, re-run, add a new PASS row.

## If something goes wrong

| You see | Why | Do |
|---|---|---|
| nothing on the console | port opened after the Pico started | start `cat` first, then reset the Pico |
| `[line] Await Navigation` then `[nav] … search`, then `Safe Stop` | no front sensor saw black: line lost | tape under one front sensor; GP21 to restart |
| `[safety] fault from source 2, code 0x1` and `Fault / Inhibited` | both front sensors black too long: stuck sensor | only one sensor on black; GP21 after reset |
| `[b5] …` takes over, or `[safety] fault from source 4` | the ultrasonic sees something close, or is not connected | point it at open space, check Grove 4 |
| `bad` goes up, letter never read | uneven hand speed, sensor too high, or GP19 conflict | steadier slide; 5–10 mm height; see *Safety first* |
| `backwards` goes up | slid the wrong way | slide the other way |
| `[safety] warning … code 0x4` | A or B but no junction within 1 m of travel | make the junction sooner after the barcode |
