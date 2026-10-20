# How to test Buddy 5 (Obstacle Scan and Recovery), step by step

This guide tells you exactly what to do for every Buddy 5 test in
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
- Use the **HC-SR04+** (3.3 V version) only. A 5 V HC-SR04 puts 5 V on GP17
  and can damage the Pico.
- The servo horn is only pushed on (no screw). If it slips, re-seat it with
  the sensor facing straight ahead while the servo is at centre (the task
  centres it at power-up).

## What you need

| Item | For |
|---|---|
| Car with Pico W on the Robo Pico; USB cable from the **Pico W's** socket to the computer | all |
| HC-SR04+ on **Grove 4** (TRIG GP16, ECHO GP17), 3V3 and GND | all |
| SG90 servo on **servo port 1** (GP12) | all |
| A flat box about **100 mm** wide and one about **200 mm** wide (shoe box, book) | U08, U09, U10 |
| Tape measure or ruler, phone for photos and video | evidence |

The IR line sensors are not needed: Buddy 3 is simulated in these tests.

## Settings used in this guide

All in `app_program/system/sys_config.h`. Change, rebuild, reflash.

| Switch | Parts 2–4 | Part 5 (car) |
|---|---|---|
| `SYS_SIM_CAR` | 1 | 0 (when Buddy 2's Motion Control is in) |
| `SYS_SIM_LINE` | **1** (simulated line: no line under the desk) | 0 (real Buddy 3) |
| `SYS_CONSOLE_SENSORS` | 1 for U08 only, else 0 | 0 |

`SYS_SIM_LINE` must be 1 here. With 0 the real Buddy 3 task runs, finds no
line on the desk, and stops the run.

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
cat /dev/cu.usbmodem1101 | tee ../testing/subsystem5/logs/2026-10-21_bench.txt
```

Start `cat` first, then press the Pico's reset (or unplug and replug USB).
A run starts by itself 2 s after power-up. **GP20** = emergency stop,
**GP21** = reset and start a new run.

---

## Part 1: PC tests, no Pico (S5-U12, S5-U16)

```sh
make -C testing/host 2>&1 | tee testing/subsystem5/logs/2026-10-21_host.txt
```

Pass: `test_b5_pack` `43 passed, 0 failed` (S5-U16), `test_b5_profile`
`95 passed, 0 failed` and scenarios 1–4 `PASS` (S5-U12). Then
`make -C testing/host clean`.

## Part 2: quick hardware check (optional, before anything else)

The standalone servo and ultrasonic test you already have
(`servo_us_test_55.uf2`, not part of the repo) checks the wiring on its
own: flash it, open the console, and follow its menu (self-test, ranging,
left/right finder). Then flash the repo firmware again.

## Part 2b: calibrate on the bench (S5-C01, S5-C05)

Record each value in `TEST_LOG.md` → *Calibration record*, and commit the
config change with it. S5-C02 … C04 come later, with the tests they use.

**S5-C01: servo centre, span and direction.**

1. Make a paper guide: a dot for the servo's turning axis, a line straight
   ahead from it, and two lines at ±55°: each goes through a point
   **100 mm ahead and 143 mm to the side** of the dot. Put the car on the
   paper with the servo axis over the dot, square to the forward line.
2. **Centre:** with the standalone tool (`servo_us_test_55.uf2`, its
   centre finder) find the pulse C (µs) at which the sensor faces straight
   along the forward line. In `app_program/subsystem5/b5_config.h` shift
   both `B5_SERVO_MIN_US` and `B5_SERVO_MAX_US` by (C − 1450). One degree
   is about 10.6 µs.
3. **Direction:** S5-U07 below (`B5_SERVO_REVERSED`).
4. **Span:** with the repo firmware, start a scan (box at 250 mm) and film
   from above: at the two ends of the sweep the sensor must point along
   the ±55° lines. If it only reaches ±θ°, make the span
   (`MAX − MIN`) larger by 55/θ, keeping the centre C:
   `MIN = C − span/2`, `MAX = C + span/2`.

Target: 0° straight ahead and ±55° within ±3°.

**S5-C05: car size.** Measure the car's width (wheels and brackets
included) and length (front to back). Set `B5_CAR_WIDTH_MM` and
`B5_CAR_LENGTH_MM` in `b5_config.h`.

## Part 3: self-test image (S5-U01 … S5-U06, S5-U13 … S5-U15)

1. Ultrasonic pointing at **open space** (nothing within 60 cm).
2. `make TEST=1 -j8`, flash, start logging, reset.
3. Look for `[TEST] S5-U01 PASS` … `[TEST] S5-U15 PASS`.

Pass: every S5 line PASS. `S5-U05 SKIP` means no run had started: check
`SYS_SIM_LINE` is 1 and run it again.

## Part 4: bench tests (normal image, `make -j8`)

After power-up you should see:

```
[bench] starting a run
[nav] Running, motion owner: line
[b5]  Guard  (range … mm)
```

### S5-U07: servo direction

1. Watch the sensor at power-up: it must face **straight ahead** (±5°).
   Photo from above.
2. Hold the box at about 250 mm. During `Coarse Sweep` the sensor steps
   from −55° to +55° (−55, −27.5, 0, +27.5, +55), so it must turn to the
   car's **LEFT first**, then sweep across to the **right**.
3. If it goes right first, set `B5_SERVO_REVERSED` to 1 in
   `app_program/subsystem5/b5_config.h`, rebuild, check again.

### S5-U08: distance accuracy

1. Set `SYS_CONSOLE_SENSORS` to 1, rebuild, flash. The console prints
   `[b5]  range … mm` about twice a second.
2. Put the box face square to the sensor at **500 mm**, measured from the
   sensor's front. Note 10 readings.
3. Repeat at 300 mm and 100 mm. (Below 500 mm the car "slows"; below
   300 mm it stops and scans. That is fine: the range lines continue.)
4. Set `SYS_CONSOLE_SENSORS` back to 0.

Pass: the average at each distance within ±10 mm of the tape measure.

**S5-C03 (calibration, from the same readings).** If every reading is off
by about the same *percentage* (e.g. 2 % long at all three distances), the
speed of sound is off: the code uses 343 m/s (20 °C); it is about 346 m/s
at 25 °C and 349 m/s at 30 °C. Change `SOUND_NUM` in
`app_program/subsystem5/drv_ultrasonic.c` to match the room. Then measure
from the sensor's face to the very front of the car: that is the offset.
`B5_D_STOP_MM` is measured from the sensor, so the gap left in front of
the car is `B5_D_STOP_MM` minus the offset. Record the offset.

### S5-U09: obstacle width

1. Put the **100 mm** box square in front at about **280 mm** (just inside
   the 300 mm stop distance), nothing else
   within 60 cm. Then press **GP21** to start a fresh run.
2. Hold still until the console prints:
   ```
   [b5]  Planning  (range … mm)
   [b5]  obstacle 103 mm wide at 298 mm, go right
   ```
3. Repeat 5 times, then with the **200 mm** box.

Pass: width within 10 % (90–110 mm; 180–220 mm).

**S5-C04 (calibration): beam half-angle.** If the widths are wrong by
about the same amount every time, correct `B5_ALPHA_DD` in `b5_config.h`.
With Δ = measured − true width (mm) at distance d (mm):
new `B5_ALPHA_DD` = old + atan(Δ / (2·d)) in tenths of a degree.
Example: the 100 mm box reads 124 mm at 300 mm → Δ = 24 →
atan(24/600) = 2.3° → add 23 (75 → 98). Too narrow: Δ is negative, so it
gets smaller. Repeat S5-U09 with both boxes.

**S5-C02 (calibration): servo settle time.** Repeat S5-U09 once with
`B5_SERVO_SETTLE_MS_30` doubled (40 → 80). If the widths change by more
than 5 %, the sensor was still moving when it measured: keep 80.

### S5-U10: full bypass on the bench

1. Run going, `[b5] Guard`. Bring the box slowly to about 250 mm and hold
   it still.
2. You should see, in order:
   ```
   [nav] Running, motion owner: line (slow)        below 500 mm
   [b5]  Confirm  ...
   [b5]  Halting  ...
   [nav] Running, motion owner: obstacle
   [b5]  Coarse Sweep  ...
   [b5]  Edge Refinement  ...
   [b5]  Planning  ...
   [b5]  obstacle … mm wide at … mm, go right
   [b5]  Bypassing  ...
     [motion] turn 45 deg
     [motion] drive … mm
   ```
3. **As soon as `Bypassing` appears, take the box away.** The simulated
   car is driving round it, but the real sensor would still see it and
   start again.
4. Then `[b5] Recovering`, the line found, `[nav] Running, motion owner:
   line` and `[b5] Guard`.
5. Note the time from `Confirm` to `Guard` (the console has no clock: use
   the video). Video the whole run with the console in shot.

Pass: every state in that order, back to `Guard`, no `Fault`. Write the
time from confirmation to line regained in the notes (NFR-USO-05 asks for
3 s; the code's own estimate is about 11 s, a known issue to report).

### S5-U11: emergency stop during a bypass

1. Start S5-U10 again. While `Bypassing`, press **GP20**.
2. The console prints:
   ```
   [button] EMERGENCY STOP
   [safety] Emergency Stop, motor drive inhibited
   [safety] drive off … ms after the stop (worst … ms, limit 100)
   [nav] Emergency Stop, ...
   ```
3. Press **GP21** to reset and start again.

Pass: drive off under 100 ms, `Emergency Stop` state. (On the bench the
simulated motors stop at once; repeat on the real car in Part 5.)

---

## Part 5: on the car (needs Buddy 2's Motion Control)

Set `SYS_SIM_CAR` to 0 and `SYS_SIM_LINE` to 0 once Buddy 2's and Buddy
3's code run on the car. Power from the **power bank only** (Robo Pico
power input; Pico W USB unplugged), so there is no console: until WiFi
telemetry (Buddy 1) works, the evidence is **video**.

**Ask Buddy 2 first** for calibrated distance and turn angle (the bypass
legs are driven by Motion Control) and the car's braking distance.

| Test | What to do | Pass |
|---|---|---|
| S5-C06 | Calibration: drive at a box on the line at line speed, 10 runs; measure the gap left in front of the car. Too small: raise `B5_D_STOP_MM` (and `B5_D_SLOW_MM` by the same). | gap at least 100 mm every run |
| S5-U10 | Box on the line about 1 m ahead; start the run (GP21). Film from above. | stops, scans, drives round without touching it, rejoins the line |
| S5-U11 | Press GP20 during the bypass. Film the wheels. | wheels stop at once (count video frames: under 100 ms) |
| S5-U09 | Repeat the 100 mm and 200 mm boxes on the track. | same as on the bench |


---

## Part 6: record the result

For each test, in `TEST_LOG.md` → *Results*, add a row:

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S5-U08 | `8e63e12` | n1sh0 | PASS | `logs/2026-10-21_u08.txt` | 500 mm: avg 503 |

- **Commit:** `git rev-parse --short HEAD` (commit before testing).
- **Logs** go in `logs/` (`YYYY-MM-DD_what.txt`), photos too; videos as
  unlisted YouTube links listed in `videos/README.md`.
- A FAIL stays in the table. Fix, re-run, add a new PASS row.

## If something goes wrong

| You see | Why | Do |
|---|---|---|
| nothing on the console | port opened after the Pico started | start `cat` first, then reset the Pico |
| `[b5] Fault` and `[safety] fault from source 4` right after the start | no echo: sensor not connected or wired wrong | check Grove 4, 3V3, GND; Part 2 tool |
| `[nav] … search`, then `Safe Stop`, no obstacle involved | `SYS_SIM_LINE` is 0: the real Buddy 3 finds no line | set it to 1 |
| the scan keeps restarting | the box is still in front during `Bypassing` | take it away when `Bypassing` appears |
| `go nowhere`, `[b5] Blocked` | something else within reach on both sides | clear the desk around the car |
| the sensor hits the casing at the sweep ends | horn seated off-centre | re-seat at centre; the sweep is ±55° |
