# Subsystem 5 — test log

**Subsystem:** Adaptive ultrasonic scanning and obstacle profiling  ·  **Student ID:** 2500339  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results, and
**`HOW_TO_TEST.md`** (this folder) for step-by-step instructions for every
test below.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Q15 trig and median-of-three helpers | FR-USO-03 | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | 100 mm obstacle from a recorded scan measured within 10 % | NFR-USO-02 | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS`, width 90–110 mm |
| S5-U03 | open on both sides: a bypass side is chosen | FR-USO-04 | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | obstacle task created and running | – | auto | `subsystem5_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | HC-SR04+ answers pings (needs a run started; SKIP otherwise) | – | auto | `subsystem5_test.c`, bench sim autostarts | `[TEST] S5-U05 PASS` |
| S5-U06 | pings at least 60 ms apart | NFR-USO-06 | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | servo at 0 (signed ±90° convention) faces straight ahead; + angle turns right | FR-USO-03 | manual | 1. Power up, task centres servo. 2. Photo from above. 3. Watch first scan sweep direction | sensor square to car front ±5°; sweep goes right for + angles |
| S5-U08 | distance accuracy at 100, 300, 500 mm | – | manual | 1. Flat box at measured distance. 2. Read `range` on console status | within ±10 mm |
| S5-U09 | obstacle width on the car: 100 mm and 200 mm boxes at 300 mm | NFR-USO-02 | manual | 1. Box in front, car stopped. 2. Read profile on console | within 10 % |
| S5-U10 | full bypass on the bench (simulated car) | FR-USO-04, FR-USO-06, NFR-USO-05 | manual | 1. `SYS_SIM_CAR` = 1, flash. 2. Hold box < 300 mm. 3. Save console + video | Guard → Stopping → Scanning → Bypassing → Recovering → Guard; line regained < 3 s after confirmation |
| S5-U11 | emergency stop time while S5 owns motion | NFR-SYS-04 | manual | 1. During a bypass press GP20. 2. Read stop time on console | drive off < 100 ms, state ESTOP |
| S5-U12 | host tests: profile + 4 statechart scenarios | NFR-USO-02, FR-USO-04 | auto (PC) | `make -C testing/host` | 95 passed; scenarios 1–4 PASS |
| S5-U13 | binary telemetry frame round trip in the team schema (common header + payload; negative servo angle, range clamped to 4095) | IF-13, minimum-bit change | auto | `subsystem5_test.c` | `[TEST] S5-U13 PASS` |
| S5-U14 | scan sample packs to 4 bytes; 4396 mm clamps to 4095 (never wraps to 300) | minimum-bit change | auto | `subsystem5_test.c` | `[TEST] S5-U14 PASS` |
| S5-U15 | 16-bit wrap difference on Buddy 2's distance (−32700 → 32700 = 136; 100 → −150) | IF-16, minimum-bit change | auto | `subsystem5_test.c` | `[TEST] S5-U15 PASS` |
| S5-U16 | host packing tests: record sizes, clamping, sign extension, echo word, frame layout; statechart checks fault-mask bits and decodes every frame | minimum-bit change, shared fault mask | auto (PC) | `make -C testing/host` | `test_b5_pack` 43 passed (incl. common header: wrong version, source or length rejected); scenarios 1–4 PASS with 0 bad frames |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Calibration

Values that must be measured or tuned on the real hardware before the
tests above mean anything. Do them in this order; steps are in
`HOW_TO_TEST.md`. Record every value you set in the *Calibration record*
below (and commit the config change with it).

| ID | What is calibrated | Value(s) set | Needs | How | Target |
|---|---|---|---|---|---|
| S5-C01 | servo centre, span and direction | `B5_SERVO_MIN_US`, `B5_SERVO_MAX_US`, `B5_SERVO_REVERSED` (`b5_config.h`) | bench | paper guide with lines straight ahead and at ±55°; command 0 and ±55° | 0° faces straight ahead, ±55° within ±3°, + is right |
| S5-C02 | servo speed and settle time | `B5_SERVO_SPEED_DD_S`, `B5_SERVO_SETTLE_MS` | bench | S5-U09 with 40 ms and with 80 ms | widths agree within 5 % (else keep the longer) |
| S5-C03 | range scale and sensor position | `SOUND_NUM` (`drv_ultrasonic.c`) only if needed; sensor-to-front offset | bench | S5-U08 readings vs tape measure; measure sensor face to the car's front | readings within ±10 mm; offset recorded |
| S5-C04 | beam half-angle | `B5_ALPHA_DD` | bench | S5-U09 widths at a known distance | 100 mm and 200 mm boxes within 10 % |
| S5-C05 | car size | `B5_CAR_WIDTH_MM`, `B5_CAR_LENGTH_MM` | bench | measure, wheels and brackets included | values set |
| S5-C06 | stop and slow distances | `B5_D_STOP_MM`, `B5_D_SLOW_MM` | car (Buddy 2) | drive at a box at line speed, 10 runs; measure the gap left | gap at least 100 mm every run, box fully seen by the scan |

Needs from Buddy 2 first: distance and turn angle calibrated (the bypass
legs are driven by Motion Control), and the car's real braking distance.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-20 | S5-U08 | – (standalone `servo_us_test_55`) | n1sh0 | PASS | `logs/2026-10-20_S5-U08_ranging.txt` | 100 mm → 106, 300 → 303, 500 → 498 (spread 0 / 4 / 9 mm). Tape from the front of the cylinders, flat box face-on. 1 m: two groups (≈ 900 and ≈ 990 mm, box smaller than the beam, table reflection), not needed. Before this run TRIG / ECHO were swapped at the sensor (NO ECHO); fixed. |
| 2026-10-20 | S5-U07 | – (standalone `servo_us_test_55`) | n1sh0 | PASS | `logs/2026-10-20_S5-U07_servo_direction.txt` | left/right finder: box seen at +5…+45° (≈ 245 mm) on the right; SHORT pulses point RIGHT, so `B5_SERVO_REVERSED` stays 0. Centre (1450 µs) faces straight ahead by eye. |
| 2026-10-20 | S5-U12 | `6b6edea` + S5-C05 config change | n1sh0 | PASS | `logs/2026-10-20_S5-U12_host_profile.txt` | test_b5_profile 95 passed (worst width error 6 %); statechart scenarios 1–4 PASS. Run in the Mac's Linux shell (gcc 11.4). |
| 2026-10-20 | S5-U16 | `6b6edea` + S5-C05 config change | n1sh0 | PASS | `logs/2026-10-20_S5-U16_host_pack.txt` | test_b5_pack 43 passed; scenarios 1–4 with 0 bad frames and the expected fault-mask bits (in the S5-U12 log). |
| 2026-10-20 | S5-U09 | – (standalone `servo_us_test_55`, box width scan) | n1sh0 | FAIL | `logs/2026-10-20_S5-U09_box_width.txt` | 230 mm box: 231–237 mm (≤ 3 %, PASS); 100 mm box: 299–319 mm (≈ +200 %, FAIL). A flat face echoes back wherever it is within the ±25–35° beam, so narrow obstacles read too wide. Next: tubes to narrow the beam, then re-test. |

### Calibration record

Add a row for every value you measure or change. Never edit old rows.

| Date | ID | Commit | Who | Measured | Value set (file) | Evidence |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-C04 | `abc1234` | | 100 mm box read 124 mm at 300 mm | `B5_ALPHA_DD` 75 → 98 (`b5_config.h`) | `logs/…` |
| 2026-10-20 | S5-C03 | – (standalone tool) | n1sh0 | errors +6 / +3 / −2 mm at 100 / 300 / 500 mm: no common percentage; sensor front to car front 45 mm | none: `SOUND_NUM` stays 343; gap in front of the car at the 300 mm stop = 255 mm | `logs/2026-10-20_S5-C03_range_scale.txt` |
| 2026-10-20 | S5-C05 | – (measured) | n1sh0 | width 155 mm, length 190 mm (generous: widest and longest points) | `B5_CAR_WIDTH_MM` 150 → 155, `B5_CAR_LENGTH_MM` 200 → 190 (`b5_config.h`); host tests still pass | `logs/2026-10-20_S5-C05_car_size.txt` |
| 2026-10-20 | S5-C01 | – (standalone tool) | n1sh0 | centre 1450 µs faces 0° by eye (no trim); ±55° ends (870 / 2030 µs) symmetric about 0° and clear of the casing, re-checked in a second session | none: `B5_SERVO_MIN_US` 500, `B5_SERVO_MAX_US` 2400, `B5_SERVO_REVERSED` 0 stay (`b5_config.h`); end angles judged by eye, checked indirectly by S5-U09 | `logs/2026-10-20_S5-C01_servo_centre.txt` |
| 2026-10-20 | S5-C01 | – (standalone tool) | n1sh0 | box scans at 1450 µs put the box 3–4° left; trial at 1490 µs: 230 mm box −27°…+33° (now 3° right, edges moved unevenly), and 1490 µs looked about 4° left by eye. Scans vary ±4° with box placement, so the eye check decides | none: `B5_SERVO_MIN_US` 500, `B5_SERVO_MAX_US` 2400 stay (540 / 2440 tried and reverted) | `logs/2026-10-20_S5-C01_servo_centre.txt` |
| 2026-10-20 | S5-C04 | – (standalone tool) | n1sh0 | implied half-angle 20–21° (100 mm box) but 0–0.6° (230 mm box) | `B5_ALPHA_DD` 75 → 0: never makes an obstacle look narrower | `logs/2026-10-20_S5-C04_beam_angle.txt` |
| 2026-10-20 | S5-C02 | – (standalone tool) | n1sh0 | the original full-speed sweep snapped the bracket into the wiring; after re-routing the cables, 5 end-to-end sweeps at 600°/s (tool key `f`) were clear | `B5_SERVO_SPEED_DD_S` added: 600 (60°/s), now 6000 (600°/s, full speed); `B5_SERVO_SETTLE_MS` 40; settle-time check (40 vs 80 ms with the firmware) still to do | – |
