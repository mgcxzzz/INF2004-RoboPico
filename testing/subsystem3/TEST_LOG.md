# Subsystem 3 — test log

**Subsystem:** Line following and barcode navigation (Line and Barcode)  ·  **Student ID:** 2501910  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results, and
**`HOW_TO_TEST.md`** (this folder) for step-by-step instructions for every
test below. The template's
example test S3-U01 (consumer statistics) had no results and is replaced
by the tests below; INT-01 stays while the template demo runs.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 table: 44 characters, 3 wide elements each | FR-LINE-03 | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | the course print `*A*` (3 mm / 9 mm) decodes to A | FR-LINE-03, FR-LINE-04 | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | the same barcode read backwards gives no command | NFR-LINE-02, Tribunal 1 test plan | auto | `subsystem3_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | half-size `*Z*` (1.5 mm narrow) decodes to Z | FR-LINE-03 | auto | `subsystem3_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | line logic: steer towards the sensor on the line; junction; line lost | FR-LINE-01, FR-LINE-02 | auto | `subsystem3_test.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | Line and Barcode task running, sensors readable (needs `SYS_SIM_LINE` 0) | – | auto | `subsystem3_test.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | telemetry frame round trip (team schema, source 3) | IF-13, minimum-bit change | auto | `subsystem3_test.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | host: all four course barcodes from the PDF, every character at 3:1 and 2:1, rejects, 2000 noisy reads never misread, the reader at steady / varying speed and stopping on a space, line logic, frame | FR-LINE-01…04, NFR-LINE-02, minimum-bit change | auto (PC) | `make -C testing/host` (`test_s3`) | 58 passed, 0 failed |
| S3-U09 | host: statechart 6.3 on a simulated track: LEFT / RIGHT posted with the axle over the next junction (creep straight and slow), only the leading sensor ends the spin, U-turn right after the barcode, STRAIGHT keeps following, no junction = dropped with a warning, line lost = command dropped, one event per barcode (FR-LINE-05), all four write-up commands A/B/C/D, sample Z read but no command, stray mark, backwards / unknown / torn barcodes, no distance, ignored event, junction, lost and found, pause, e-stop, stuck sensor + fault bit latch, lost edges, every telemetry frame | FR-LINE-02…05, FR-SYS-03, IF-10, IF-11, IF-17 | auto (PC) | `make -C testing/host` (`test_s3_task`) | 74 passed, 0 failed |
| S3-M01 | IR polarity and threshold at the final mounting height | A3 | manual | `HOW_TO_TEST.md` Part 2: `SYS_CONSOLE_SENSORS` 1, hold each sensor over white then black, read the `[line] sensors` lines, adjust the potentiometers. | each sensor reads 1 over black, 0 over white (else set `S3_IR_BLACK_LEVEL` 0) |
| S3-M02 | stays on the line 30 s, three lighting conditions | NFR-LINE-01 | manual | 1. Car on the course at 150 mm/s. 2. Daylight, room light, dim. 3. Video each run. | 30 s without manual correction in each |
| S3-M03 | 20 barcode traversals at 150 mm/s | NFR-LINE-02, FR-LINE-05 | manual | 1. 20 passes over full-size `*A*`, `*B*`, `*C*` and `*D*` (5 each; print them like Barcode_Sample.pdf, 3 mm / 9 mm). Put a junction after each A and B (they wait for one). 2. Save the console log. 3. Count correct, missed, wrong and duplicate events separately. | ≥ 19 correct, 0 wrong, 0 duplicates; an unreadable one gives no command |
| S3-M04 | junction vs curve at target speed | FR-LINE-02, Week 6 feedback | manual | 1. `*A*` before the sharpest curve, drive it. 2. `*A*` before a crossing, drive it. (Junctions show only on the console; a pending turn makes them visible.) | curve: no spin there; crossing: spins there, once |
| S3-M05 | steering latency and stale-target stop | IF-10, IF-19, Week 6 feedback | manual | 1. Scope GP6 and a motor PWM pin: time from the sensor edge to the wheel change. 2. Suspend the line task: the car must stop within 50 ms of the last target. | latency ≤ 20 ms (two cycles); stop ≤ 50 ms + braking |
| S3-M06 | side sensor passes over the barcode bars | hardware | manual | `HOW_TO_TEST.md` 4c: tape line, printed `*C*` 20 mm beside it, push the car by hand along the line, 5 passes. | `barcode 'C' … result 0` 5 out of 5 |
| S3-M07 | barcode turn by line search, no fixed angle | FR-LINE-04, IF-10 | manual | 1. Measure front sensors to wheel axle; set `S3_PIVOT_OFFSET_MM`. 2. Tape a + crossing and a T with a barcode before each. 3. Run A (left), B (right), D (U-turn), C (straight) through each; 5 runs each. 4. Video; note where the car pivots and where it stops on the new line. | pivots within ±15 mm of the junction centre; stops with the new line under the leading sensor; follows the correct branch every run; C goes straight across |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Calibration

Values that must be measured or tuned on the real hardware before the
tests above mean anything. Do them in this order; steps are in
`HOW_TO_TEST.md`. Record every value you set in the *Calibration record*
below (and commit the config change with it).

| ID | What is calibrated | Value(s) set | Needs | How | Target |
|---|---|---|---|---|---|
| S3-C01 | front sensor spacing and height, threshold | mechanical; potentiometers | bench | measure the gap between the two front sensors (centre to centre) and their height above the floor; set each potentiometer (S3-M01) | gap 25–40 mm (wider than the 18 mm line); clean switching at that height |
| S3-C02 | barcode sensor position | mechanical | bench | measure from the line centre to the barcode sensor, and its height | 40–50 mm from the line centre (at least 29 mm); same height as the front sensors |
| S3-C03 | pivot point | `S3_PIVOT_OFFSET_MM` (`s3_config.h`) | bench | ruler: from the front sensors to the wheel axle | value set to the measurement |
| S3-C04 | line-following speeds | `S3_BASE_MM_S`, `S3_TURN_*`, `S3_SOFT_*` | car (Buddy 2) | test track: straight, tightest curve | smooth on the straight, never loses the tightest curve at the chosen base speed |
| S3-C05 | line-lost distance | `S3_LOST_MM` | car (Buddy 2) | 2 m straight and the tightest curve: no false LOST; lift the car off the line: LOST comes | no false LOST in 5 runs; a real loss is reported |
| S3-C06 | junction length | `S3_JUNCTION_MM` | car (Buddy 2) | S3-M04: an `*A*` before the sharpest curve and one before a crossing | spins at the crossing only, never at the curve |
| S3-C07 | barcode-turn speeds | `S3_CREEP_MM_S`, `NAV_BC_TURN_MM_S` (`nav_config.h`) | car (Buddy 2) | film S3-M07 from above | pivot within ±15 mm of the junction centre; stop overshoot under 10 mm |

Needs from Buddy 2 first: distance in mm calibrated (encoder mm per tick;
barcode bars are measured with it) and turns that spin on the spot (the
pivot lands on the junction only if the car turns about its axle centre).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

### Calibration record

Add a row for every value you measure or change. Never edit old rows.

| Date | ID | Commit | Who | Measured | Value set (file) | Evidence |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-C03 | `abc1234` | | sensors → axle 68 mm | `S3_PIVOT_OFFSET_MM` 68 (`s3_config.h`) | photo |
