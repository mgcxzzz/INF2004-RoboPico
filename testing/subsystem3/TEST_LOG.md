# Subsystem 3 — test log

**Subsystem:** Line following and barcode navigation (Line and Barcode)  ·  **Student ID:** 2501910  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results. The
template's example test S3-U01 (consumer statistics) had no results and
is replaced by the tests below; INT-01 stays while the template demo runs.

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
| S3-U09 | host: statechart 6.3 on a simulated track: one event per barcode (FR-LINE-05), all four write-up commands A/B/C/D, sample Z read but no command, stray mark, backwards / unknown / torn barcodes, no distance, ignored event, junction, lost and found, pause, e-stop, stuck sensor + fault bit latch, lost edges, every telemetry frame | FR-LINE-02…05, FR-SYS-03, IF-10, IF-11, IF-17 | auto (PC) | `make -C testing/host` (`test_s3_task`) | 55 passed, 0 failed |
| S3-M01 | IR polarity and threshold at the final mounting height | A3 | manual | 1. `make` with `SYS_SIM_LINE` 0, flash. 2. Hold each sensor over white, then black; read `[line]` lines on the console. 3. Adjust each potentiometer until it switches cleanly. | each sensor reads 1 over black, 0 over white (else set `S3_IR_BLACK_LEVEL` 0) |
| S3-M02 | stays on the line 30 s, three lighting conditions | NFR-LINE-01 | manual | 1. Car on the course at 150 mm/s. 2. Daylight, room light, dim. 3. Video each run. | 30 s without manual correction in each |
| S3-M03 | 20 barcode traversals at 150 mm/s | NFR-LINE-02, FR-LINE-05 | manual | 1. 20 passes over full-size `*A*`, `*B*`, `*C*` and `*D*` (5 each; print them like Barcode_Sample.pdf, 3 mm / 9 mm). 2. Save the console log. 3. Count correct, missed, wrong and duplicate events separately. | ≥ 19 correct, 0 wrong, 0 duplicates; an unreadable one gives no command |
| S3-M04 | junction vs curve at target speed | FR-LINE-02, Week 6 feedback | manual | 1. Drive a curve, then a crossing line. 2. Read `[line]` / junction counter. | curve: no junction; crossing: exactly one |
| S3-M05 | steering latency and stale-target stop | IF-10, IF-19, Week 6 feedback | manual | 1. Scope GP6 and a motor PWM pin: time from the sensor edge to the wheel change. 2. Suspend the line task: the car must stop within 50 ms of the last target. | latency ≤ 20 ms (two cycles); stop ≤ 50 ms + braking |
| S3-M06 | side sensor passes over the barcode bars | hardware | manual | 1. Push the car past a barcode by hand. 2. Watch the barcode sensor bit (B) toggle. | B toggles for every bar; never sees the line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
