# Subsystem 5 — test log

**Subsystem:** Adaptive ultrasonic scanning and obstacle profiling  ·  **Student ID:** 2500339  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

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
| S5-U13 | binary telemetry frame round trip (negative servo angle, range clamped to 4095) | IF-13, minimum-bit change | auto | `subsystem5_test.c` | `[TEST] S5-U13 PASS` |
| S5-U14 | scan sample packs to 4 bytes; 4396 mm clamps to 4095 (never wraps to 300) | minimum-bit change | auto | `subsystem5_test.c` | `[TEST] S5-U14 PASS` |
| S5-U15 | 16-bit wrap difference on Buddy 2's distance (−32700 → 32700 = 136; 100 → −150) | IF-16, minimum-bit change | auto | `subsystem5_test.c` | `[TEST] S5-U15 PASS` |
| S5-U16 | host packing tests: record sizes, clamping, sign extension, echo word, frame layout; statechart checks fault-mask bits and decodes every frame | minimum-bit change, shared fault mask | auto (PC) | `make -C testing/host` | `test_b5_pack` 36 passed; scenarios 1–4 PASS with 0 bad frames |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
