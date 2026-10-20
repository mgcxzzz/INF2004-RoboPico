# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | S3 steers through the coordinator only while granted; a barcode turn hands control over and back with no conflicting command | IF-10, IF-11, FR-SYS-02 | manual | 1. Car on the course with S2 in. 2. Run over a barcode. 3. Save the console log. | wheel speeds stop at the barcode event, one turn, line following resumes; no wheel-speed command during the turn |
| INT-04 | stale-target stop: Motion Control stops when S3's targets stop | IF-19 | manual | 1. Running on the line. 2. Pause the line task (debugger) or unplug GP6/GP7 and force LOST. | wheels stop within 50 ms of the last target |
| INT-05 | barcode bars measured by distance with Motion Control's estimate | IF-18, NFR-LINE-02 | manual | 1. Drive over `*A*` at 100, 150, 200 mm/s. 2. Read `[line]` lines (measured by distance = 1). | A decoded at every speed |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
