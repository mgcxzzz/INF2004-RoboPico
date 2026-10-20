# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | e.g. device reports a reading over WiFi every 10 s | REQ-? | manual | 1. Power on. 2. Watch the dashboard for 2 minutes. | a reading appears every 10 s ± 1 s |
| SYS-02 | | | manual | 1. … 2. … | |
| SYS-U01 | Navigation Coordinator (host): states, ownership, bypass, barcode, line search, timeouts; minimum bits: 8-byte inbox message, clamped step values, ids above 15 bits ignored, dropped count saturating at 255, every telemetry frame decoded in the team schema with no sequence gaps | FR-SYS-02, IF-08 to IF-13, minimum-bit change | auto (PC) | `make -C testing/host` (`test_nav`) | 63 passed, 0 failed |
| SYS-U02 | Safety Supervisor (host): e-stop timing, latching, reset order, health checks; minimum bits: stop time saturating at 1023 ms, 8-bit counters wrapping, shared fault mask in every telemetry frame (team schema, no sequence gaps) | FR-SYS-03, NFR-SYS-04, IF-13 to IF-15, IF-17, minimum-bit change | auto (PC) | `make -C testing/host` (`test_safety`) | 36 passed, 0 failed |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
