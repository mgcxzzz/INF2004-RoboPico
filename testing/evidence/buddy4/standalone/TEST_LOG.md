# Subsystem 4 — test log

**Subsystem:** IMU-Based Motion and Terrain Monitoring (IMU Terrain)  ·  **Student ID:** 2503753  ·  **Code:** `app_program/subsystem4/`

How to run each test: `HOW_TO_TEST.md` (same folder). One log file per
test ID in `logs/`, named `YYYY-MM-DD_<TEST-ID>_<what>.txt`.

## Test cases

Define a test here before running it. Test IDs never change.

### Calibration (bench tool `imu_bench.uf2`, keys in brackets)

| ID | What is set or checked | Method | Expected result |
|---|---|---|---|
| S4-C00 | GY-511 wiring and chip on I2C1 (GP2/GP3) [`s`] | manual | 0x19 and 0x1e answer, accelerometer set up OK |
| S4-C01 | mounting: FORWARD and UP axes and signs [`o`] | manual | four `#define` lines, copied into `b4_config.h` |
| S4-C02 | calibration window, 2.0 s still [`c`] | manual | PASS 3 of 3 still; FAIL when nudged |
| S4-C03 | noise, still and pushed [`n`] | manual | hump threshold (4.0 deg) >= 5 x pitch noise |
| S4-C04 | shock threshold [`k`] | manual | `B4_SHOCK_MG` above a hump jolt, below a wall bump |
| S4-C05 | wheelbase, then block test 20 / 40 mm [`p`] | manual | front height within max(5 mm, 10 %) of each block |

### Unit

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-H01 | statechart 6.4, hump maths (200 random humps), shock, motion classes, data health, telemetry frame | FR-IMU-01..03, NFR-IMU-01..03, Tribunal 1 | auto (PC) | `make -C testing/subsystem4/host` | `69 passed, 0 failed` |
| S4-U01 | GY-511 answers and calibrates | statechart 6.4 | auto | test image, car still | `[TEST] S4-U01 PASS` |
| S4-U02 | 250 periods of 20 ms in 5 s, none missed | NFR-IMU-01 | auto | test image | `[TEST] S4-U02 PASS` |
| S4-U03 | pitch maths (asin table) | FR-IMU-02 | auto | test image | `[TEST] S4-U03 PASS` |
| S4-U04 | 20 mm hump through the statechart code on the Pico | NFR-IMU-02 | auto | test image | `[TEST] S4-U04 PASS` |
| S4-U05 | shock flagged in the period it happened | NFR-IMU-03 | auto | test image | `[TEST] S4-U05 PASS` |
| S4-U06 | IMU telemetry frame round trip (min, max, negative) | IF-22, Tribunal 1 | auto | test image | `[TEST] S4-U06 PASS` |
| S4-U07 | 60 s at 50 Hz: late periods, FIFO overruns, sensor rate | NFR-IMU-01, A4 | manual | bench `f` | 0 late, 0 overruns, ~400 Hz |
| S4-U08 | shock detect-to-notify time, 10 taps | NFR-IMU-03 | manual | bench `k` | 10 of 10 <= 50 ms |
| S4-U09 | hump pushed by hand between marks, 5 runs | FR-IMU-01/02 | manual | bench `h` | hump detected; height near the ruler (rough) |
| S4-U10 | hump on the car with Buddy 2's distance, 10 runs | NFR-IMU-02 | manual | normal image, needs Buddy 2 | 10 of 10 within max(5 mm, 10 %), no false hump |
| S4-U11 | statechart on the hardware: calibrate, track, hump (no odometry), impact, e-stop, fault, reset | statechart 6.4, IF-20, IF-23 | manual | normal image, `HOW_TO_TEST.md` S4-U11 | console shows each step as listed |

*Method:* **auto** = `TEST_CHECK` in code (or the host test program), result
printed on the console; **manual** = a person runs the bench tool or watches
the console.

## Calibration record

| Date | Value | Result | Evidence |
|---|---|---|---|
| | `B4_AXIS_FWD / SIGN_FWD / AXIS_UP / SIGN_UP` (S4-C01) | | |
| | `B4_WHEELBASE_MM` (S4-C05) | | |
| | `B4_SHOCK_MG` (S4-C04) | | |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-20 | S4-H01 | the commit adding `subsystem4/` | n1sh0 | PASS | `logs/2026-10-20_S4-H01_host_tests.txt` | 69 passed; U-12: 200 random humps 10-80 mm, all within max(5 mm, 10 %), worst 4 mm (simulation, not yet real humps) |
