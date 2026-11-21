# Subsystem 4 — IMU, hump measurement and pickup stop

[RTOS IMU integration](../../app_program/imu_navigation/). Earlier hardware and bench results are in the [IMU archive](../evidence/buddy4/standalone/TEST_LOG.md).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S4-U20 | IMU task, transport and Reset | FR08, NFR01 | auto | `test_imu_service_rtos` | Calibration and reset use fresh observations and bounded I/O. |
| S4-U21 | Lift/overturn latch in release service | FR09, NFR02 | auto | `test_release_imu` | Qualified lift latches, invalid reset is refused and upright readiness is rechecked. |
| S4-U22 | Hump quality and odometry | FR08, NFR06 | auto | `test_hump_odo_grace` | Flat and hump cases, uncertain odometry and bounded recovery follow measurement policy. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S4-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Actual task/service checked calibration, reset epoch, lifetime counters and unlocked I/O. |
| 2026-11-21 | S4-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Lift, fresh/upright reset and blocked recalibration handoff passed. |
| 2026-11-21 | S4-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Flat/clean hump, uncertain travel, duration and turn/stop cases passed. |
