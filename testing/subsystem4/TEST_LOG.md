# Subsystem 4 — IMU, hump measurement and pickup stop

[RTOS IMU integration](../../app_program/imu_navigation/). Earlier hardware and bench results are in the [IMU archive](../evidence/buddy4/standalone/TEST_LOG.md).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S4-U20 | IMU task, transport and Reset | FR08, NFR01 | auto | `test_imu_service_rtos` | Calibration and reset use fresh observations and bounded I/O. |
| S4-U21 | Lift/overturn latch in release service | FR09, NFR02 | auto | `test_release_imu` | Qualified lift latches, invalid reset is refused and upright readiness is rechecked. |
| S4-U22 | Hump quality and odometry | FR08, NFR06 | auto | `test_hump_odo_grace` | Flat and hump cases, uncertain odometry and bounded recovery follow measurement policy. |
| S4-U40 | One-wheel hump and flat reference | FR08, NFR06 | hardware | Compare an uninterrupted flat run and a run with one wheel on a measured hump; read accepted peak and rejected count. | Flat run produces no accepted hump. The one-wheel reference produces an accepted peak for comparison. |
| S4-U41 | Lift and inversion stop test | FR09, NFR02 | hardware | Compare recorded orientation, Safety latch, PWM and Reset readiness during a physical lift. | A qualifying lift latches the stop and leaves usable evidence for motor response and Reset. |
| S4-U42 | Off-center right-wheel hump qualification | FR08 | hardware + host replay | Run the right wheel near the hump edge. Compare candidate height, quality flags, roll, freshness and pickup latch. | Record accepted or rejected peak, reason and timing without treating a candidate as accepted. |
| S4-U43 | Unexpected overturn flag during flat tracking | FR09 | hardware capture | Leave the robot on a flat straight. Record the first stop and any IMU latch before moving it. | Flat tracking does not qualify a pickup stop. A stopped capture preserves the order of motor-stop and IMU records. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S4-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Actual task/service checked calibration, reset epoch, lifetime counters and unlocked I/O. |
| 2026-11-21 | S4-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Lift, fresh/upright reset and blocked recalibration handoff passed. |
| 2026-11-21 | S4-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Flat/clean hump, uncertain travel, duration and turn/stop cases passed. |
| 2026-11-05 | S4-U40 | v0.1.5 | Team | PASS (functional) | [Saved log](../evidence/rtos_buddy1_4/step5_v0_1_5/README.md) | Zero accepted humps on flat and a 13 mm accepted peak on the nominal 10 mm reference. Accuracy needs repeated measured trials. |
| 2026-11-16 | S4-U41 | v0.2.4 | Team | FAIL (observed lift) | [Saved log](logs/2026-11-16_v0.2.4_lift.md) | 194 C9 frames showed no latch. Near a tilted sample the requested PWM was 105/115. |
| 2026-11-17 | S4-U42 | v0.2.5 | Team | FAIL (hump acceptance) | [Saved log](logs/2026-11-17_v0.2.5_hump_edge.md) | 12 mm candidate rejected as RANGE; logged turn rate stayed below the exclusion threshold. No pickup latch. [v0.2.6 fix](https://github.com/mgcxzzz/INF2004-RoboPico/commit/0ef813c) addressed the reproduced ramp-assist time-ordering race. |
| 2026-11-21 | S4-U40 | v0.4.2 | Team | PASS (run 4) | [Saved log](../system/logs/2026-11-21_run_4/README.md) | Accepted 12 mm and later 16 mm. Accuracy remains unmeasured for this circuit. |
| 2026-11-19 | S4-U43 | v0.2.11 | Team | FAIL (reported false trip) | [Trip event and operator observation](logs/2026-11-19_v0.2.11_flat_overturn.json) | An overturn latch was recorded during a reportedly flat run. [v0.2.12 reporting](../../docs/caster_assist_v0_2_12.md) preserves the initial drive-stop cause and subsequent IMU reporting. |
