# Subsystem 3 — Line following and barcode routing

[RTOS line and barcode integration](../../app_program/line_navigation/).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S3-U20 | Decode and mapping | FR06 | auto | `test_rtos_barcode` | Recorded and synthetic labels decode; unmapped labels queue no action. |
| S3-U21 | Junction stripe qualification | FR07 | auto | `test_junction_sensor` | Width, white margins, rejected stripes and C7 encoding follow the junction policy. |
| S3-U22 | Stationary search | FR05 | auto | `test_line_stationary_search` | Search stays bounded and resumes on credible tape evidence. |
| S3-U40 | Recorded barcode decode and application | FR06 | hardware capture | Review peak decoder outputs from repeated sequences of printed labels on the mat. | Valid reads identify the letter and apply the configured action. Incomplete readings apply no action. |
| S3-U41 | Outgoing stripe acquisition after U-turn | FR07 | hardware + video | Compare status, encoder rotation and ADC on the first normal-follow rows with the robot's physical path. | Completion corresponds to actual reacquisition and tracking of the outgoing tape. |
| S3-U42 | Front IR white and black references | FR05, FR10 | hardware | Place each installed sensor on white then black at the usual mounting height and read back active white/black values. | Each channel has positive contrast, and applied calibration values are read back. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S3-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Captured Z/Y, synthetic A–Z, mapping and raw ring passed. |
| 2026-11-21 | S3-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | 7–30 mm boundaries, white outcomes, contrast, cancellation and C7 codec passed. |
| 2026-11-21 | S3-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Still/reverse gates, line reacquisition, PWM envelope and signed targets passed. |
| 2026-10-31 | S3-U42 | SDK combined baseline | Team | PASS (readback) | [Saved log](../evidence/combined_baseline/2026-10-31_front_calibration_readback.txt) | Saved front-sensor readings and calibration readback under the earlier mounting; dynamic behavior is recorded separately. |
| 2026-11-06 | S3-U40 | v0.2.0 | Team | MIXED | [Saved log](logs/2026-11-06_v0.2.0_barcode_attempts.csv) | Run 3 decoded and applied Y, Z and F. Runs 4 and 5 each rejected an incomplete attempt after two valid reads. |
| 2026-11-17 | S3-U41 | v0.2.7 | Team | FAIL (false completion) | [Saved log](logs/2026-11-17_v0.2.7_uturn.md) | Firmware accepted a transient one-sided reading and reported completion while video showed the robot off the intended path. |
| 2026-11-21 | S3-U40, S3-U41 | v0.4.2 | Team | PASS (run 4) | [Saved log](../system/logs/2026-11-21_run_4/README.md) | Z, Q and A decoded and executed right, U-turn and left; video confirms the outgoing line. |
