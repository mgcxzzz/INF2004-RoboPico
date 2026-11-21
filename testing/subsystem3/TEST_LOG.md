# Subsystem 3 — Line following and barcode routing

[RTOS line and barcode integration](../../app_program/line_navigation/).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S3-U20 | Decode and mapping | FR06 | auto | `test_rtos_barcode` | Recorded and synthetic labels decode; unmapped labels queue no action. |
| S3-U21 | Junction stripe qualification | FR07 | auto | `test_junction_sensor` | Width, white margins, rejected stripes and C7 encoding follow the junction policy. |
| S3-U22 | Stationary search | FR05 | auto | `test_line_stationary_search` | Search stays bounded and resumes on credible tape evidence. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S3-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Captured Z/Y, synthetic A–Z, mapping and raw ring passed. |
| 2026-11-21 | S3-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | 7–30 mm boundaries, white outcomes, contrast, cancellation and C7 codec passed. |
| 2026-11-21 | S3-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Still/reverse gates, line reacquisition, PWM envelope and signed targets passed. |
