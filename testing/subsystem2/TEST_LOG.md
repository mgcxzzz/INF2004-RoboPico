# Subsystem 2 — Wheel control and odometry

[Motion code](../../firmware/integration/buddy1_4/src/motion/) and [RTOS motor and safety tasks](../../app_program/rtos_bringup/).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S2-U20 | Encoder error qualification | FR04 | auto | `test_encoder_quality` | Sparse errors, sustained errors, counter wrap and Pause follow the configured policy. |
| S2-U21 | Encoder fault snapshot | FR04, FR09 | auto | `test_encoder_fault` | The first fault latches and the compact C4 report round-trips and clears. |
| S2-U22 | Motion readiness and stop gates | FR02, NFR02 | auto | `test_bringup_policy` | Clock, start and urgent-stop policy blocks drive under the exercised conditions. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S2-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Sparse and repeated errors, density, Pause and saturation covered. |
| 2026-11-21 | S2-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Latch, compact codec and clear frame passed. |
| 2026-11-21 | S2-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Clock disagreement, start gating, urgent-stop gates and wrap passed. |
