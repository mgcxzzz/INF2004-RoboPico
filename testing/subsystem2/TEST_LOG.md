# Subsystem 2 — Wheel control and odometry

[Motion code](../../firmware/integration/buddy1_4/src/motion/) and [RTOS motor and safety tasks](../../app_program/rtos_bringup/).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S2-U20 | Encoder error qualification | FR04 | auto | `test_encoder_quality` | Sparse errors, sustained errors, counter wrap and Pause follow the configured policy. |
| S2-U21 | Encoder fault snapshot | FR04, FR09 | auto | `test_encoder_fault` | The first fault latches and the compact C4 report round-trips and clears. |
| S2-U22 | Motion readiness and stop gates | FR02, NFR02 | auto | `test_bringup_policy` | Clock, start and urgent-stop policy blocks drive under the exercised conditions. |
| S2-U40 | Wheel response and drive symmetry | FR04, NFR04 | hardware | Request equal wheel speed; record PWM, encoder speed, wheel distance and heading. | Both wheel responses are observable and independent; compare their speed and distance traces. |
| S2-U41 | Encoder safety latch on a turn | FR04, FR09 | hardware capture | Review stored controller and navigation samples around a stopped U-turn. | Evidence identifies the triggering fault family and its effect on the action. |
| S2-U42 | Loaded wheel response at a mat seam | FR04, FR05 | hardware | Use the usual mat and Baseline drive. Capture active targets, measured wheel speed and PWM through a snag and return to motion. | Record sustained near-zero responses under power and whether the car continues unaided. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S2-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Sparse and repeated errors, density, Pause and saturation covered. |
| 2026-11-21 | S2-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Latch, compact codec and clear frame passed. |
| 2026-11-21 | S2-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Clock disagreement, start gating, urgent-stop gates and wrap passed. |
| 2026-10-21 | S2-U40 | SDK wheel test v4 | Team | MEASURED | [Saved log](../evidence/buddy2/hardware/v4_wheel_response_2026-10-21.txt) | Paired 65 mm/s targets produced asymmetric measured response. Raw trace preserves target, measured speed and PWM. |
| 2026-11-06 | S2-U41 | v0.2.0 | Team | FAIL (U-turn) | [Saved log](logs/2026-11-06_v0.2.0_encoder_trip.json) | Run 2 ended with reason 9 and fault mask 64 at 29.189 s. [Follow-up C4 snapshot](https://github.com/mgcxzzz/INF2004-RoboPico/commit/6da9d49) captures more evidence on the first trip. |
| 2026-11-19 | S2-U42 | v0.2.9 | Team | FAIL (manual intervention) | [Motion windows](logs/2026-11-19_v0.2.9_no_motion_windows.json) | Multiple periods of near-zero speed had positive drive demand. Operator reported clearing snags manually. Exact intervention times are unknown. [v0.2.12 follow-up](../../docs/caster_assist_v0_2_12.md) added bounded assist and a first-stop record. |
