# Subsystem 5 — Obstacle detection, width and bypass

[Buddy 5 mission code](../../app_program/obstacle_navigation/) and [physical obstacle tests](../system/TEST_LOG.md).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S5-U20 | Straight qualification and junction protection | FR11, FR12 | auto | `test_b5_straight_gate` | Stale and turning states block normal takeover; a qualified fresh straight permits it. |
| S5-U21 | Borderline echo replay | FR11, FR12 | auto | `test_b5_mission`, captured borderline batch | The corroborated short echo yields an accepted warning; the simulated bypass rejoins. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S5-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | 2,588 gate checks, including completion and fresh-straight evidence. |
| 2026-11-21 | S5-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Exact batch completed simulated bypass and handback with zero simulated contacts. |
