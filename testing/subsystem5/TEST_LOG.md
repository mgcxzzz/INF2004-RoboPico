# Subsystem 5 — Obstacle detection, width and bypass

[Buddy 5 mission code](../../app_program/obstacle_navigation/) and [physical obstacle tests](../system/TEST_LOG.md).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S5-U20 | Straight qualification and junction protection | FR11, FR12 | auto | `test_b5_straight_gate` | Stale and turning states block normal takeover; a qualified fresh straight permits it. |
| S5-U21 | Borderline echo replay | FR11, FR12 | auto | `test_b5_mission`, captured borderline batch | The corroborated short echo yields an accepted warning; the simulated bypass rejoins. |
| S5-U40 | Standalone sonar and servo profiling | FR11, NFR07 | hardware bench | Measure fixed ranges and run physical sonar/servo scans with simulated vehicle and line. | Range and side repeatability are measured under the bench setup. |
| S5-U41 | Takeover after a bend or junction | FR11, FR12 | hardware capture | Approach a box on a straight just after the bend, and separately pass a side echo during a junction turn. | When straight, progress through detection and probe. During the protected turn, allow navigation to finish. |
| S5-U42 | Stopped range and width probe | FR11 | hardware capture | Approach a stationary obstacle. Compare trigger range, width probe state and width-reference result across repeats. | A qualified stopped range transitions to a probe. The reported width includes its basis. |
| S5-U43 | Approach, rejoin and handback on course | FR12 | hardware + video | Review line evidence before bypass and after the return leg; verify navigation handback. | The robot finds the intended stripe and returns ownership to line following. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S5-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | 2,588 gate checks, including completion and fresh-straight evidence. |
| 2026-11-21 | S5-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Exact batch completed simulated bypass and handback with zero simulated contacts. |
| 2026-10-31 | S5-U40 | Buddy 5 bench | Teammate | PARTIAL | [Saved log](../evidence/buddy5/standalone/BENCH_PROFILE_TEST_2026-10-31.md) | Physical scans of 100/180 mm boxes exercised side choice. The test record retains the RTOS fixed-range bench fault and unfinished repeats. |
| 2026-11-20 | S5-U41 | v0.2.13 / v0.2.14 | Team | FAIL (takeover) | [Saved log](logs/2026-11-20_to_21_obstacle_capture_index.csv) | The captured settling and bend-exit gates blocked approach before width or direction was available. |
| 2026-11-20 | S5-U42 | v0.2.15 / v0.3.0 | Team | FAIL (probe) | [Saved log](logs/2026-11-20_to_21_obstacle_capture_index.csv) | Exports show RUN_OR_REFERENCE and width probe aborts at distinct 274 mm and 293 mm trigger readings. |
| 2026-11-20 | S5-U43 | v0.3.1 to v0.3.7 | Team | FAIL (approach / rejoin) | [Saved log](logs/2026-11-20_to_21_obstacle_capture_index.csv) | Different captures preserve approach line loss, alignment search exhaustion, and return-to-line failure after starting bypass. |
| 2026-11-21 | S5-U41 | v0.3.12 / v0.4.1 | Team | FAIL (legacy gate) | [Saved log](logs/2026-11-20_to_21_obstacle_capture_index.csv) | The earlier turn gate held in v0.3.12, and a separate v0.4.1 capture held during straight settling after the bend. |
| 2026-11-21 | S5-U42, S5-U43 | v0.4.2 | Team | PASS (run 4) | [Saved log](../system/logs/2026-11-21_run_4/README.md) | The car detected the box, estimated 138.7 mm, travelled around it, reacquired the original tape and logged HANDBACK. |
