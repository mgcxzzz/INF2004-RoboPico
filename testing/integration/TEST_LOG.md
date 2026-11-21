# Integration — test log

Cross-subsystem checks for the production micro T-Kernel release. Use the
[test runner](../README.md#run-the-tests) for the named programs.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| INT-10 | MQTT commands to motion supervisor | FR02, FR03 | auto | `test_connected_bridge`, `test_release_policy` | Pause/Resume and valid configuration reach the coordinator; stale commands and unsafe starts are rejected. |
| INT-11 | Motion lease and bounded control | FR04, FR09 | auto | `test_continuous_integration`, `test_nav_steer` | One owner commands motion; loss of permission invalidates an obsolete motor update. |
| INT-12 | Barcode to coordinator and junction | FR06, FR07 | auto | `test_rtos_barcode`, `test_barcode_gate` | An accepted mapping waits for a qualified crossing, then dispatches the approved action once. |
| INT-13 | IMU and ramp assist to shared motion | FR08, FR09 | auto | `test_rtos_ramp_bridge`, `test_release_imu` | Fresh bounded assistance leaves ownership with Buddy 3; stale data, stops and incompatible manoeuvres remove assist. |
| INT-14 | Obstacle takeover, warning and line handback | FR11, FR12 | auto | `test_b5_mission`, `test_b5_straight_gate` | Confirmed close obstacle transfers the lease; the borderline replay warns and completes the bypass; Buddy 3 resumes. |
| INT-15 | Compact records and dashboard interoperability | FR10, NFR08 | auto | `test_compact_signed`, `test_release_exports` and dashboard Node tests | Packed values and flags round-trip with expected byte counts and reserved fields. |
| INT-16 | Release image, kernel and C checks | NFR01, NFR09 | auto + review | Rebuild the pinned release; check source, UF2 hash, linked tasks, allocation and coding exceptions. | The UF2 matches the release, RTOS tasks and motion ownership link as intended, and in-scope checks pass. |
| INT-17 | RAM and payload size comparison | NFR08 | review | Build the equivalent baseline and current profile with the same settings. Compare linker RAM use and the same encoded messages. | The record shows baseline and current byte counts and identifies the fields responsible for any difference. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | INT-10 | `8aa5f2c` | Release verification | PASS | [Host and dashboard output](logs/2026-11-21_v0.4.3/host-and-dashboard.txt) | Epochs, STOP priority, configuration readback and real Pause/Resume passed. |
| 2026-11-21 | INT-11 | `8aa5f2c` | Release verification | PASS | [Host and dashboard output](logs/2026-11-21_v0.4.3/host-and-dashboard.txt) | Ownership, stale commit rejection and motion envelope checks passed. |
| 2026-11-21 | INT-12 | `8aa5f2c` | Release verification | PASS | [Host and dashboard output](logs/2026-11-21_v0.4.3/host-and-dashboard.txt) | Captured barcode, synthetic inputs and gated dispatch passed. |
| 2026-11-21 | INT-13 | `8aa5f2c` | Release verification | PASS | [Host and dashboard output](logs/2026-11-21_v0.4.3/host-and-dashboard.txt) | Real RTOS ramp and lift service paths passed. |
| 2026-11-21 | INT-14 | `8aa5f2c` | Release verification | PASS | [Host and dashboard output](logs/2026-11-21_v0.4.3/host-and-dashboard.txt) | Replayed the stopped sonar range change and decoded its warning. No simulated contact. |
| 2026-11-21 | INT-15 | `8aa5f2c` | Release verification | PASS | [Host and dashboard output](logs/2026-11-21_v0.4.3/host-and-dashboard.txt) | Compact layout checks and 28 dashboard tests passed. |
| 2026-11-21 | INT-16 | `8aa5f2c` | Release verification | PASS | [Build and review record](logs/2026-11-21_v0.4.3/README.md) | Byte-identical Arm rebuild, 2,024-byte estimated kernel margin and 145 files checked. |

See the [physical system results](../system/TEST_LOG.md) for the circuit run.
