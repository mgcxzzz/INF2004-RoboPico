# Testing

The layout follows the [course project template](https://github.com/sirfonzie/INF2004_ProjectTemplate/tree/main/testing). Each `TEST_LOG.md` contains test definitions and dated results. Its `logs/`
directory holds the saved output for those results. Use the
[project requirements](../docs/requirements.md) to look up the FR and NFR IDs.

| Level | ID format | Test cases and results |
| --- | --- | --- |
| Unit | S1-Uxx to S5-Uxx | [Buddy 1](subsystem1/TEST_LOG.md) · [Buddy 2](subsystem2/TEST_LOG.md) · [Buddy 3](subsystem3/TEST_LOG.md) · [Buddy 4](subsystem4/TEST_LOG.md) · [Buddy 5](subsystem5/TEST_LOG.md) |
| Integration | INT-xx | [Interfaces, release verification and saved output](integration/TEST_LOG.md) |
| System | SYS-xx | [Physical cases and final circuit results](system/TEST_LOG.md) |

[Final circuit run and video timestamps](system/logs/2026-11-21_run_4/README.md) ·
[v0.4.3 verification](integration/logs/2026-11-21_v0.4.3/README.md) ·
[Earlier evidence](evidence/README.md) · [Video links](../videos/README.md)

## Development evidence

The numbered ledgers include selected bench runs, on-car failures, and retests from earlier firmware. The result rows link to saved extracts or to archived standalone records in this repository. Capture extracts retain the source filenames and SHA-256 hashes where available. Related subsystem ledgers can cite the same circuit capture.

- [Buddy 1 standalone test ledger](evidence/buddy1/standalone/TEST_LOG.md)
- [Buddy 2 wheel-response traces](evidence/buddy2/hardware/)
- [Buddy 3 standalone test ledger](evidence/buddy3/standalone/TEST_LOG.md)
- [Buddy 4 standalone IMU ledger](evidence/buddy4/standalone/TEST_LOG.md) and [RTOS migration runs](evidence/rtos_buddy1_4/step5_v0_1_5/)
- [Buddy 5 standalone bench record](evidence/buddy5/standalone/BENCH_PROFILE_TEST_2026-10-31.md) and [integrated obstacle captures](subsystem5/logs/2026-11-20_to_21_obstacle_capture_index.csv)

## Run the tests

From the repository root, use Python 3, a native C compiler and Node.js. Set
`CC` if the default compiler is unavailable. Run the C suite before dashboard
checks so its packet fixtures are available.

```sh
python tools/rtos_migration/run_tests.py
node --test dashboard/test/test_b5_rtos.js dashboard/test/test_imu_lift_v023.js dashboard/test/test_wireless_calibration.js dashboard/test/test_source15.js
python tools/coding_standard/check.py
python tools/coding_standard/format.py --check
```

To repeat one named C check, run
`python tools/rtos_migration/run_tests.py --only test_b5_mission --skip-python`
with the desired program name. For hardware tests, build or flash the
[release firmware](../docs/build.md) and follow the relevant system case.

## Record a result

1. Add the case, procedure and expected result to its ledger. Keep its ID for
   later reruns.
2. Run the check and save the console output, measurements or log excerpt in
   the ledger's `logs/` folder. For driving tests, export the run ZIP after
   stopping and before Reset or another Start.
3. Add a result row with the date, tester, exact firmware commit or captured
   version, outcome and evidence link. Keep earlier results when adding a
   rerun. If the result covers only part of a requirement, record that scope.
4. For physical results that a log cannot show, add the unlisted video link
   and timestamps to the evidence row and the [video index](../videos/README.md).
