# Offline regressions

On `barr-c-combined`, run `python tools/rtos_migration/run_tests.py` from the
repository root. See the [combined test guide](../../../../docs/testing.md)
for the browser fixtures and physical test procedure. The standalone SDK
regression runner is `python firmware/integration/buddy1_4/tests/run_tests.py`.
Both runners keep C assertions enabled and use host hardware stubs.

`unit/` contains inherited control/interface/storage/network checks plus the new coordinator-assist and interrupted-barcode regressions. `barcode/` contains real captured traces and dual-decoder integration tests. `ramp/` contains original numeric references and decision/plant/budget tests. `imu/` contains fake-sensor service, powered-acceptance and packer checks. `transport_test_stubs/` substitutes only SDK/network boundaries. `compat/` retains the old schema decoder as a compatibility reference.

The source preservation report in testing/evidence/current compares17 retained algorithm token streams against the pinned combined baseline, ignoring only includes/comments/format. A passing software suite does not mark this provisional image as hardware-accepted; consult STATUS.md and the separate evidence index.
