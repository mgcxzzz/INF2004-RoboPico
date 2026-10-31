# Offline regressions

Run `python firmware/integration/buddy1_4/tests/run_tests.py` from repository root. C assertions remain enabled. Tests use a normal host C compiler and hardware-boundary stubs; they do not flash, open serial, connect to a broker or drive the car.

`unit/` contains inherited control/interface/storage/network checks plus the new coordinator-assist and interrupted-barcode regressions. `barcode/` contains real captured traces and dual-decoder integration tests. `ramp/` contains original numeric references and decision/plant/budget tests. `imu/` contains fake-sensor service, powered-acceptance and packer checks. `transport_test_stubs/` substitutes only SDK/network boundaries. `compat/` retains the old schema decoder as a compatibility reference.

The source preservation report in testing/evidence/current compares17 retained algorithm token streams against the pinned combined baseline, ignoring only includes/comments/format. A passing software suite does not mark this provisional image as hardware-accepted; consult STATUS.md and the separate evidence index.
