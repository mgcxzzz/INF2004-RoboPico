# Offline regressions

Run `python tools/rtos_migration/run_tests.py` from the repository root for the current RTOS profile. See the [testing index](../../../../testing/README.md) for browser fixtures and on-car cases. The standalone SDK
regression runner is `python firmware/integration/buddy1_4/tests/run_tests.py`.
Both runners keep C assertions enabled and use host hardware stubs.

`unit/` contains inherited control/interface/storage/network checks plus the new coordinator-assist and interrupted-barcode regressions. `barcode/` contains real captured traces and dual-decoder integration tests. `ramp/` contains original numeric references and decision/plant/budget tests. `imu/` contains fake-sensor service, powered-acceptance and packer checks. `transport_test_stubs/` substitutes only SDK/network boundaries. `compat/` retains the old schema decoder as a compatibility reference.

The [archived source-preservation report](../../../../testing/evidence/current/README.md) records comparison of 17 retained algorithms against an earlier baseline. See the [v0.4.3 verification record](../../../../testing/integration/logs/2026-11-21_v0.4.3/README.md) for current validation.
