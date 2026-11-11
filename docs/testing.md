# Focused checks for Buddy 1–4 RTOS v0.2.3

This release adds IMU end-run safety while retaining v0.2.2 controls. Purposeful
checks cover the new detector/service path, safety gates, packet/dashboard and
release compatibility, plus one retained legacy IMU/ramp fixture after the
release-only collision opt-out. No unrelated exhaustive suites are needed.

```bash
python tools/rtos_migration/run_tests.py --only test_bringup_policy test_imu_lift_guard test_imu_lift_packet test_release_imu test_release_bridge test_release_exports test_imu_service_rtos --skip-python
node --test dashboard/test/test_imu_lift_v023.js
node dashboard/test/test_release.js
node dashboard/test/ui_release.mjs
```

Generate the native C fixtures before browser checks. The real production service
check compiles APP_RTOS_RELEASE in every translation unit: a header-local macro
alone cannot exercise the release-only FIFO hook. The host fixture checks latched
lift with the actual transformed FIFO, no production Impact transition, blocked
raw reset,500 ms upright readiness, stale evidence rejection and recalibration
handoff. The browser uses simulated MQTT; it does not command the robot.

Physical lift/overturn discrimination, wheel-stop latency, and ramp false-positive
acceptance remain to test. See [the guide](imu_end_run_v0_2_3.md).

The earlier v0.2.2 checks below remain historical evidence for retained code.

# Focused checks for Buddy 1–4 RTOS v0.2.2

The hardware baseline is Step 5 v0.1.5 and main v0.2.1. This release changes
only opt-in bend/search behaviour, RTOS junction width qualification and copied
diagnostics/dashboard support. Physical acceptance of those changes is pending.

From repository root, with CC pointing to the native C compiler:

```bash
python tools/rtos_migration/run_tests.py --only test_continuous_bend_assist test_line_stationary_search test_junction_sensor test_drive_diagnostics test_continuous_steering test_continuous_follow test_line_search test_release_bridge test_release_exports test_rtos_barcode --skip-python
python testing/rtos_migration/test_capture_wireless.py
python testing/rtos_migration/test_export_line_log.py
node dashboard/test/test_release.js
node --test dashboard/test/test_drive_v022.js dashboard/test/test_diagnostics_v021.js
node dashboard/test/ui_release.mjs
```

The C programs generate binary fixtures consumed by the browser-codec checks;
run them first. The browser check uses simulated MQTT and Playwright/Chrome,
not real robot commands. It verifies the live diagnostics and stopped exports.

Each test has a purpose: exact legacy straight behaviour; persistent bend entry,
mirrored response and fade; signed search power and settling, no-motion/angle/time
limits and line return; 7 mm junction/white gating; shared coordinator/Stop/export
behaviour; unchanged decoder integration; and actual C-to-dashboard codecs.

The ARM build and package check required production symbols, exact reviewed
source fingerprints, unchanged protected algorithms, flash bounds, static RAM,
stack frames and archive hashes. No broad IMU/ramp rerun is required when those
sources remain identical. Hardware steps are in drive_refinements_v0_2_2.md.
