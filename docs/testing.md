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
