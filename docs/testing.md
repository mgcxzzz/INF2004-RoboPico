# Focused checks for Buddy 1–4 RTOS v0.2.1

The tested hardware baseline is Step 5 v0.1.5. This release changes supervision,
wireless exports and dashboard/capture support, not the tracker/decoder/control/
IMU/ramp algorithms. [Physical status](../firmware/integration/buddy1_4/STATUS.md)
keeps circuit and new powerbank checks distinct from software checks.

With CC pointing to a host C compiler, from repository root:

```bash
python tools/rtos_migration/run_tests.py --only test_bringup_policy test_connected_bridge test_release_policy test_release_bridge test_release_exports test_release_imu test_encoder_fault test_network_stop test_barcode_attempt --skip-python
python testing/rtos_migration/test_capture_wireless.py
python testing/rtos_migration/test_capture_connected_line.py
python testing/rtos_migration/test_export_line_log.py
python testing/rtos_migration/test_export_barcode_log.py
node dashboard/test/test_release.js
node --test dashboard/test/test_diagnostics_v021.js dashboard/test/test_source15.js dashboard/test/test_ramp_targets.js dashboard/test/test_merged_schema.js
```

The C export/IMU fixtures produce binary vectors consumed by the browser-codec
check, so run them before test_release.js. The focused local-browser check
`node dashboard/test/ui_release.mjs` uses simulated MQTT and needs Playwright/
Chrome(set PLAYWRIGHT_NODE_MODULE_PATH and CHROME_PATH if needed). It sends no
real car commands. No hardware test is fabricated by these fixtures.

The ARM build and packager additionally check required production symbols,
reviewed source fingerprints, unchanged baseline algorithm inputs, UF2 flash
bounds, static RAM, stack-frame reports and bundle hashes. Wider unchanged
algorithm suites are not repeated for this promotion. The original broad
SDK tests remain documented in sdk_reference_testing.md.
