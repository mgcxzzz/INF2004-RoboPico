# RoboPico v0.2.9 calibration reporting test build

This candidate is on `barr-c-cleanup`. The dashboard identifies calibration
packets by length and subtype, so a ramp or barcode-peak timestamp byte of
`0xCC` no longer masks valid telemetry. Firmware takes front sensor samples on
the control task and encodes calibration status from a protected copy only when
sending a report. Heartbeat investigation is tracked separately.

## Set up

1. Extract the full bundle and flash
   `firmware/robopico_buddy1_4_rtos_v0_2_9_calibration_reporting.uf2` through BOOTSEL.
   Existing saved front IR calibration and Wi-Fi settings are loaded on boot.
2. Start the dashboard from this bundle. On Windows run `Start dashboard.cmd`.
   On macOS run `python3 tools/dashboard/run_dashboard.py` from the bundle root.
3. Check that the console and the firmware label both show **v0.2.9**.

## Focused test

1. While fully stopped, open **Diagnostics → Front IR calibration**. Verify
   the live raw readings and active reference values appear. Begin, capture
   white/left black/right black, and check the readback. Discard the candidate
   to keep the active values, or choose **Use in RAM** to test the new capture.
2. Do one ordinary barcode or ramp run. Check that the corresponding dashboard
   telemetry updates. Offline checks already exercise the exact timestamp cases
   that caused the dashboard decode error.
3. If reproducing the hotspot stop, start `Capture wireless test.cmd` before
   the run. On macOS run `python3 tools/rtos_migration/capture_wireless.py`.
   After the car reconnects, export **Event CSV** and **All Frames CSV** before
   Reset and keep the capture ZIP or its MQTT CSV and summary together.

See `docs/wireless_calibration_v0_2_8.md` in the bundle for sensor positioning,
capture guidance and calibration persistence. The two fixes preserve the
existing command and telemetry layouts and stopped-only flash path. Hardware
timing and the reported hotspot stops still need a run on the car.
