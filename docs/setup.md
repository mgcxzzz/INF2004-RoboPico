# Set up RoboPico Buddy 1–5 RTOS v0.2.13

Download and extract the [full bundle](../firmware/integration/buddy1_4/artifacts/v0_2_13_combined/).
Use [the first hardware test guide](buddy1_5_v0_2_13.md) for obstacle placement
and expected results.

## Flash and connect

1. Install Python 3.10+ and Mosquitto on the laptop. Close the older dashboard
   launcher. With motion stopped, hold BOOTSEL while plugging in the Pico W.
   Copy `firmware/robopico_buddy1_5_rtos_v0_2_13.uf2` from the bundle onto
   `RPI-RP2`. Let the car boot flat and still for IMU calibration.
2. Start the bundled dashboard. On Windows, run **Start dashboard.cmd**. On
   macOS, run `python3 tools/dashboard/run_dashboard.py` from the bundle root.
   If Mosquitto is installed through Homebrew and the helper does not find it,
   pass `--mosquitto "$(brew --prefix mosquitto)/sbin/mosquitto"`.
3. Open `http://localhost:8765` and connect to `ws://localhost:9001`.
   **Broker connected** confirms the laptop connection. Wait for **Car online**
   and **RTOS Buddy 1–5 v0.2.13** to confirm the car connection and firmware.
4. If the saved network or broker address changed, run **Configure WiFi.cmd**
   on Windows or `python3 tools/rtos_migration/provision_network.py` on macOS.
   Enter the 2.4 GHz network details and the laptop's active LAN IPv4. Follow
   the helper's BOOTSEL prompt to apply the settings-only UF2. The Pico uses
   the laptop's LAN address on MQTT TCP 1883. The dashboard uses localhost
   WebSockets on TCP 9001. If the broker connects but the car stays offline,
   check `dashboard_session/broker.log` for the Pico connection and verify
   that the laptop accepts LAN connections to TCP 1883.

## Run from a powerbank

1. Use the car's normal controller powerbank and motor supply. Keep the laptop
   broker running on the same network. USB serial and capture helpers are optional.
2. Keep telemetry on and choose **Baseline · 45 mm/s** for the first test.
   Apply four distinct A–Z barcode mappings and confirm readback. Mapping
   lives in RAM and needs to be reapplied after power loss. Saved front IR
   calibration is reported in **Diagnostics → Front IR calibration**.
3. Press **Start** after the robot is ready. The two-second countdown runs
   before motion. **Stop** or GP21 stops the run. GP20 is the physical emergency
   stop. A stale supervised network link stops motion. Restart requires
   an explicit Start after any required Reset.
4. During avoidance, **Pause** ends the run. When Buddy 5 reports a hold, press
   **Stop** to release the hold before starting a new run.

## Save a run

1. Press **Stop** and keep the car and the broker powered. Wait about five
   seconds for coordinator Off. Click **Download run ZIP** next to drive mode
   in the dashboard.
2. Wait for collection. The ZIP includes dashboard frames and events, Buddy 5
   status, and the stored motion, diagnostics, raw sensor and navigation
   datasets. A `_PARTIAL.zip` suffix means collection was interrupted. Keep it
   and retry while the same run is still in RAM.
3. Export before starting again, Reset or reboot. Note the obstacle placement
   and last stage shown in **Obstacle & avoidance**. For probe detail see
   **Diagnostics → Buddy 5 obstacle mission**.

For an optional passive MQTT capture, run
`python3 tools/rtos_migration/capture_wireless.py` before Start
(`py -3` on Windows). It records the live stream for diagnosis. See the
[dashboard guide](../dashboard/README.md) for dashboard paths and file details.
