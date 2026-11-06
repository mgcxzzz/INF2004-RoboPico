# Combined diagnostic test — Buddy1–4 RTOS v0.2.1

This update preserves Profile1 at45mm/s, calibration priority, both Code39
decoders, junction policy, IMU/ramp control, APIs/coordinator and stop rules.
It adds evidence for the three observed problems; it does not yet tune them.
Buddy5 is not integrated. No repeat ramp test is required for this update.

## Install

1. Download/extract the complete new bundle into a new folder. Stop the robot.
2. Flash `firmware/robopico_buddy1_4_rtos_v0_2_1.uf2` in BOOTSEL mode. Existing
   saved Wi-Fi and calibration remain. Do not recalibrate for this comparison.
3. Close the previous dashboard/broker helper with Ctrl+C, then run **Start
   dashboard.cmd** from this bundle. Hard-refresh the browser with Ctrl+F5.
   If a Windows Mosquitto service occupies1883 alone, stop that service first.
4. Reuse saved network settings if SSID/password and laptop broker IP are
   unchanged. Otherwise run **Setup Wi-Fi.cmd** and follow its settings-only
   BOOTSEL step. Do not reflash the application afterward.
5. Powerbank plus working motor supply is supported; no USB serial/capture
   helper is required. Wait for car-online and IMU calibration. Verify firmware
   v0.2.1, Profile1 and current calibration (tested car white122/138,
   black246/267). Apply and read back the desired A–Z mappings after power loss.

## One purposeful run

Keep one dashboard open and Telemetry on. The optional **Capture wireless
test.cmd** gives an additional passive recording, but dashboard exports suffice.
Do the same Y → Z → F route, with Y=right, Z=U-turn and F=left for comparison.
Use enough mat for the sensors/wheels throughout the turns. Video the U-turn
and final F approach if possible. Do not change gains, speed, sensor position,
calibration or labels during this comparison.

- If the car stops with an encoder or network stop, preserve that run first.
  Do not immediately Reset/restart and erase its evidence.
- If it reaches F, allow the trailing white portion of the label to pass
  if room permits, then Stop. This lets the decoder finish reporting the group.
- A short junction stripe after the label will not replace a pinned reportable
  attempt. A later multi-bar candidate can replace it: stop before another label.

## Download before another Start, Reset, refresh or power cycle

1. Exports: **Export all frames CSV** and **Export event CSV**.
2. Messages: **Save log**.
3. Stored data: **Diagnostics**, **Navigation rows**, **Raw sensor rows**, one
   at a time. Wait for Complete and retain CSV/JSON/BIN for each dataset. Allow
   multiple browser downloads if prompted. Increasing row numbers mean progress.
4. Copy the active bundle's `dashboard_session/broker.log`.
5. Zip large text/CSV logs without deleting rows; include label order, mappings,
   scan direction and physical observation. No USB connection is needed.

Keep the robot powered until downloads finish. These histories are RAM-only.
The640 recent controller rows cover roughly6.4s, not the entire run. The
all-frames page/capture records are the continuous context. The new raw JSON
also identifies its exact run/attempt; send that metadata with the waveform.

## What the new panels mean

- **Encoder fault:** frozen wheel, burst/density/wrong-direction check, recent
  bad/observed counts, lifetime counts, requested speed, measured speed and
  previous PWM. Kept through Stop/export; eligible Reset or a new actual run
  clears it. Clamping is explicitly flagged if a diagnostic value exceeded
  its packet domain. Existing encoder thresholds are unchanged.
- **Latest barcode attempt:** reportable candidates begin after at least6
  detected peaks or an actual original-decoder acceptance. A512-row pre-roll
  includes earlier samples; the main2048-row capture pins at a report. Logging
  has a minimum5ms spacing; ADC acquisition and decoding still run at their
  original rate. Trailing white/single-bar junctions cannot erase the capture.
  Truncated/start-valid/unfinished fields expose coverage limits. A failed
  14-bar group remains an unknown letter, never guessed as F.
- **Last network stop:** frozen link/connection/subscription state, Wi-Fi poll
  age, app-service age, TCP publish-ACK age and failed condition(s). These are
  TCP delivery acknowledgements, not proof of application execution. The
  record survives reconnection/Stop/Reset and clears when a new actual run
  starts. It is historical evidence, not a current-online indicator.

The network policy remains poll/app freshness250ms and recent TCP ACK3000ms.
Network loss stops motion and reconnect never automatically restarts it.
Main/IMU control priorities and the kernel-owned TX/RX architecture are unchanged.
