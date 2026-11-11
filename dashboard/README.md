# Buddy 1–4 RTOS dashboard v0.2.3

The dashboard displays the main v0.2.3 release metadata and the source-15 C9
IMU lift/overturn safety snapshot. C9 is sent once per second, including when
ordinary telemetry is off. The end-of-run panel names a lift or overturn only
from a fresh, latched snapshot; heartbeat Fault remains a separate generic
operating-state report. A stale C9 reason is not presented as a current trip,
and the dashboard never resets or restarts the car automatically.

Use the full release bundle and hard-refresh the page after updating. Start
with [the diagnostic test/download guide](../docs/diagnostics_v0_2_1.md).

Run `python tools/dashboard/run_dashboard.py` from repository root or double-click
Start dashboard.cmd. The helper serves this folder, starts a local Mosquitto
broker if available, and keeps its broker log under dashboard_session/. Existing
listeners are checked, not silently replaced. Install Mosquitto separately.

Connect to `ws://localhost:9001` on the laptop. The Pico uses the laptop's LAN
IPv4 on MQTT TCP1883, not localhost. Car-online requires a fresh heartbeat,
independently of the browser-to-broker connection. The A–Z map applies through
MQTT and uses confirmed readback.

This view retains the merged SDK/RTOS decoders, requested/effective ramp targets,
and stopped data export. It does not implement Buddy5. Detailed feature logic
lives under `firmware/integration/buddy1_4/src/*/README.md`; export instructions
are in `docs/setup.md`. Vendored MQTT.js and its license are included for offline
use.
