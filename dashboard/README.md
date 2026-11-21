# RoboPico RTOS dashboard v0.2.15

The dashboard displays the current firmware release metadata and the source-15 C9
IMU lift/overturn safety snapshot. C9 is sent once per second, including when
ordinary telemetry is off. The end-of-run panel names a lift or overturn only
from a fresh, latched snapshot; heartbeat Fault remains a separate generic
operating-state report. A stale C9 reason is not presented as a current trip,
and the dashboard never resets or restarts the car automatically.

Use the full release bundle and hard-refresh the page after updating. Start
with [the v0.4.3 firmware and dashboard bundle](../firmware/integration/buddy1_4/artifacts/v0_4_3/).

Run `python tools/dashboard/run_dashboard.py` from repository root or double-click
Start dashboard.cmd. The helper serves this folder, starts a local Mosquitto
broker if available, and keeps its broker log under dashboard_session/. Existing
listeners are checked, not silently replaced. Install Mosquitto separately.

Connect to `ws://localhost:9001` on the laptop. The Pico uses the laptop's LAN
IPv4 on MQTT TCP1883, not localhost. Car-online requires a fresh heartbeat,
independently of the browser-to-broker connection. The A–Z map applies through
MQTT and uses confirmed readback.

**Obstacle & avoidance** is the first card in the second row on the desktop
overview. It shows detection, width, direction and line rejoin using fresh
Buddy 5 telemetry from `APP=rtos_buddy1_5`. A fallback
width is labelled **assumed**. **Diagnostics → Buddy 5 obstacle mission** shows
the takeover gate, motion owner, probe policy, range, direction basis and
record completeness. Width and direction traces have independent counters.
The latest received trace may be from a prior mission.

After Stop, **Download run ZIP** includes `dashboard/obstacle.json` for Buddy 5
firmware. `dashboard/frames.csv` and `dashboard/frames.json` retain the actual
packets captured during the page session, including the width and direction
traces. For offline analysis see [the integration notes](../docs/buddy5_rtos_integration.md)
and [the trace decoder](../tools/telemetry/b5_rtos_decode.py).

The dashboard also supports the Buddy 1–4 firmware, legacy SDK and RTOS
telemetry, requested and effective ramp targets, and the stopped data export.
Setup instructions are in [the setup guide](../docs/setup.md). Vendored MQTT.js
and its license are included for offline use.


## Front IR calibration

Open **Diagnostics → Front IR calibration** after a complete Stop. Follow
[the staged capture and save instructions](../docs/wireless_calibration_v0_2_8.md).
The controls require fresh source-15 CC/28 status from compatible firmware;
older firmware cannot enable them. Command ACK alone never confirms an applied
or saved calibration. Captures and saving work over MQTT with powerbank power.
