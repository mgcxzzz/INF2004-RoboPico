# RoboPico RTOS bench dashboard

This is the browser MQTT dashboard for the micro T-Kernel integration snapshot.
It intentionally renders only the live Buddy 1 source-1 heartbeat, ACKs, and
source-8 barcode-map readbacks. This build has its motors disabled and its
supervisory commands route through Buddy 1 to the kernel navigation stub.
Motion, line, IMU, obstacle, safety, and navigation-event telemetry are not
claimed by this bench page; their panels and charts are absent until those
firmware bridges publish real frames.

## Start the laptop dashboard and broker

On Windows, double-click **`Start dashboard.cmd`**, or open PowerShell here and
run:

```powershell
py -3 run_dashboard.py
```

The helper starts Mosquitto if it is installed and both local broker ports are
available. It uses `mosquitto.conf` for TCP MQTT on port 1883 and browser
WebSockets on port 9001. The dashboard is served on port 8765 and opened in the
default browser. If Mosquitto is not installed, the page still opens but stays
disconnected until a broker listens on both ports. Stop with Ctrl+C in the
helper window. It stops only the Mosquitto process it started itself.

Connect the Pico to the same Wi-Fi network and configure its MQTT host as the
laptop's LAN IP shown by the helper. In the laptop browser, the default is
`ws://localhost:9001`. A phone on the same LAN can use the printed dashboard
and WebSocket URLs. Do not expose the anonymous example listeners outside an
isolated lab network.

`vendor/mqtt.min.js` is the MQTT.js 5.10.1 browser build from the official npm
package and is served locally, so the page does not need a CDN or internet
access. Its SHA-256 is
`B088A7F9045DF4E478DBC378F41125066E43D9C602755EE4C5CDA0F3E9380BA0`;
`vendor/LICENSE.mqtt.md` contains its MIT license.

## What the page reports

- Browser-to-broker status and Buddy 1 heartbeat status are shown separately.
  Heartbeat fields show the Wi-Fi/MQTT state, kernel navigation state, Pico
  uptime, publish/loss counts, accepted/rejected commands, reconnect count,
  queue fill, and the heartbeat sequence delivery estimate.
- The barcode map editor offers four distinct A–Z slots in the kernel's fixed
  Forward, U-turn, Left, Right order. An ACK is request acceptance; only a
  matching source-8 map readback confirms the active map and revision.
- Start, pause, resume, stop, reset, emergency-stop, and ping are supervisory
  Buddy 1 commands. Direct steer/drive/turn commands are not exposed. The
  banner states that motors are disabled in this build and that an accepted
  command is not evidence of physical motion or completion.
- The event timeline contains observed broker, command, ACK, and malformed or
  unexpected-frame messages. Empty or unavailable telemetry is never filled
  with demo data.

## Binary contract and checks

`is28_schema.js` and `test_schema.js` are copied from the stable WiFi dashboard;
their frame layouts and command encoding are not changed here. Source 1 and
source 8 follow `source/app_program/system/sys_tlm.h` and
`source/app_program/subsystem1/b1_mapping.h`. The page does not interpret
additional Buddy telemetry as if the RTOS integration already supplied it.

With Node.js installed, run the decoder tests:

```sh
node --test test_schema.js
```

The optional `ui_smoke.mjs` test serves the page locally with a mock MQTT
transport and headless Chrome. It checks source-1 values, map readback
mismatch/confirmation, supervisory command routing, the absent motion UI, and
mobile layout. It does not connect to the broker, firmware, or hardware. Set
`PLAYWRIGHT_NODE_MODULE_PATH` and `CHROME_PATH` if Playwright or Chrome is in a
different location, or pass `--browser=PATH`.
