# RoboPico UTK Drive V1 dashboard

This is the browser MQTT client for the `interface_utk_drive_v1` integration.
It uses the unchanged WiFi V2 frame and command contract and displays only
packets received from the car. The integration brings the V2 steering/kernel
network path together and is drive-capable; the robot-level hardware drive
path still needs physical validation. Keep the robot secured and emergency
stop accessible during first tests. This image does not publish IMU or
obstacle telemetry, and the dashboard does not make up values for those
systems.

## Start the broker and dashboard

1. Install Mosquitto on the laptop and open a terminal in this dashboard
   directory.
2. Start the broker with `mosquitto -c mosquitto.conf -v`. The config listens
   for car MQTT clients on TCP port **1883** and browser WebSockets on port
   **9001**.
3. Open `index.html` in a modern browser on the laptop. Enter
   `ws://localhost:9001` (pre-filled by default), then select **Connect**.
   The dashboard uses the vendored MQTT.js 5.10.1 browser bundle and runs
   without a CDN or internet connection. The firmware must use the laptop's
   LAN IP as its broker host; `localhost` only works for the browser running
   on the laptop.
4. Confirm the browser shows “broker connected” and then wait for the car's
   heartbeat and telemetry. A broker connection alone does not mean the Pico
   is online.

The Mosquitto example allows anonymous connections and is intended for an
isolated lab network. Do not expose these listeners to an untrusted network.

`vendor/mqtt.min.js` is the MQTT.js 5.10.1 browser distribution from the
official npm package (`mqtt@5.10.1`). SHA-256:
`B088A7F9045DF4E478DBC378F41125066E43D9C602755EE4C5CDA0F3E9380BA0`.
The corresponding MIT license is included at `vendor/LICENSE.mqtt.md`.

## Dashboard behavior

- Motion telemetry shows packet timestamp/uptime, run time and ID, measured and
  target wheel speeds, PWM, raw ADC (left GP26/ADC0, right GP28/ADC2, barcode
  GP27/ADC1), calibrated line levels, line mask, distance, heading, profile,
  phase, recovery state and average encoder speed derived from distance/time. Scrolling speed, PWM, ADC, and calibrated-level
  charts keep the latest 240 received samples and use packet uptime on the
  horizontal axis.
- The laptop page retains up to 72,000 motion samples and 20,000 real event
  records for the current page session. The dashboard displays dropped-row
  counts and a truncation warning at either cap. CSV and JSON exports include
  all retained rows; JSON also records the limits and dropped counts. Routine
  source-10/source-11 status packets update the live panels but enter the event
  timeline only when tracked status changes.
- Motion exports download the retained page-session sample buffer as CSV or JSON. Event
  exports contain only received line/barcode/coordinator records. Exporting an
  empty buffer produces an empty data file; the dashboard does not create demo
  samples.
- Stored downloads are enabled only when coordinator telemetry reports `Off`
  continuously for at least 5 seconds after a manual stop. Paused mode does
  not enable them because its active snapshot can still change. The browser
  fetches rows sequentially with a matching request ID,
  reassembles source-13 chunks, and retries a timed-out/missing row at most
  three times. Any rejection, disconnect, missing part, or row-size mismatch
  stops the transfer without producing a complete export. Successful transfers
  download concatenated fixed-size `.bin` rows, a `.csv`, and a JSON manifest.
  Motion rows (48 bytes) are decoded to CSV; diagnostic (58-byte), raw sensor
  (10-byte), and navigation (31-byte) rows are exported as explicit row hex
  until their field schemas are available.
- The barcode selector enforces four distinct uppercase A–Z letters in the
  fixed Forward, U-turn, Left, Right slot order. Commands publish to
  `is28/cmd` with `retain=false`. An ACK means the firmware accepted the
  request; only a matching source-8 map readback confirms the active mapping
  and revision.
- Source-10 events expose the decoded letter, pending letter/action/revision,
  and executing action/revision separately. Coordinator and run-summary
  records are timestamped in the event timeline. These fields describe
  reported firmware events and do not imply completion unless a completion
  event is received.
- Browser commands are supervisory only. Direct steering, drive, and turn
  commands are not offered. Emergency stop remains available.

The banner on the dashboard identifies this as **UTK Drive V1** and explains
that Start/Resume reach the live kernel drive path. An accepted ACK means the
firmware accepted the request; telemetry and coordinator events describe only
what the car actually reported.

## MQTT topics and binary schema

The firmware publishes `is28/hb`, `is28/tlm`, `is28/ack`, and `is28/lwt`;
commands are sent to `is28/cmd`. Telemetry frames have a 3-byte header:
`[source << 4 | version 2, payload length, sequence]`. Integer fields are
little-endian. The browser decoder and its no-dependency tests are
`is28_schema.js` and `test_schema.js`; protocol layout is maintained with
`../source/WIFI_CONTRACT.md` and the WiFi protocol implementation.

The dashboard source decoder handles source 1 WiFi status, source 8 active map
readback, source 9 motion, source 10 line/barcode events, source 11 coordinator
events, source 12 run summaries, variable-length source 13 log chunks, and
source 14 periodic barcode/transport diagnostics.
Command 11 (`GET_LOG`) uses its high argument nibble for dataset 0–3 and low 12
bits for row index. Event codes use the WiFi protocol enum;
phase and recovery codes remain numeric until their firmware enums are fixed.
Unknown enum codes stay visible as their numeric values instead of being guessed.

## Decoder tests

With Node.js installed, run:

```sh
node --test test_schema.js
```

This exercises exact frame lengths, malformed headers, signed motion fields,
the separate barcode states, and A–Z mapping slots. It does not emulate a
browser, broker, WiFi transport, or Pico hardware.

The optional headless browser smoke test is `ui_smoke.mjs`. It serves this
dashboard locally, replaces the MQTT transport with deterministic packet
vectors, exercises map mismatch/readback plus multi-part stored-log download,
and writes desktop/mobile screenshots under the ignored
`../build/review/dashboard-smoke/` folder. It needs Playwright and Chrome; for
other paths, set `PLAYWRIGHT_NODE_MODULE_PATH` and `CHROME_PATH`, or pass
`--browser=PATH`. It does not connect to a real broker or Pico.
