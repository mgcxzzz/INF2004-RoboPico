# IS28 merged dashboard v0.3

v0.3 (2026-10-30) adds **source 15** for the SDK v2.3 integration builds, dispatched by validated payload
length and subtype (never by source number alone): ramp assist (payload 28; detector/estimator markers) and
barcode peak decoder v0.1 (payload 24, subtype 0xB1; flags bit7 = combined Buddy1-4 build). New cards in
*Barcode and junction progress*: barcode peak result, ramp assist, firmware build; timeline events for peak
reads/conflicts and ramp activations. The connectivity test counts the two source-15 streams separately.
"Last decoded letter (B3)" updates from SDK sources 10/11 with the mapped movement from the confirmed readback
(verified with frames from the real C packers, `test/ui_b4_peak.mjs`, `test/test_source15.js`). RTOS decoding,
restart and staleness handling are unchanged. Diff: `DASHBOARD_V0_3_SOURCE15.patch`; v0.2 page in `archive/v0_2/`.

## v0.2 notes

v0.2 (2026-10-30) simplifies v0.1: the Recording panel (run label, markers,
sync clock, export scope, evidence bundle, buffer discard) and the Firmware
profile card (manual override, packet-source list) are removed. Firmware-family
detection still runs automatically and is shown in the compact header badge;
hover it to see the observed sources. Plain exports stay in an Exports panel.
The v0.1 page is kept, not served as the default, in `archive/v0_1/`.

One local page for both firmware families on the same Mosquitto broker:

| Firmware | Sources | Panels ported from |
|---|---|---|
| RTOS (micro T-Kernel) Buddy1/Buddy4 build | 1–8 | `tools/dashboard` (port 9071, unchanged) |
| SDK v2.3 Buddy2/Buddy3 build, optionally with the Buddy4 add-on | 1, 2, 4, 8, 9–14 | `testing/subsystem2/interface_wifi_v2_3/dashboard` @ 8d7f3aa (unchanged) |

This is dashboard-only work. Firmware, calibration, motor control and the MQTT
protocol are untouched. Plain HTML/JS/CSS with the vendored MQTT.js 5.10.1
(`vendor/mqtt.min.js`, SHA-256 `B088A7F9…80BA0`, MIT licence in `vendor/`). It
needs no CDN or internet access.

## Launch

The existing broker is reused (TCP 1883, WebSocket 9001). Do not start a second
broker. The 9071 dashboard and the native `mosquitto_sub` recorder keep running.
From the repository root:

```sh
python3 -m http.server 9072 --bind 127.0.0.1 --directory tools/dashboard_merged
```

Open <http://127.0.0.1:9072/> and press **Connect** (default `ws://127.0.0.1:9001`).

## What it shows

- **Broker and car link.** Car online requires a heartbeat on `is28/hb` within
  2 s. Retained and duplicate heartbeats do not count as fresh. A last-will
  message marks the car offline.
- **Operating state comes from car telemetry**, never from command ACKs:
  - SDK: coordinator mode (source 11).
  - RTOS: navigation state (source 6).
  - Otherwise: the heartbeat state, labelled as such.

  The command panel separately shows what the car *accepted*. On SDK v2.3, an
  accepted Reset can leave the previous run's per-run Fault in the heartbeat
  and coordinator state until a new run starts. The page explains this instead
  of hiding it.
- **Firmware badge** (header). The firmware family is detected automatically
  from the packet types seen since the last connect or car restart:
  - RTOS-only sources: 3, 5, 6, 7.
  - SDK-only sources: 9–14.
  - The RTOS Buddy4 bench build sends only the shared sources 1/2/4/8. It is
    inferred as RTOS after 3 s of Buddy 2/4 frames with no SDK telemetry.
  - Frames from both families at once are flagged **MIXED**.
  - There is no manual override in v0.2. The SDK *drive* profile (1/2/3) is a
    separate thing and still appears in the motion panel.
- **RTOS Safety (source 7)** is shown only from fresh RTOS frames:
  - On SDK firmware it reads "not provided by SDK v2.3 firmware".
  - With mixed frames it reads "not trusted".
  - When stale it reads "stale — last state not shown". It never shows a stale
    or unsupported "Normal".
- **Buddy 2/Buddy 3 (SDK source 9/10/11/12/14):**
  - measured and target wheel speeds, PWM, raw ADC, calibrated line levels;
  - heading, distance, profile, phase, line recovery;
  - barcode and junction progress, coordinator events and diagnostics;
  - the v2.3 charts.
- **Buddy 4 (sources 2/4):**
  - IMU state, pitch, roll;
  - hump and rejection counters, impacts, I2C errors, missed periods;
  - odometry validity and quality diagnostics.
  - **LIVE right-minus-left wheel height** (green box, every source-2 frame)
    is kept apart from the **ACCEPTED completed-ramp height** (blue box, Buddy
    4's confirmed hump). A later live reading never replaces the accepted one.
- **Barcode A–Z mapping** with confirmed readback. An ACK alone never confirms
  the mapping.
- **Telemetry on/off.**
- **Stale data and restarts:**
  - Each live group goes stale after 2–3.5 s without frames.
  - A disconnect, last-will message or car restart clears live readings and the
    observed profile.
  - Recorded data is kept.
- **Commands from another client** on the broker (e.g. the 9071 page) are
  surfaced. Command IDs start at a random value to avoid ACK collisions.

The page sends nothing automatically except `BARCODE_GET`, the read-only
mapping query, exactly as both original dashboards do. Start, Resume, Reset and
all other motion commands are only ever sent by a button press.

## Exports (this page session; downloads to this laptop only)

| Button | Contents |
|---|---|
| Export telemetry CSV | v2.3 columns unchanged in order, plus `src, seq, laptop_ms, firmware_profile` |
| Export telemetry JSON | motion, events and Buddy4 rows, with buffer limits and drop counts |
| Export Buddy4 CSV | `live_right_minus_left_height_mm` vs `accepted_peak_mm` / `highest_accepted_mm` |
| Export event CSV / JSON | v2.3 columns plus `kind` (car event, Buddy 4, command, ACK), `src, seq, laptop_ms` |
| Export all frames CSV / JSON | every message: laptop ISO time, topic, retained flag, source/sequence, length, Pico uptime, run ID, profile, raw hex and decoded data |
| Save log (Messages panel) | the message log |

- Every export covers the whole retained page session. Reloading the page
  starts a new, empty session.
- **Bounded history**, with retained/dropped counts in the Exports panel:

  | Buffer | Limit |
  |---|---|
  | All MQTT messages | 150,000 |
  | SDK motion samples | 72,000 |
  | Buddy4 rows | 72,000 |
  | Events | 20,000 |
  | Message-log lines | 20,000 |

- **Stored-data downloads** (Live motion panel). The four SDK v2.3 stopped-only
  downloads (Diagnostics, Motion rows, Navigation rows, Raw sensor rows) are
  the v2.3 logic unchanged:
  - coordinator Off for 5 s;
  - per-row request IDs, source-13 reassembly and validation;
  - at most 3 retries per row;
  - no partial export.

  They are disabled when coordinator status is stale or the firmware is RTOS.
- **Clear visible** (Messages) only empties the on-screen log. There is no
  control that discards history; reloading the page does.
- Nothing is uploaded or sent anywhere.

## Tests (simulated data only)

```sh
sh tools/dashboard_merged/test/run_all.sh
```

- `test/test_merged_schema.js`:
  - Differential checks against pinned copies of both original decoders
    (`test/reference/`, hashes in `test/SOURCES.json`).
  - Units, signs, unavailable markers, 64-byte frames.
  - Profile detection.
  - Replay of real native captures (`test/fixtures/`).
- `test/ui_merged.mjs`: browser checks with a mock MQTT transport. It covers:
  - car-online freshness;
  - profiles and Safety handling;
  - live vs accepted height;
  - Reset/Fault;
  - restart and disconnect invalidation;
  - removal of the Recording panel and Firmware profile card;
  - the compact firmware badge and SDK drive-profile display;
  - whole-session exports;
  - stale coordinator;
  - that no motion command is ever sent automatically.
- `test/compat/`: the existing RTOS and v2.3 dashboard tests run against this
  page. They are changed only in lines marked `COMPAT` (paths, the mock route
  for the vendored MQTT.js, no screenshots).

These checks do not test a real broker, Wi-Fi, the Pico or the car.
