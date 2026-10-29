# IS28 merged dashboard v0.1

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
- **Firmware profile.** It is detected from the packet types seen since the
  last connect or car restart:
  - RTOS-only sources: 3, 5, 6, 7.
  - SDK-only sources: 9–14.
  - The RTOS Buddy4 bench build sends only the shared sources 1/2/4/8. It is
    inferred as RTOS after 3 s of Buddy 2/4 frames with no SDK telemetry,
    because the SDK Buddy4 add-on's frames are gated by the same switch as its
    20 Hz source 9/10 telemetry.
  - Frames from both families at once are flagged **MIXED**.
  - A manual override (Auto / RTOS / SDK v2.3) is available.
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

## Recording (laptop only — never sends anything to the car)

- **Run label.** It is sanitised to `A–Z a–z 0–9 . _ -`, and every row is
  tagged with the label active when the row arrives.
- **Mark event.** Records the laptop ISO time, an estimate of Pico uptime, the
  last sequence number per source, the latest SDK run ID and an optional note.
- **Video sync clock** (laptop time with milliseconds) to film at the start of a
  take.
- **Bounded buffers**, with retained/dropped counts and a truncation warning:

  | Buffer | Limit |
  |---|---|
  | All MQTT messages | 150,000 |
  | SDK motion samples | 72,000 |
  | Buddy4 rows | 72,000 |
  | Events | 20,000 |
  | Message-log lines | 20,000 |
  | Markers | 5,000 |

- **Exports** (scope: current run label, or the whole session):

  | Button | Contents |
  |---|---|
  | Export telemetry CSV | v2.3 columns unchanged in order, plus `src, seq, laptop_ms, run_label, firmware_profile` |
  | Export telemetry JSON | motion, events, Buddy4 rows and markers, with limits and drop counts |
  | Export Buddy4 CSV | `live_right_minus_left_height_mm` vs `accepted_peak_mm` / `highest_accepted_mm` |
  | Export event CSV / JSON | v2.3 columns plus `kind` (car event, Buddy 4, command, ACK, marker), `src, seq, run_label, marker_no, note` |
  | Export all frames CSV / JSON | every message: laptop ISO time, topic, retained flag, source/sequence, length, Pico uptime, run ID, run label, profile, raw hex and decoded data |
  | Save log | the message log |
  | Export evidence set | one-click bundle: telemetry CSV, Buddy4 CSV, events CSV/JSON, frames JSON, message log, and a manifest with counts, drops, markers and SHA-256 of every file |

- **Stored-data downloads.** The four stopped-only downloads (Diagnostics,
  Motion rows, Navigation rows, Raw sensor rows) are the v2.3 logic unchanged:
  - coordinator Off for 5 s;
  - per-row request IDs, source-13 reassembly and validation;
  - at most 3 retries per row;
  - no partial export.

  They are disabled when coordinator status is stale or the profile is RTOS.
- **Clear visible** only empties the on-screen log. Discarding recordings is a
  separate, two-click action that reports exactly what it removed.

## Filming one run for a teammate

1. Connect and confirm **car: online**. Apply the mapping and wait for "Confirmed by car". Switch **Telemetry on**.
2. Type a run label, e.g. `ramp10-run1`, and press **Set label**.
3. Start filming. Hold the camera on the **sync clock** for about 2 s, press **Mark event** with the note `video start`, then frame the car.
4. Press **Start**. Press **Mark event** at moments worth finding later: ramp, junction, fault.
5. End the run. Mark `run end` and stop filming.
6. SDK v2.3 only: wait for coordinator **Off** for 5 s. Then download the four stored datasets.
7. Press **Export evidence set**, scope "Current run label". Keep the files with the video. The manifest's markers and SHA-256 hashes tie them together.

Nothing is uploaded or sent anywhere. Share the files yourself.

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
  - recording, exports and the evidence-set hashes;
  - two-click discard;
  - stale coordinator;
  - that no motion command is ever sent automatically.
- `test/compat/`: the existing RTOS and v2.3 dashboard tests run against this
  page. They are changed only in lines marked `COMPAT` (paths, the mock route
  for the vendored MQTT.js, no screenshots).

These checks do not test a real broker, Wi-Fi, the Pico or the car.
