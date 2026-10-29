# Merged dashboard v0.1 — implementation and test record (2026-10-29)

**Location.** `tools/dashboard_merged/` (new). The usage guide is
`tools/dashboard_merged/README.md`.

**What it merges.**

- The RTOS Buddy1/Buddy4 dashboard (`tools/dashboard`, served on 9071).
- The SDK v2.3 Buddy2/Buddy3 dashboard
  (`testing/subsystem2/interface_wifi_v2_3/dashboard` at commit 8d7f3aa, in
  the `INF2004-RoboPico-v23b4` worktree).

**Scope.** Both originals are unchanged; their hashes before and after are
listed below. Dashboard work only: no firmware, calibration, motor-control or
protocol change. The work was done without git operations: nothing was
checked out, staged, committed or pushed. The unfinished merge in this
checkout is untouched.

## Design summary

- **Decoder.** `is28_schema.js` is the union of the two existing decoders.
  - Function bodies are copied; only the colliding name was changed (RTOS
    source-2 `decodeMotion` became `decodeB2Motion`; SDK source 9 keeps
    `decodeMotion`).
  - Sources 1–14 are supported, including the SDK + Buddy4 add-on sources 2/4.
  - SDK frames of up to 64 bytes are accepted whole.
  - The only deliberate label difference: command 11 is named `GET_LOG` (the
    SDK name) instead of the RTOS placeholder `code11`.
- **Firmware profile.** It is set by the observed packet types since the last
  connect or car restart:
  - RTOS-only sources 3, 5, 6, 7.
  - SDK-only sources 9–14.
  - RTOS inferred from Buddy 2/4 frames with no SDK telemetry for 3 s or more.
  - Both families at once are flagged MIXED.
  - A manual override is available.
- **Safety.** Shown only from fresh RTOS frames. Otherwise it reads "not
  provided" (SDK), "not trusted" (mixed) or "stale". It never shows a stale or
  unsupported "Normal".
- **State vs acceptance.** Command acceptance (ACK) is shown separately from
  the car-confirmed state (coordinator, navigation or heartbeat telemetry).
  When v2.3 accepts a Reset but keeps the previous run's per-run Fault
  (`wifi_state()` returns 6 while `circuit.fault` is set), the page says so.
- **Freshness and invalidation.**
  - Car online needs a non-retained, non-duplicate heartbeat less than 2 s old.
  - Live groups go stale after 2–3.5 s.
  - A disconnect, last-will message or restart clears live values but keeps
    the recordings.
  - The stored-data download is disabled while coordinator status is stale.
- **No automatic motion commands.** Only `BARCODE_GET` is sent automatically,
  as in both originals.
- **Recording.** Run label, timestamped markers, video sync clock, bounded
  buffers with drop counts, the original exports plus event JSON, Buddy4 CSV
  and all-frames CSV/JSON, and a one-click evidence set whose manifest carries
  SHA-256 hashes.
  - **Clear visible** only empties the on-screen log.
  - Discarding recordings is a separate two-click action.
- **Hosting.** `python3 -m http.server 9072 --bind 127.0.0.1 --directory tools/dashboard_merged`,
  using the existing broker on 1883 (TCP) and 9001 (WebSocket). The 9071 page
  and the native recorder are unaffected.

## Tests (simulated messages only; no broker, Pico or car commanded)

| Suite | Where run | Result |
|---|---|---|
| `test/test_merged_schema.js` | Mac VM and cloud | 9/9 |
| `test/compat/v23_test_schema.js` (original v2.3 schema tests) | Mac VM | 10/10 |
| `test/compat/rtos_check_speed_dashboard.mjs` (INT-07, source-2 panel, DOM stub) | Mac VM | 62 assertions PASS |
| `test/compat/rtos_check_dashboard.mjs` (C-packed vectors from `test_b1`) | Mac VM | 20/20 |
| `test/compat/rtos_check_imu_dashboard.py` (INT-03, production C packer + panel) | Mac VM | 36 + 10 PASS |
| `test/ui_merged.mjs` (new browser checks, mock MQTT) | cloud Chromium | 44/44 (also passed on a second run) |
| `test/compat/v23_ui_smoke.mjs` (original v2.3 browser smoke incl. 4-file stored-download, retries, rejection) | cloud Chromium | PASS, 0 page errors |
| `test/compat/rtos_check_dashboard_ui.mjs` (original S1-U19) | cloud Chromium | 16/16 |

Notes on the suites:

- **`test_merged_schema.js`:**
  - Over 4,000 differential cases against pinned copies of both originals.
  - The 64-byte frame and unit/sign/0xffff checks.
  - Profile detection.
  - Real capture replay: SDK v2.3 + Buddy4 segment (1,762 frames, sources
    1/2/4/8/9/10/11/12/14, 0 errors, 0 mismatches) and RTOS v0.9 segment
    (1,498 frames, 0 errors, inferred RTOS).
- **INT-03 on the original page.** The original `tools/dashboard` page
  currently fails INT-03's panel step (`imuSpeed` initial text lacks
  "unavailable"). This failure was there before this work. The merged page
  passes it, and the original was left unchanged.
- **`ui_merged.mjs` covers:**
  - retained and fresh heartbeats;
  - the SDK profile with Safety "not provided";
  - live vs accepted height;
  - START acceptance vs state;
  - the v2.3 Reset/Fault note;
  - MIXED Safety not shown as Normal;
  - restart and last-will invalidation that keeps recordings;
  - label sanitising, markers, the 64-byte frame, other-client commands;
  - Clear visible;
  - telemetry/event/frames/Buddy4 exports;
  - the 7-file evidence set with SHA-256 verified;
  - two-click discard;
  - stale coordinator disabling downloads;
  - only GET sent automatically;
  - RTOS stale Safety, inference, mobile layout, 0 console errors.
- **Compat copies.** The compat tests are the originals with paths, the mock
  route (vendored MQTT.js instead of the unpkg CDN) and screenshot lines
  changed. Every changed line is marked `COMPAT`. No screenshots were taken.

Logs:

- `logs/2026-10-29_dashboard_merged_v0_1_host_tests.txt`
- `logs/2026-10-29_dashboard_merged_v0_1_browser_tests.txt`

## Originals unchanged (sha256, first 16)

| File | sha256 |
|---|---|
| `tools/dashboard/is28_schema.js` | e194e8aacb41f38c |
| `tools/dashboard/index.html` | 39983a2facd0f02c |
| v2.3 `dashboard/is28_schema.js` | ae24305ebe4e35d5 |
| v2.3 `dashboard/index.html` | 21dbaaf3be8946a6 |

## Not verified

- Live operation against the real broker, Pico and car.
- Browser download prompts on the Mac (Chrome may ask once to allow the
  multi-file evidence set).
- The 9072 server: it was not started from here.
