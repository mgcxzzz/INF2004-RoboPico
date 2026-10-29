# Merged dashboard v0.2 — simplification record (2026-10-30)

v0.2 supersedes v0.1 (`DASHBOARD_MERGED_V0_1.md`). It is a dashboard-only
change, made at the user's request (handoff
`HANDOFF_2026-10-30_DASHBOARD_SIMPLIFY.md`).

- No firmware, UF2, calibration, MQTT protocol, motor, PWM or steering change.
- No git operations. The buddy4 unfinished merge is untouched.
- The original dashboards (`tools/dashboard`, v2.3 `dashboard`), the test
  references and the fixtures are unchanged.
- The same URL, http://127.0.0.1:9072/, is served by the static server already
  running. The broker is unchanged at 1883/9001.

## Changes from v0.1

| Area | v0.1 | v0.2 |
|---|---|---|
| Recording panel | run label, markers, video sync clock, export-scope selector, 7-file evidence bundle with SHA-256 manifest, two-click buffer discard, filming steps | **Removed** (DOM, bindings, 100 ms clock/status timer, label/marker/scope/evidence/discard code). |
| Header | `recPill` recording badge | Removed. `profilePill` firmware badge kept; its tooltip lists the observed sources. |
| Firmware profile card | detected text, Auto/RTOS/SDK override, packet-source list | **Removed.** Detection is still automatic (`detectProfile` in the schema, Auto semantics), and Safety, stale, restart and MIXED handling are unchanged. |
| Exports | inside Recording; label/scope filtering; label in filenames and columns | New **Exports** panel. It contains telemetry CSV/JSON, Buddy4 CSV, event CSV/JSON and all-frames CSV/JSON, each covering the whole retained page session. Buffer status (retained/dropped) moved here, and `archiveStatus` was relocated here. The `run_label`, `marker_no` and `note` columns were removed; all other columns are unchanged. |
| Stored data | four SDK stopped-only downloads | Unchanged (Live motion panel). Filenames no longer carry a label: `is28_<dataset>_<stamp>.*`, as in the original v2.3. |
| Message log | Clear visible / Save log | Unchanged. Clear visible still only empties the view. No control discards history. |
| SDK drive profile 1/2/3 | motion panel | Unchanged (distinct from firmware family). |

The v0.1 files are preserved in `tools/dashboard_merged/archive/v0_1/` (the
folder is untracked, so this is the only copy). Its index.html sha256 is
`9776562b6e3a6004…`; v0.2 index.html is `d48d2cc087f59b6e…`.

## Tests (simulated MQTT only; the real broker and car were not used)

| Suite | Where | Result |
|---|---|---|
| `test/test_merged_schema.js` (differential vs both originals, 64-byte, units, profile detection, real-capture replay) | Mac VM, cloud | 9/9 |
| `compat/v23_test_schema.js` | Mac VM | 10/10 |
| `compat/rtos_check_speed_dashboard.mjs` (INT-07) | Mac VM, cloud | 62 PASS |
| `compat/rtos_check_dashboard.mjs` (S1 C vectors) | Mac VM, cloud | 20/20 |
| `compat/rtos_check_imu_dashboard.py` (INT-03, C packer + panel) | Mac VM | 36 + 10 PASS |
| `test/ui_merged.mjs` (updated to the v0.2 spec) | cloud Chromium | 44/44 |
| `compat/v23_ui_smoke.mjs` (incl. 4-file stored downloads, retries, rejection) | cloud Chromium | PASS, 0 page errors |
| `compat/rtos_check_dashboard_ui.mjs` (S1-U19) | cloud Chromium | 16/16 |

**What `ui_merged.mjs` changed.** The v0.1 checks for label, markers, evidence
bundle, scope and discard were replaced. The new checks assert:

- the removed controls are absent from the DOM (not hidden);
- the badge and its tooltip are present;
- the SDK drive profile is shown;
- the buffer status is shown;
- the exports cover the whole session, with exact column headers.

All live/stale, SDK vs RTOS Safety, MIXED, restart and last-will, mapping,
live vs accepted height, Reset/Fault, other-client and no-automatic-motion
checks are kept.

**Environment limits.**

- Playwright is not installed on the Mac VM, so browser suites ran in the cloud
  workspace on byte-identical files (per-file sha256 compared).
- No screenshots were taken.

**Logs.**

- `logs/2026-10-30_dashboard_merged_v0_2_host_tests.txt`
- `logs/2026-10-30_dashboard_merged_v0_2_browser_tests.txt`
