# Integration review (SDK v2.3 + Buddy 4) — start here

Branch: `integration/v23-buddy4-review`. Baseline: **8d7f3aa** — the team's SDK Wi-Fi v2.3 (Buddy 1 MQTT, Buddy 2 odometry/drive, Buddy 3 line following/barcode/navigation).

This is an SDK integration branch, not the micro T-Kernel port. Buddy 5 (obstacle) is not part of this branch.

## Current status (2026-10-31)

The current integrated firmware is **combined v0.5** in [`testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5/`](testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5/): v2.3 + Buddy 4 IMU estimator v0.2 + ramp assist v0.6 + barcode peak decoder v0.1.

| Feature | Status on the car | Evidence |
|---|---|---|
| Line following, junctions, MQTT, dashboard (Buddy 1/2/3) | Working, unchanged from v2.3 apart from the hooks listed below | Every run below |
| Barcode reading (peak decoder alongside the original decoder) | **Working.** Labels the original decoder had stopped reading decode again (standalone build, 2026-10-30: all test labels; combined build: Y read 2026-10-31 02:38) | `..._v0_5_speed_hold_runs_native_excerpt.txt`, standalone folder README |
| Ramp detection (Buddy 4 side tilt) | **Working.** 10 mm ramp detected on every run; Buddy 4 accepted the hump at 12 / 11 / 13 mm | same excerpt |
| Ramp crossing (ramp assist + speed hold) | **Working.** 3/3 runs crossed without stalling; earlier versions stalled at the crest or on the descent | same excerpt; history below |

## Which source to use

| Item | Location | Meaning |
|---|---|---|
| **Current integrated build** | `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5/` | Build this. Notes: `B4_PEAK_V0_5.md`; diff vs previous version: `B4_PEAK_V0_5.patch` |
| Standalone peak decoder | `testing/subsystem2/interface_wifi_v2_3_barcode_peak_v0_1/` | Original v2.3 + peak decoder only (no Buddy 4). Best place to read how the decoder works (README with plots) |
| Combined build history | `testing/integration/combined_history/` | Notes, patches, manifests and host-test logs of combined v0.1–v0.4 (each patch is against the previous version) |
| Previous Buddy 4 build | `testing/subsystem2/interface_wifi_v2_3/` | v2.3 + Buddy 4 + ramp assist v0.2, no peak decoder. Superseded by combined v0.5 |
| Earlier barcode experiments | `testing/subsystem2/interface_wifi_v2_3_barcode_fine_v0_3/`, `testing/integration/barcode_history/` | Threshold-based attempts (fine v0.1–v0.3); did not fix the car. Superseded by the peak decoder |
| Original v2.3 | `git show 8d7f3aa:testing/subsystem2/interface_wifi_v2_3/<file>` | Comparison point |
| Dashboard | `tools/dashboard_merged/` | Merged SDK/RTOS dashboard; v0.3 adds the source-15 ramp/peak/build cards |
| Results and logs | `testing/integration/V23_BUDDY4_V0_1_TEST_RESULTS.md`, `testing/integration/logs/` | Result ledger (newest at the end) and bounded native MQTT excerpts |

## What we fixed during integration

### 1. Barcode not reading (Buddy 3)

- **Problem:** after mechanical changes, the original decoder read nothing, even with the original v2.3 UF2. The original decoder needs one ADC threshold that separates every bar from every gap.
- **Cause:** the barcode IR (GP27) gives white ~175, narrow bar 204–225 (a small bump), gap next to a narrow bar 190–209, wide bar 1,700–2,300. Narrow bars and gaps overlap, so no threshold works; earlier the margin was only ~10–15 ADC and a small height/tilt change removed it. The threshold-tuning experiments (fine v0.1–v0.3) could not fix this.
- **Fix:** `barcode_peak.h` finds bars as **local peaks** (positions), classifies bars by height relative to the tallest bar and spaces by bar-centre spacing in the label's own narrow width, and accepts only a complete Code39 `*X*` frame in either direction. It runs **alongside** the unchanged original decoder: the same letter from both = one queued movement; different letters = the pending movement is cancelled (never silently chosen).
- **Integration:** 3 lines in `main.c` (include, `barcode_peak_begin()` at Start, `barcode_peak_sample()` after the original sample); letters go through the normal `barcode_queue_code()` path, so the runtime MQTT mapping and junction logic are unchanged.

### 2. Car stalling on the ramp (Buddy 2 drive + Buddy 4)

Ramp assist uses the Buddy 4 side-tilt estimate to detect the one-sided ramp and adds help in the existing continuous speed controller (`continuous_drive.h`). Each step came from a recorded failure:

| Ramp assist | Failure seen with the previous version | Change |
|---|---|---|
| v0.2 | Assist (v0.1) ended 241 ms into the climb on tilt noise | Median-of-3 tilt, 100 ms persistence for level/descent |
| v0.3 (combined v0.2) | Assist cut out at the crest when Buddy 4 flagged ACCEL/SPREAD for 60 ms | Hold through ACCEL/SPREAD-only exclusions up to 150 ms |
| v0.4 (combined v0.3) | Stuck at the crest for 20 s at only 295/165 PWM | Stall push: +60 PWM on both wheels when both stop for 200 ms |
| v0.5 (combined v0.4) | Ramp wheel starved: steering asked it for 2–9 mm/s (12–51 PWM) because the tilted car saw the line off-centre | Fixed 300 PWM floor per wheel — climbed at 107–131 mm/s, lost the floor on a 40 ms invalid sample, then stalled at the crest; Buddy 4 rejected every hump |
| **v0.6 (combined v0.5, current)** | Fixed PWM too fast, then lost | **Speed hold** instead of fixed PWM: until the tilt is flat for 200 ms, minimum speed target 35 mm/s per wheel, integral limit 300 (normal 100), 3× integral gain; survives invalid tilt < 500 ms; 5 s cap. The speed loop adds what is needed (≤ 420 PWM) and the climb stays near 45 mm/s |

Stop / Pause / ESTOP / line search / junction remove all ramp help at once. The 0–420 PWM clamp and 1200/s slew are never changed. With all ramp fields zero the controller is bit-identical to v2.3 (host test).

Mechanical: the barcode bracket was tilted slightly up for clearance; the front still touches the mat at the crest (barcode ADC rises to ~1,000), which the speed hold now pushes through. Raising the bracket further made barcode reading worse.

### 3. Buddy 4 estimator v0.2 (earlier in this branch)

Fixed quantised 20 ms heading falsely reporting a turn, and allowed brief acceleration-only exclusions within the existing budgets. Accepted ramp heights 10–16 mm on the 10 mm ramp across the integration runs (12 / 11 / 13 mm on the current build).

### 4. Dashboard / telemetry

Source 15 now carries two layouts (ramp assist 28-byte payload, peak decoder 24-byte payload with subtype `0xB1`), dispatched by length and subtype. The dashboard shows "Last decoded letter (B3)", the peak-decoder card, the ramp-assist card and a "Firmware build" card (combined build, detector 6 = ramp assist v0.6). No reserved diagnostic bits are used.

## Files changed vs v2.3 (current build)

Existing files: `main.c` (Buddy 4 init/poll/reset, ramp-assist hooks, 3 peak-decoder lines), `CMakeLists.txt` (sources, options, versioned output name), `wifi_harness.h` (Reset also resets Buddy 4), `continuous_drive.h` (ramp fields: `boost_frac/cap`, `boost_add`, `hold_min_speed/hold_i_max/hold_ki`).

New files: `buddy4/`, `buddy4_harness.h`, `ramp_assist.h`, `ramp_assist_harness.h`, `barcode_peak.h`, `barcode_peak_harness.h`, decoders (`ramp_assist_decode.py`, `barcode_peak_decode.py`, `source15_decode.py`) and tests (`ramp_assist_tests/`, `barcode_peak_tests/`, `run_combined_tests.sh`).

Unchanged: gains/feedforward/target speed, encoders, steering, line recovery, junction logic, original barcode reader, MQTT command/mapping protocol, calibration (`CALIBRATION_CHANGES.patch` not applied).

## Known limitations

- While the speed hold is on (ramp only), neither wheel drops below 35 mm/s, so steering is limited; in one run the car corrected slowly just after the descent but kept the line.
- Ramp constants were tuned on one car and one 10 mm right-wheel ramp.
- Barcode: the peak decoder needs a soft (blurred) sensor response; with a very sharp sensor narrow and wide bars reach the same height, the peak decoder rejects the label and only the original decoder can read it. Full A–Z on the car and the junction action after a peak-decoded letter on the combined build have not been recorded (the 02:38 Y was unmapped).

## Build and test

Each source folder needs a git-ignored `wifi_config.local.h` (copy `wifi_config.example.h`). Built UF2s contain Wi-Fi credentials — never commit them.

```sh
cd testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w   # ramp assist + peak decoder ON by default
cmake --build build      # -> build/buddy1234_peak_v0_5_ra_active_bp_active.uf2
```

Host checks (no Pico, broker or motors), from the repository root:

```sh
sh tools/review/run_checks.sh
```

Runs the combined suite (peak decoder, real-decoder integration, C packers → dashboard decoders, ramp assist 360,906 checks, Buddy 4 service/estimator) and the standalone peak decoder suite.

Dashboard: `python3 -m http.server 9072 --bind 127.0.0.1 --directory tools/dashboard_merged`, open http://127.0.0.1:9072/ (broker `ws://127.0.0.1:9001`). Hardware rules: one power source at a time (Mac USB to flash, power bank to drive).

## Not in Git

UF2/ELF images (credentials), `wifi_config.local.h`, full raw MQTT captures (bounded excerpts are in `testing/integration/logs/`), and the local intermediate source folders for combined v0.1–v0.4 (their patches and notes are in `combined_history/`).
