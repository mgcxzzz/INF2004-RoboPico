# v2.3 + Buddy4 review — start here

Branch: `integration/v23-buddy4-review`. Baseline: **8d7f3aa4935565636bba013c74e563f5e8135bcb**, the team's original SDK Wi-Fi v2.3 (Buddy1 MQTT + Buddy2 odometry/drive + Buddy3 line/barcode/navigation).

**Current blocker: barcode decoding still fails on the car, including today's retries with the original v2.3 UF2. Neither the root cause nor a working barcode fix has been established.** The user reports that original v2.3 worked yesterday. The patched decoder passed some captured replays but did not fix the latest physical run. Do not describe that as solved, or conclude that the sensor is incapable from hand-positioned ADC readings.

This is an SDK integration/review snapshot, not a completed micro T-Kernel port. Buddy5 is outside this branch's integration scope.

## Which source to use

| Item | Location | Meaning |
|---|---|---|
| Original v2.3 | `git show 8d7f3aa:testing/subsystem2/interface_wifi_v2_3/<file>` | Immutable comparison point; published UF2 hash below |
| Current Buddy1+2+3+4 | `testing/subsystem2/interface_wifi_v2_3/` | Buddy4 estimator v0.2 + ramp-assist detector v0.2 on original v2.3. This is the integrated ramp/speed test build |
| Latest barcode experiment | `testing/subsystem2/interface_wifi_v2_3_barcode_fine_v0_3/` | Original v2.3 + barcode changes only. **No Buddy4/ramp assistance. Physical barcode test failed.** Keep separate while diagnosing |
| Earlier barcode changes | `testing/integration/barcode_history/` | v0.1/v0.2 patches, notes and build manifests; older local UF2s retained outside Git |
| Updated dashboard | `tools/dashboard_merged/` | v0.2 merged SDK/RTOS dashboard with bank63 decoder extension. Recording and manual firmware-profile card removed |
| Results and evidence | `testing/integration/V23_BUDDY4_V0_1_TEST_RESULTS.md`, `testing/integration/logs/` | Historical result ledger, decoded reviews, stored raw ADC CSVs, host/build records and bounded native MQTT excerpts |
| Exact differences | `testing/integration/V23_REVIEW_SOURCE_COMPARISON.json`, `V23_BASELINE_TO_*.patch` | Per-file hashes and patches against original v2.3 |

The main local `buddy4` checkout has an unrelated unfinished merge. It is preserved. This branch starts directly from original v2.3; it does not import that unfinished RTOS merge or teammate work from other branches.

## What changed from working v2.3

### Integrated Buddy4 build

Only four pre-existing files differ from the original source tree; the remaining baseline files are unchanged by hash:

- `CMakeLists.txt`: compile Buddy4 service/IMU/telemetry sources, version the UF2 name, select shadow or active ramp assistance. **Default is shadow**; active requires `-DRAMP_ASSIST_ACTIVE=ON`.
- `v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c`: add Buddy4 init/poll/reset and read-only odometry observation; add bounded ramp-assist hooks around the existing control step.
- `wifi_harness.h`: accepted Reset also resets Buddy4.
- `continuous_drive.h`: optional feedforward boost fields and addition. **This controller is therefore not byte-identical to v2.3 once ramp assistance was added.** With assistance off, parity tests compare it against frozen v2.3. Gains, nominal target speed, output limits, slew and anti-windup were not retuned.

New Buddy4 processing lives in `buddy4/`, `buddy4_harness.h`; assist in `ramp_assist*.h`/decoder/tests. Assistance is +20% of existing feedforward, capped at40 PWM per wheel, at most2.5s per ramp. Detector v0.2 uses median-of3 tilt samples and100ms level/descent persistence; invalid data and stop/navigation exclusions cancel it. Primary version markers: source15 `detector=2`, `estimator=2` (bit7 identifies estimator2).

Estimator v0.2 fixes quantised20ms heading falsely reporting a turn despite acceptable200ms average, and permits brief acceleration-only exclusions within existing100ms/25% budgets. Invalid samples do not contribute to peak/return evidence. Reversal, real sustained turn, stale odometry, timing, shock, gravity and Stop guards remain.

**Original barcode reader/acquisition files are unchanged in this integrated build.** The experimental barcode fixes below have not been folded into it.

### Barcode experiments — unsuccessful on hardware so far

- v0.1 appended204/205ADC hysteresis1 lanes, retaining original63 lanes. It recovered one saved Z scan in replay. Its diagnostic version marker incorrectly used a reserved source14 flag, so the real packet packer dropped diagnostics. This was an agent-introduced reporting bug, not proof of the barcode failure's cause.
- v0.2 removed that reserved flag, allowed actual winning bank63 through the packer/dashboard decoder, and identifies itself in periodic source11 STATUS detail **0xbc02 /48130**. Diagnostics then worked, but the physical scan still decoded nothing. One saved failed scan contains Z only at205ADC/hysteresis2, which the acquisition/adjacent-consensus setup missed.
- v0.3 changes the32 fine lanes to odd193..255ADC/hysteresis2, keeps coarse lanes and the204/205 hysteresis1 pair, and retains the original immediate neighbouring-vote path. A fallback accepts a single fully validated A-Z Code39 frame only after measured leading/trailing white quiet zones of10 learned narrow-module widths, no conflicting complete-frame vote, and age <=3s. It never inserts missing bars or assumes a letter. Storage version31 distinguishes the changed lane semantics; old calibration import is preserved. Build marker **0xbc03 /48131**. Fallback acceptance is delayed until sufficient trailing travel; it needs space before the next junction.
- v0.3 motor, steering/follow, encoder, motion/coordinator, transport and command implementations remain original v2.3 by hash. Full-frame character/width checks remain; only quiet-history tracking is added to that reader.
- Both earlier Z captures pass v0.3 replay, and synthetic A-Z both ways, noise, damaged-pattern, conflict/stale/quiet-zone tests pass. **The latest physical v0.3 scan still has0 complete candidates, even in a diagnostic threshold/hysteresis sweep.** Replay success was insufficient to establish real-world robustness. Y remains unresolved.

### Dashboard

Merged Buddy2/3 SDK and Buddy4/RTOS panels; automatic family badge, commands, mapping readback, messages, plain exports, and four stopped-only stored dataset downloads. Recording/video tools and manual profile card were removed at the user's request. SDK Safety is not presented as RTOS Safety. Live wheel height and accepted ramp height are distinct. The latest bounded decoder change accepts source14 bank63 but still rejects64+ and reserved flags. A “firmware SDK v2.3” family badge does **not** identify these exact release versions.

No calibration controls were added. Source15 assist details can be decoded offline with `ramp_assist_decode.py`; do not assume the merged UI displays every assist field. Some dashboard compatibility tests require the separate RTOS checkout; see test notes below.

## What hardware testing established

- Current integrated Buddy4 build crossed the measured10mm right-wheel ramp unaided twice and accepted heights **10mm and11mm**. These are two observations, not broad accuracy/repeatability validation.
- Dashboard Stop during active assistance cleared assist output and motor commands; a deliberately incomplete ramp is correctly rejected as Stopped.
- Approximate powered400mm/11.05s check: SDK404mm/11.309s. Useful sanity check; human timing/start intervals differ, and a midway slowdown occurred.
- Stuttering/line acquisition changed with mechanical IR adjustments; pure original v2.3 also reproduced symptoms. No definitive cause isolated. User reports “not much stuttering” in the latest v0.3 barcode run.

## Current barcode evidence and uncertainty

Times below are **2026-10-30 Asia/Singapore (UTC+8)**.

| Physical run | Telemetry/result | Replay finding |
|---|---|---|
| Original v2.3 earlier comparison ~05:04–05:05 | No decoded letter; no invalid encoder counts or reported line recovery | One earlier captured Z can be recovered at204/205 h1; Y cannot |
| Barcode v0.2 run3 ~05:33 | Verified0xbc02;230mm/5455ms;decoded0; no recovery/invalid encoders | Z at205ADC/hysteresis2 only; no adjacent-bank agreement |
| Barcode v0.3 run1 ~05:54 | Verified0xbc03;232mm/5344ms;decoded0; no recovery/invalid encoders; ADC169..2154 | Real current decoder0candidates. Sweep160..300ADC with hysteresis0/1/2/3/8/16 also0 full-frame matches |
| User's original-v2.3 retry ~06:17 |220mm/5455ms;decoded0; no recovery/invalid encoders. Untagged telemetry, preview212ADC, consistent with original. Exact flashed bytes not independently read back | Native excerpt and decoded review saved; no new raw download for this retry |

The on-disk original published UF2 was verified against the pinned Git artifact: SHA256 **cceab279c3dd99b1548e65a67bf5764fcffddf4c288dc5175d3fbdb4d15bdf5e**. Do not confuse it with our earlier local original-source rebuild (f846cd5e…): same source baseline is not necessarily the same binary.

Latest manual spot readings: white~170, narrow black190–210, narrow white gap also190–210. These suggest weak/overlapping contrast but are imprecise hand-positioned observations, **not proof of sensor incapability or a measured root cause**. Wide bars still produce much higher ADC. The sensor sees changes, but that does not establish that all29 elements are resolved.

Both original and patched firmware currently report0decoded, so the dashboard's empty letter is consistent with current car telemetry. There was a real v0.1 diagnostic-reporting bug, fixed in v0.2; that is separate from no valid barcode candidate. Do not claim the current failure was proved to be a dashboard, motor, print-dimension or hardware problem.

User confirms barcode dimensions are correct; do not repeat dimension checks. Z and Y have both failed. Hardware went through raised/lowered bracket, angle changes, original/spare left front IR and an optical divider during testing. The original left sensor was not proven faulty: neighbour on/off changed its reading dramatically; divider helped. **Optical interference at the barcode sensor itself has not been demonstrated.** Current lower bracket/original front preset is being used. Avoid speculative parameter changes or changing multiple physical variables at once.

Raw captures contain the first2048samples (~5.1s), not the entire longer run. Their truncation and path quantisation to0.01mm must be accounted for in replays. Do not claim a complete high-rate replay from50ms dashboard telemetry. Raw fixtures/notes are in the barcode candidate and integration logs; latest unresolved scan is `fixtures/barcode_Z_v03_failed.csv`.

## Hardware/configuration notes

- Pico W on RoboPico; all peripherals normally connected. One power source at a time: Mac USB to flash/calibrate; disconnect it before power-bank driving. SDK motion can reject/stop on USB-host state changes.
- Analog IR: leftGP26/ADC0, rightGP28/ADC2, barcodeGP27/ADC1. Current front preset white163/160, black3457/3700, profile1. Barcode acquisition is not calibrated through the front `w/l/r` procedure.
- IMU GY-511/LSM303, I2C1GP2/3; no gyro. Measured drive wheel centre spacing113mm. Ramp only raises the right drive wheel; test height10mm. Do not assume both wheels climb or the caster stays fixed on the floor.
- The low bracket/barcode IR can contact the mat at the crest; that contact and its contribution to drag are part of this test setup, not a demonstrated barcode root cause. Higher bracket improved clearance but changed optical response/line behaviour.
- Mapping is runtime MQTT configuration, not hard-coded. Z/P/O/S was the user's recent forward/U-turn/left/right mapping. An unmapped letter should still decode; mapping cannot explain decodedTotal0.

## Build and run locally

Use your own network settings; **no private configuration or newly built credential-bearing UF2 is published in this review**. Copy each source folder's `wifi_config.example.h` to ignored `wifi_config.local.h`, configure your2.4GHz Wi-Fi/broker, and use the installed Pico SDK/ARM toolchain. Wi-Fi saved in flash may override compiled settings; follow the existing setup notes. Never paste passwords in logs.

Example (Pico SDK path set in your environment; select your installed Make/Ninja generator):

```sh
cmake -S testing/subsystem2/interface_wifi_v2_3 -B build/review-b4 -DPICO_BOARD=pico_w -DRAMP_ASSIST_ACTIVE=ON
cmake --build build/review-b4 --parallel
# Separate barcode-only experiment:
cmake -S testing/subsystem2/interface_wifi_v2_3_barcode_fine_v0_3 -B build/review-barcode -DPICO_BOARD=pico_w
cmake --build build/review-barcode --parallel
```

Versioned outputs: `buddy123_interface_wifi_v2_3_b4_v0_2_ra_v0_2_active.uf2` and `buddy123_interface_wifi_v2_3_barcode_fine_v0_3.uf2`. The versions must not be mixed up. Local historical hashes/build manifests are records, not binaries supplied to your network.

Dashboard (reuse one Mosquitto broker with TCP1883/WebSocket9001):

```sh
python3 -m http.server 9072 --bind 127.0.0.1 --directory tools/dashboard_merged
```

Open http://127.0.0.1:9072/; default broker `ws://127.0.0.1:9001`. Car must reach the broker's LAN address;127.0.0.1 is browser-to-local-broker only. Export before reloading if needed. No commands are sent automatically on page load. Saved local settings may need updating for your network.

## Review/test entry points

Run the self-contained source-package checks from the repository root:

```sh
sh tools/review/run_checks.sh
```

This checks integrated v2.3/ramp assistance, Buddy4 service/estimator, barcode candidate and the two merged decoder suites, without connecting to the car. It passed on the Mac; output is `testing/integration/logs/2026-10-30_integration_review_package_checks.txt`. Host replay/test success does not clear the physical barcode blocker.

The isolated barcode folder omits inherited generated baseline metadata (`BUILD_METRICS.json`, `SOURCE_SHA256.json`, `firmware/SHA256SUMS.txt`); this is explicit in the comparison JSON's missing list. Its own `BARCODE_FINE_V0_3_MANIFEST.json` describes its source/build. No runtime source is missing.


- From barcode folder: `CC=clang python3 run_tests.py` (requires Node on PATH);36 C executables + schema/Python tests. `test_barcode_fine.c` contains positive earlier captures and directed negatives; latest physical failure is retained explicitly as unresolved, not silently reported as passing.
- From integrated folder: `sh run_ramp_assist_tests.sh`; `buddy4/host/run_host_test.sh <outdir> <absolute path to tools/dashboard_merged/is28_schema.js>`; focused estimator test `buddy4/host/test_powered_acceptance.c` includes the real processing unit.
- From root: `node --test tools/dashboard_merged/test/test_merged_schema.js tools/dashboard_merged/test/compat/v23_test_schema.js`. The full historical `run_all.sh` also includes RTOS compatibility checks needing separate RTOS sources; browser suites need Playwright. Historical Claude browser results came from simulated data in its workspace, not a live-car test here.
- Comparison patches, source hashes, native excerpts and saved ADC scans are the primary review inputs. Full continuously growing native capture stays local; bounded excerpts and a provenance manifest are included.

**Suggested next investigation:** reproduce against the exact original binary with the current hardware (the latest retry is already negative), inspect raw element acquisition and its variation between runs, then isolate optical geometry/coupling, power/drive effects and sampling separately. Do not keep broadening acceptance rules to make one recorded Z pass; preserve malformed/noise/ambiguous rejection and test new captures not used to tune the change. Full A-Z both-way physical reads, reliable pending-letter-to-junction behaviour and subsequent merge of any proven barcode fix into Buddy4 remain outstanding.
