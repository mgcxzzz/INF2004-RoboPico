# Combined build v0.1 — v2.3 + Buddy4 v0.2 + ramp assist v0.2 + barcode peak decoder

Folder: `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_1/` (2026-10-30)

| Part | Source |
|---|---|
| Base | `interface_wifi_v2_3/` as committed in a647b2c: Buddy1 MQTT, Buddy2 drive, Buddy3 line/barcode/navigation, Buddy4 estimator v0.2, ramp-assist detector v0.2 |
| Barcode peak decoder | `interface_wifi_v2_3_barcode_peak_v0_1/` (`barcode_peak.h` is byte-identical, sha256 `e5c8949f…`) |

Both source folders are unchanged. All earlier UF2s are preserved in `bench_tools/buddy4`
(now 29). This build is **host-tested only; it has not been flashed or driven.**

## Firmware

`bench_tools/buddy4/buddy1234_peak_v0_1_ra_active_bp_active.uf2`

| Item | Value |
|---|---|
| UF2 sha256 | `26910f823893797e4fca3e5f68476765540ed64f1e5d88695bd7714bbbe4cd75` |
| ELF sha256 | `f464b36a5928cf13d00e9ad0f3087b8b3eb2ad5af3d73a26af2528eb9b03a3db` |
| UF2 | 1,037,824 bytes, 2,027 blocks; RP2040 family; flash 0x10000000–0x1007eb00 |
| Enabled modes | `RAMP_ASSIST_ACTIVE=1` (assist applied), `BARCODE_PEAK_ACTIVE=1` (peak letters may be queued), `BUDDY4_ADDON=1`, `BARCODE_PEAK_COMBINED=1` |
| Credentials | Contains the local Wi-Fi credentials from the git-ignored `wifi_config.local.h`. Keep it out of Git; none appear in the build logs. |

## Exact changes compared with `interface_wifi_v2_3/`

The full diff is in `B4_PEAK_V0_1.patch`.

1. **`v12_2/.../circuit_with_barcode/main.c`: 3 added lines, nothing else changed.**
   - `#include "../../../../../barcode_peak_harness.h"`, placed after `ramp_assist_harness.h`.
   - `barcode_peak_begin(now);` in `start_run()`, right after `barcode_analog_begin(...)`.
   - `barcode_peak_sample(side_adc,last_side_sample,wheel);` right after
     `barcode_analog_sample(...)`, inside the same AUTO-run GP27 sampling branch.
2. **`CMakeLists.txt`:**
   - adds the option `BARCODE_PEAK_ACTIVE` (default OFF = shadow);
   - output name `buddy1234_peak_v0_1_ra_<ramp>_bp_<peak>`;
   - compile definitions `BARCODE_PEAK_ACTIVE`, `BARCODE_PEAK_COMBINED=1`.
3. **New files:**
   - `barcode_peak.h` (identical decoder);
   - `barcode_peak_harness.h` (the standalone harness, plus the combined-build ID and the
     conflict cancel below);
   - `barcode_peak_tests/`, `barcode_peak_decode.py`, `source15_decode.py`;
   - `run_barcode_peak_tests.sh`, `run_combined_tests.sh`.
4. **Unchanged, byte-for-byte:**
   - `continuous_drive.h` (gains, feedforward, limits), `ramp_assist.h`, `ramp_assist_harness.h`;
   - `buddy4_harness.h` and `buddy4/`, `wifi_harness.h`, the transport and packets;
   - startup, buttons, stop controls, line recovery and encoders;
   - the original barcode reader and cross-junction logic, the runtime MQTT letter mapping
     and calibration.

   `CALIBRATION_CHANGES.patch` was not applied. The standalone peak `main.c`,
   `CMakeLists.txt` and Wi-Fi harness were **not** copied over.

## Coexistence with the original decoder (active build)

| Situation | Result |
|---|---|
| Original decoder misses the label, peak decoder reads it | Letter queued via `barcode_queue_code`, using the runtime mapping. `B_DECODE` (flag 0x40) produces the normal dashboard DECODED/QUEUED. The original junction logic then acts on it. |
| Both decoders read the same letter | Duplicate suppressed: one queue. Navigation and decoder state are bit-identical to the original decoder alone (tested). |
| Original decoder accepted a different letter first | **The pending movement is cancelled**, never silently kept. `B_ANALOG_CONFLICT` is logged and source-15 flags bit5 is set. |
| Peak decoder first, original decoder later reads a different letter | The original conflict rule cancels the pending movement. |
| Letter not in the mapping | Counted as unsupported; no movement queued; dashboard shows UNASSIGNED. |
| Turn or U-turn manoeuvre (`nav.busy`) | Peak state is reset, so there are no partial labels. |
| No barcode | Navigation and original decoder state are bit-identical with the peak harness running. Only one telemetry frame (the run-start announce) is sent. |

Queue and navigation ownership APIs are unchanged.

**Note:** with a *sharp* sensor (for example a future slit shroud), narrow and wide bars reach
the same height. The peak decoder then rejects the label (wide bar count / bar height
unclear) and the original decoder reads it. The coexistence rules cover this case.

## Telemetry coexistence (source 15)

| Layout | Frame | Identified by |
|---|---|---|
| Ramp assist v0.2 + Buddy4 estimator marker | 31 bytes (payload 28) | payload length 28; flags bits4–6 = detector 2, bit7 = estimator 2 |
| Barcode peak v0.1 | 27 bytes (payload 24) | payload length 24 **and** payload[4] = `0xB1` |

- Dispatch uses the validated header, length and subtype, never the source number alone.
  Anything else is rejected as an unknown layout.
- The two layouts keep independent sequence counters.
- Decoders that follow this rule:
  - dashboard `is28_schema.js` (`decodeSource15`);
  - `source15_decode.py` (both layouts in capture order);
  - `ramp_assist_decode.py` and `barcode_peak_decode.py` (each validates its own layout).

**Combined-build identifier.** It uses only fields of the peak frame itself, so no shared or
reserved diagnostic bits are touched (source 14 is unchanged):

- peak flags **bit7 = 1** in this build; the standalone peak build never sets it;
- one **run-start announce** frame (reason 10, bars 0) is sent at every Start, so the build is
  identifiable before any barcode;
- USB console: `BPEAK build buddy1234_peak v0.1: v2.3 + Buddy4 v0.2 + ramp assist active + peak decoder active`;
- the dashboard shows "combined Buddy1-4 + peak · peak v0.1 active · ramp active · detector 2 ·
  estimator 2".

## Dashboard (`tools/dashboard_merged`)

Changes are in `DASHBOARD_V0_3_SOURCE15.patch`; the previous page is in `archive/v0_2/`.

- Source 15 decoded by length and subtype. New cards: "Barcode peak decoder", "Ramp assist"
  and "Firmware build", plus timeline events for peak reads, conflicts and ramp
  activations/ends.
- The connectivity test counts the two source-15 streams separately.
- **"Last decoded letter (B3)"** updates from SDK source 10 (and the source 11 DECODED event).
  The movement comes from the confirmed mapping readback (source 8). This is verified with
  frames from the real C packers after a real (peak) decode: it shows `Y` / `Forward` and does
  not stay "no frames".
- RTOS decoding, restart and staleness handling are unchanged. A restart clears the new cards
  and makes the mapping unknown.

## Validation — HOST/REPLAY ONLY (`B4_PEAK_HOST_TESTS_v0_1.txt`)

**`sh run_combined_tests.sh`** passed in the cloud (clang + gcc) and on the Mac VM (gcc):

1. **Peak decoder suite:**
   - 4 real failed captures decode, including variants;
   - synthetic A–Z/0–9 in both directions: 930/936, 0 misreads;
   - harsh conditions: 0 misreads;
   - malformed and noise inputs: no decode;
   - telemetry.

   Also the existing v2.3 `run_tests.py` (35 executables).
2. **Integration** (real original decoder + runtime mapping + peak harness, active and
   shadow, ASan/UBSan):
   - apply, duplicate and both conflict orders;
   - unmapped letter, junction after a peak decode, no-barcode identity, busy reset;
   - mixed and interleaved source-15 streams.
3. **Reporting path:**
   - the real C packers produce a 124-frame fixture, identical when built with clang and gcc;
   - `source15_decode.py` decodes it with 0 unknown frames;
   - the dashboard node tests pass (25).
4. **Existing ramp-assist suite:** 360,849 checks.
5. **Existing Buddy4 suites:** the service test and powered acceptance (24 checks).

**Dashboard browser checks** (Playwright, simulated MQTT, cloud): `ui_b4_peak.mjs` 13/13 and
`ui_merged.mjs` 44/44 (including the RTOS R01–R08 checks).

**ARM build** (Mac VM, xPack arm-none-eabi-gcc 13.3.1, pico-sdk 98a542c, Release):

| Check | Result |
|---|---|
| Warnings / errors | 0 / 0 |
| RAM | text 528,992; bss 202,072 (+2,036 vs the integrated build: peak state 1,256 B + 768 B static scratch); `__bss_end__` 0x20030c58, heap limit 0x2003c000 |
| Stack (`-fstack-usage`) | main 2,368 B; `barcode_peak_sample` 592 B; `control_step` 864 B. The 8 KiB core-0 stack is unchanged. |

**Not hardware-validated:** combined timing on the car, real labels other than the four
captures, peak and ramp interaction on the track, and the junction distance after a label
(it must be ≥ ~12 mm after the last bar).

## Build

```sh
export PICO_SDK_PATH=/path/to/pico-sdk          # 98a542c1a62fb549ffb5d66a3e5892b06276b670
cp ../interface_wifi_v2_3/wifi_config.local.h .  # git-ignored; never commit
cmake -S . -B build-ra-on-bp-on -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w \
      -DRAMP_ASSIST_ACTIVE=ON -DBARCODE_PEAK_ACTIVE=ON
cmake --build build-ra-on-bp-on
sh run_combined_tests.sh                          # host checks (CC=gcc CFLAGS=-Wno-misleading-indentation on the VM)
```

## Short hardware test

**Power rule:** one source at a time. Flash on Mac USB; drive on the power bank only. Keep the
native recorder running. Lower bracket, original calibration, same mat.

1. **Flash and check (USB only, no Start):**
   - BOOTSEL, then copy the UF2 (check its sha256 first: `26910f82…`).
   - Press `h` and confirm the `CAL FRONT` line.
   - On 9072 press Connect and Read mapping. Map four letters including the test labels
     (e.g. Y forward, Z U-turn, A left, F right) and confirm the readback.
2. **Flat run (power bank), Start then Stop after 2–3 s.** Check:
   - "Firmware build" shows combined Buddy1-4 + peak · peak v0.1 active · ramp active ·
     detector 2 · estimator 2;
   - normal line following and no assist activation.
3. **Barcode runs (Z, then Y), each followed by its junction.** Check:
   - "Last decoded letter (B3)" and the mapped movement update;
   - the peak card shows `read Z → queued`, or "original decoder read it too";
   - the junction performs the mapped action.

   Stop on a wrong letter, a wrong turn or a CONFLICT that is followed by any movement.
4. **10 mm ramp run** as in the ramp-assist v0.2 tests. Check:
   - assist active until DESCENT/LEVEL/TIMEOUT;
   - Stop during assistance still stops;
   - an accepted Buddy4 height near 10–11 mm.
5. **Send the capture.** Decode it with:

   ```sh
   python3 source15_decode.py <capture> FROM TO
   ```
