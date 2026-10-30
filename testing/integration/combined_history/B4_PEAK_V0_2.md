# Combined build v0.2 — ramp assist v0.3: hold through Buddy4 acceleration exclusions

Folder: `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_2/` (2026-10-31). Copy of `interface_wifi_v2_3_b4_peak_v0_1/` with one change; v0.1 and all earlier UF2s are preserved. Full diff vs v0.1: `B4_PEAK_V0_2.patch`; pre-change files in `archive/ra_v0_2/`. **Host-tested only.**

## Firmware

`bench_tools/buddy4/buddy1234_peak_v0_2_ra_active_bp_active.uf2`
sha256 `e16d2f100953dbcc2e6694051c539ce7edfe3cc7076d56aeffc5cbcf61e09c94` (ELF `aab494de171ad41da6afad0160de1543c22580a45c69c20101302814f399859e`), 1,038,336 bytes / 2,028 blocks, flash 0x10000000–0x1007ec00. Ramp assist ACTIVE, peak decoder ACTIVE. Contains local Wi-Fi credentials: keep out of Git.

## Why (run 20, 2026-10-31 00:39, 10 mm ramp, lower bracket)

Assist activated at 171 mm (+30/+28 PWM). At the crest (216–219 mm) the car slowed to 0 for ~0.15 s. Buddy4 marked tilt invalid in 60 ms bursts with source-2 quality reasons **4 (ACCEL)** and **5 (SPREAD|ACCEL)**; the 40 ms invalid rule ended the assist four times (`INVALID`), both integrals saturated, the car surged to ~80 mm/s and the hump was rejected (detail 3 = acceleration-exclusion budget exceeded, peak 13 mm). Not caused by the barcode decoder (no label seen; Buddy4/ramp code unchanged; same rejection seen before the peak decoder existed).

## Change (ramp assist v0.3, detector version 3)

- While **assisting**, a non-valid tilt sample whose **only** Buddy4 quality reasons are ACCEL and/or SPREAD, with Buddy4 tracking, **holds** the current assist (detector frozen) for up to **150 ms**; longer ends `INVALID`.
- Any other invalid reason (turn, reverse, impact, gravity, timing, unavailable, fault, idle) keeps the unchanged **40 ms** rule.
- Not assisting: such samples still block activation (unchanged).
- Unchanged: +20 % boost, 40 PWM cap, slew, 2.5 s timeout (hold time counts), 60 ms stale, Stop/Pause/fault/search/junction ends, activation, v0.2 level/descent detector, all controller limits, Buddy4 processing and hump acceptance rules, drive, barcode (original + peak), MQTT, calibration.
- Buddy4 service exports two observer copies in the tilt view (`tracking`, `quality_reasons`); no Buddy4 processing changed.
- Telemetry: ramp source-15 flags bits4–6 = **3** (dashboard "detector 3"); USB banner `BPEAK build buddy1234_peak v0.2 ...`.

## Host evidence (`B4_PEAK_HOST_TESTS_v0_2.txt`)

- Run-20-like crest (four 60 ms ACCEL/SPREAD gaps): v0.2 reference 5 activations / 4 INVALID ends; **v0.3 1 activation, full alpha held, ends on DESCENT**.
- Turn-invalid gaps still end after 40 ms; exclusion > 150 ms ends INVALID; skip→turn gap ends on 40 ms; exclusions never activate; Stop/Pause during a hold end immediately; Buddy4 stops → STALE; hold time counts toward the 2.5 s timeout; skip-bit constants checked against Buddy4 `B4_Q_*`.
- `run_combined_tests.sh` passes (cloud clang+gcc, Mac VM gcc): ramp assist 360,865 checks, peak suite, combined integration (active/shadow, ASan/UBSan), C packer→dashboard fixture, Buddy4 service + powered acceptance, `run_tests.py` 35.
- ARM: 0 warnings; bss 202,080 (+8 vs v0.1); stack main 2,368 B, control_step 872 B.

## Build

```sh
cp ../interface_wifi_v2_3/wifi_config.local.h .   # git-ignored
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w -DRAMP_ASSIST_ACTIVE=ON -DBARCODE_PEAK_ACTIVE=ON
cmake --build build
```

## Short hardware test (same setup as run 20)

1. Recorder running; flash; Reset and keep still 5 s; IMU (B4) must show Ready/Tracking (not Fault). Dashboard "Firmware build" shows detector 3.
2. One flat run: no assist, normal following.
3. 10 mm ramp, 1–2 runs. Expect one activation through the crest (no repeated `INVALID` ends), smaller crest slowdown and no surge; Stop if stalled 3 s. Check whether the hump is accepted (it can still be rejected by the estimator's own acceleration budget).
4. Send the capture.
