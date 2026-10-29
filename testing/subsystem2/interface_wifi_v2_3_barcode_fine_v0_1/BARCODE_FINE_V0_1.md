# Barcode fine v0.1 — isolated test candidate

## Why this build exists

Y and Z did not decode with the Buddy4 add-on, the local original-source rebuild, or the exact published v2.3 UF2. Barcode dimensions are accepted as correct per the user. The exact physical cause of the changed behaviour is not established; do not claim a specific bracket, light, sensor or mat change caused it.

Replaying the recorded Z scan with the original Code39 reader recovers Z at two neighbouring ADC thresholds,204 and205, with hysteresis1 and unchanged2ms debounce. The original fine banks use hysteresis3 and2ADC spacing. This is an observed software detection-margin limitation for this Z scan. The Y capture still does not decode; this build does not claim to fix every barcode or diagnose the whole stutter problem.

## Scope

Based on frozen original SDK v2.3 source, commit8d7f3aa4935565636bba013c74e563f5e8135bcb. No Buddy4 or ramp assistance is compiled into this isolated barcode candidate. Preserve the working Buddy4 estimator UF2 for return later.

- All63 original banks retain thresholds, hysteresis, default preview bank11/212ADC, debounce and reader behaviour.
- Add exactly2 banks: indices63/64 at204/205ADC, hysteresis1. Require agreement between these two on complete character, span, direction and time. Do not pair the end of the legacy family with the new family.
- All Code39 start/stop,29-element count, width/separation checks, duplicate suppression and conflicting-result rejection remain unchanged. No letter is hard-coded and no missing edges are inferred. Other characters can use this pair too if their complete pattern passes.
- Bank63 is the only new winning index and fits the existing6-bit selected-bank encoding. Default and coarse legacy banks remain unchanged.
- Source14 flags bit7 marks this build; USB `h` banner includes `BARCODE FINE v0.1`. Actual live threshold and bank continue to be transmitted normally.
- Record version30 distinguishes saved traces containing the extra-bank semantics. The candidate imports valid version28 calibration and preserves valid version30 calibration. Older original firmware rejects version30 logs and restores its original preset on return; current testing uses that same preset. Wi-Fi/broker WCF1 sector is separate and preserved.
- Motor controller, line-following targets/logic, coordinator execution, encoder capture, transport, commands and packet layouts are unchanged by SHA comparison. No stutter fix is claimed.

## Validation

- Captured Z:1 decoded Z,2 reader candidates,1 consensus selection, new bank63.
- Captured Y:0 decoded; retained as an unresolved negative fixture.
- Rejects one-bank-only votes, cross-family pseudo-pairs, conflicting letters, duplicate frames and40seconds of deterministic near-threshold noise.
- Tests record30 CRC validation and saved calibration behaviour.
- ASan/UBSan passes; original35C executables, JS schema/vectors and Python setup/network suites pass.
- Existing sensitivity test still validates every original coarse threshold and the original fine family; its bank-count/sorting assertions explicitly account for the appended pair. Frozen-source allowlist adds only that changed test file.
- Mac ARM Release build:0warnings/0errors. text511408bytes, BSS199588bytes. Separate8KiB core stacks verified nonoverlapping with BSS below them. Largest compiler frames:main2360,control_step1176bytes (not a complete runtime stack watermark).
- UF2 magic/blocks/family checked. Local credential-bearing build; never commit firmware or local config. No code was flashed and no motion command sent by agent.

## Artifact

`/Users/skyler/SIT/Y2T1/Embedded/bench_tools/buddy2/buddy123_interface_wifi_v2_3_barcode_fine_v0_1.uf2`

SHA256:`b362fe91ce6d3613cfe39468b9f218e042e343f82491c80907434a289565a153`

Source/build manifest:BARCODE_FINE_V0_1_MANIFEST.json. Build/test records are under testing/integration/logs/2026-10-30_barcode_fine_v0_1_*.txt.

## One batch of physical tests

1. Keep the lower bracket, sensors, divider and calibration exactly as during the original-control runs. Flash the exact candidate with Mac USB only, then unplug Mac USB before power-bank-only operation.
2. Wait10seconds, leave profile1/original calibration. Do not recalibrate for this comparison.
3. Test Z once, then Y once. Each time let the separate barcode sensor pass the whole pattern and trailing white region, then Stop. Record read letter and whether motion stuttered. No extra ready/positioned chat turns required.
4. Assess the decoded letter, not just pending action. Boot has no mapping; a decoded unmapped Z may have no pending action and increment unsupported. Apply a mapping if testing junction execution later.
5. Verify fresh source14 bit7 before attributing results. This candidate sends no IMU/ramp frames. Any old Buddy4 display is not live.
6. No claim of hardware success until those tests. If Z still fails, retrieve the new trace and compare margins; do not increase PWM or relax Code39 width validation blindly.
