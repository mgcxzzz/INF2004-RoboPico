# Barcode fine v0.3 — complete-frame quiet-zone fallback

## Problem and resulting behaviour

The original and latest failed Z scans are recoverable by the unchanged full Code39 reader, but the latest gives just one valid acquisition setting (205ADC,hysteresis2). v0.2 requires two neighbouring threshold votes and rejects it. v0.3 retains the existing immediate consensus path and additionally accepts a single validated A-Z frame only after physical white quiet zones before and after it, with no conflicting full-frame vote. No missing edges or letters are inferred.

## Changes

- Fine family indices1..32 now use odd thresholds193..255 and hysteresis2, instead of even192..254/hysteresis3. The coarse128/256/...3968 thresholds,2ms debounce and existing204/205 hysteresis1 pair are retained. Same65-bank count; preview bank11 is now213ADC.
- The actual reader records the measured white run before the start sentinel. Subminimum-width errors invalidate that history.
- Single-vote fallback requires a complete unchanged29-element Code39 validation, A-Z letter, both white quiet zones at least10 times the larger learned narrow-module width, no intervening edge/new frame, and a vote no older than3s. Waiting stationary cannot satisfy the trailing distance. A competing complete letter on the same span vetoes acceptance. Existing duplicate and late-conflict handling remains.
- Standard two-neighbour consensus retains its original behaviour and timing. The fallback delays its letter event until trailing quiet space is observed; allow room before the next junction. The barcode must be passed completely before stopping.
- Periodic source11 STATUS detail0xbc03 (48131) identifies this build. Diagnostic bank and flags layout stays as repaired in v0.2. Both current decoders work unchanged.
- Saved trace version31 distinguishes changed fine-bank semantics. Import supported older records/calibration (including30); historical bank thresholds are decoded by version. Wi-Fi configuration and front line calibration are preserved.
- No motor, steering/follow, encoder, motion/coordinator, transport or command-code changes. SHA checks in the manifest verify this scope. No Buddy4 is compiled into this isolated original-v2.3 barcode test candidate.

## Validation

- Both recorded Z captures decode once; latest capture uses exactly one valid reader candidate and the delayed quiet-zone path. The Y capture still has no candidate: physical Y/A-Z acceptance remains unproven.
-52 synthetic complete reader frames cover every A-Z in both directions and the fallback. Directed tests reject missing/insufficient quiet zones, stationary waiting, stale votes, intervening edges, active new frame, unsupported fallback symbol and competing complete letter. Duplicate suppression verified.
- Real latest-Z mutations reject cropped quiet zones, erased start/stop sentinels and missing wide bars.40s deterministic near-threshold noise remains negative. Existing missing-narrow-bar acquisition tests pass.
-36 C executables,10 JS schema tests and Python suites pass. Actual periodic harness/C packer output decodes in both candidate and merged dashboards. Focused replay/negative suite passes ASan/UBSan.
- ARM Release build0warnings. text514352/BSS200644, heap region47340bytes, two disjoint8KiB core stacks. Largest compiler frames main2360/control1184 (not a runtime watermark). UF2 format/family checked.

## Local artifact

/Users/skyler/SIT/Y2T1/Embedded/bench_tools/buddy2/buddy123_interface_wifi_v2_3_barcode_fine_v0_3.uf2

SHA256 `57fdb5ceb7538bd530eca48122407dc0cb734a821d1cfbcf113709190f2b6643`. Contains local credentials; do not commit. Nothing flashed or driven by agent. Old versions retained separately.

## One hardware test

Flash this exact file using Mac USB. Disconnect Mac, then use the power bank only. Reload9072, connect and enable telemetry. Keep the same bracket and front calibration. Start at least5cm before Z and allow the barcode sensor to travel at least5cm beyond the final bar before Stop. Report whether Z decoded and whether it stuttered; no separate ready/positioned messages needed. Agent verifies0xbc03 and reads fresh counters from the existing native capture. If Z succeeds, physical tests of more letters and both orientations remain necessary; do not call this universally validated from replay alone.
