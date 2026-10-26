# V12.1 build evidence —2026-10-26

Baseline: commit `f0e3b35`, isolated transitive local includes and11 regression fixtures in baseline/ (45 files). BASELINE.json contains original SHA256 values; BASELINE_PATCHES.json records the six reviewed modified snapshots. Original V12 smoothing, motor PI/PWM, recovery, junction and U-turn implementations are exact byte copies.

Changes: scan_hold.h + barcode_nav RAM override hook; main orchestration/banner; four added event IDs; saved-record acceptance through17 without layout change; two record-version fixtures extended through17. No changes in published V12–V16 source or UF2 artifacts.

## Executed

- Pico SDK2.2.0, GCC14.2.1, pico_w/RP2040 Release. Compiled -Wall -Wextra -Werror, linked and generated UF2 successfully. Optional WiFi/Bluetooth SDK submodule warnings are expected for this USB-only firmware.
- All12 host test executables passed using Zig0.13 with -Wall -Wextra -Werror, LINE_CAL_MIN_SPAN=50, CIRCUIT_SWEEP_SEARCH=1.
- New test_scan_hold: exact non-scan V12 demand parity across visible black/partial coverage/one-white cases; mark noise and bend gates; both front directions ignored during active scan; bounded trim sign and slew; bias/lag model for either wheel; quiet,180mm,5s,150ms front-white,8degree yaw exits; complete140mm A/F/Z both scan directions; pending command consumption and same-tick dispatch priority; missing bar cannot create command.
- Retained V12 fixtures: generic Code39, analogue banks/low contrast, missing bars/stop rejection, both sizes/directions, unsupported/conflict handling, historical large-A replays, cross separation,12 alternating A/F junctions per profile, outgoing mirror, Z geometry and signed output, rolling search, maneuver peak limits, ramp experiment bypass,192 start-guard combinations, smoothing response, calibration and saved raw/record CRC. These are model/unit tests, not physical course results.
- Record remains64848bytes, trace56, summary40. Versions1–17 import; unknown18 rejected. New write17 is labelled v12.1 in user output. Raw capture49408bytes unchanged. Saved calibration import does not require a new manual barcode calibration.
- UF2 structure/family/contiguous addresses, image-vs-flash-bank separation, required banners, baseline hash checks and original V12 artifact hash all pass. Packaged UF2 equals build output.

## Artifact

- file: firmware/buddy23_circuit_with_barcode_v12_1.uf2
- bytes:254976; RP2040 blocks:498
- SHA256:e02d8a735edd766b67580cf2534290b9ffdcc572ad3e9a54396b8cf2154e1177
- padded end:0x1001f200; linker end:0x1001f104; first data bank:0x101b0000
- ELF text131328, data0, BSS215504
- BSS end0x20035d70; StackLimit0x20040000; remaining gap41616bytes

## Physical limits

No physical V12.1 run yet. Use profile1 and current approximately140mm barcode first. Begin on broad white well before the barcode; hold entry estimates straight motion from encoders and visible line. A larger280mm print may exceed180mm/5s hold window, then continue under V12. Encoder guidance cannot observe lateral slip or guarantee a straight track/entry angle. A false provisional mark is bounded and cannot queue a route. Current92mm geometry remains provisional after any sensor remount. Existing V12 rolling2s search remains, not later stationary search. Valid front calibration and manual/USB/control/direction/invalid guards remain; no-progress remains warning-only.
