# Circuit With Barcode V12.1 — V12 + scan heading hold

This experiment is based on V12 commit `f0e3b35`, not V16. The transitive local C/header dependencies are isolated under `baseline/`; `BASELINE.json` records their original SHA256 values. Normal line tracking, smoothing, motor PI/PWM, rolling recovery, junction geometry and A/F/Z decoder are V12. No centre calibration or V16 coverage steering is included.

Download: [buddy23_circuit_with_barcode_v12_1.uf2](firmware/buddy23_circuit_with_barcode_v12_1.uf2).

## First physical test

1. Flash the UF2 and check `h` says **v12.1**. Select `1`, then `k` to save the profile. Valid existing front calibration imports from records V1–V16; no new calibration is needed if the mounts/surface have not changed. Check the CAL line before driving. Your latest reported references were white164/168, black3514/3389; these are not hardcoded.
2. Position the robot parallel to the line, with the offset sensor over broad white about8–10cm BEFORE the first bar. This allows the wheels to settle before the entry gate. Starting on a barcode is not the intended test.
3. Use the approximately140mm barcode first, then its separate junction. Use profile1. Test A first, then F and Z.
4. Start as usual: USB `g` gives2s; USB `f` requires GP21 held. On a power bank wait5s, hold BOTH3s, release, wait1s, tap GP21, then2s countdown. Either button/`s` stops.
5. Stop and wait5s before changing power. Send `j` first. It includes HOLD_BEGIN, HOLD_PATTERN, HOLD_END/CANCEL, DECODE, CROSS and DISPATCH. Send `p` as a file only if the hold or motion needs investigation.

The stored numeric record version is17 to distinguish this fork from historical V12/V13–V16. Help and new run dumps identify it as v12.1. Old saved runs retain their old labels. V12.1 imports the old record layout; old V12 does not read a newly saved record17, so note the CAL references before downgrading.

## Override behaviour

- Possible first mark: offset A0 rises16counts above its run-start broad-white baseline for at least6ms (observed on10ms control ticks). This provisional trigger never decodes a letter or queues a command. The unchanged63-bank reader still requires two neighbouring thresholds to validate a complete Code39 frame.
- Entry requires at least300ms steady forward encoder motion, both wheels>=15mm/s, difference<=15mm/s, encoder yaw change<=3degrees, a visible front line and clearance from the previous turn. A mark during a clear turn does not arm the hold. This gate is an estimate, not proof of track geometry.
- During a hold the base target is45/45mm/s. Relative right-minus-left wheel travel and speed provide proportional/damping trim, bounded to+/-8mm/s. Trim and entry targets slew gradually. Wheel control itself stays V12.
- Front ADC values remain real in logs and junction qualification. Ordinary optical steering/loss timers are frozen during the hold;150ms continuously below150normalised on BOTH fronts cancels it and returns to fresh V12 recovery. One white sensor alone does not cancel the hold.
- A second mark or neighbouring start-frame readers records HOLD_PATTERN. A complete accepted decode (including unsupported symbols) ends the hold. A/F/Z remains queued for the separate cross.
- No decode:12mm continuous background,180mm forward travel,5seconds,>8degrees encoder heading error, conflict/fault or junction dispatch releases/cancels the hold. It cannot remain indefinitely active. There is a12mm quiet re-arm after release; bounded noise can cause a tentative hold but cannot create A/F/Z.
- A200ms bounded target handover after decode/quiet/scan limit avoids an instantaneous return to old steering. Stale pre-hold optical state is cleared. Stops, loss recovery and junction manoeuvres bypass the handover.
- Junction dispatch takes priority on the same tick. Existing V12 encoder centring,92mm side-sensor-to-axle geometry and55mm early rolling turn start remain. The scan heading is NOT carried into a turn or outgoing-line acquisition. A single uncommanded stripe simply releases the tentative hold after the quiet gap; it cannot queue a turn.

This180mm/5s experiment is for the current roughly140mm sheets. The decoder remains size-independent, but a280mm sheet can exceed the new hold window and fall back to normal V12 following before decoding. Do not assume scan stabilisation covers larger sheets.

## Reading the logs

Motion decision24=heading hold;25=handover. HOLD event width is travel since hold entry. Its three hex pattern columns contain front normalised L, front normalised R, and encoder yaw in centidegrees+18000; they are NOT Code39 patterns on HOLD rows. All decoder rows keep their original meanings.

HOLD_END/CANCEL flags:1 decoded,2 quiet,3 travel180mm,4 time5s,5 front white150ms,6 yaw8degrees,7 junction dispatch,8 fault/conflict. A HOLD_END is not proof of a decode: check DECODE and decoded_A/F/Z counters. Encoder heading is an estimate; it cannot observe lateral sliding or correct a crooked entry heading.

## Source/build

`main.c` includes the isolated V12 main. `scan_hold.h` owns the new state and orchestration. Six baseline files have reviewed changes: main wiring/banner/record label, barcode_nav override hook, barcode_scan event IDs, barcode_record accepted versions, and two record-version regression fixtures. The original SHA manifest and `verify_firmware.py` check all other baseline dependencies exactly, including smoothing, wheel controllers and recovery.

Build with the installed Pico SDK for `pico_w` through this folder's CMakeLists. Run `python run_tests.py` with the workspace Zig host compiler and `python verify_firmware.py` after copying the build UF2 into firmware/. See BUILD_V12_1.md for executed results. Software tests do not establish physical barcode or bend success.
