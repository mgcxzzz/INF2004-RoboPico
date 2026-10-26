# V1.2: gradual drive effort and motion-based cruise — 2026-10-27

**Physical follow-up FAILED.** `line_test_3.txt` / `IMG_4310.MOV` show worse tracking and stall/surge behavior. V1.2 is withdrawn as a recommended test/integration baseline. See [full analysis and next isolation test](V1_2_PHYSICAL_REGRESSION.md). Host/replay passes below remain factual but did not validate closed-loop behavior.

This is an experimental response to the user's V1.1 straight-line logs and IMG_4308.MOV. Physical smoothness has not yet been measured on the robot. API signatures, ownership rules and external Buddy5 finite maneuvers are unchanged. The V1.1 timestamp fix remains.

## Evidence from the supplied runs

| Run | Time | Distance | Average speed | Highest retained wheel target |
|---|---:|---:|---:|---:|
| Profile1, line_test (2).txt | 10.924s | 309.8mm | 28.36mm/s | 60mm/s |
| Profile3, line_test_2.txt | 13.048s | 347.7mm | 26.65mm/s | 58mm/s |

These are separate runs with different durations, not a controlled profile benchmark. Profile3's nominal135mm/s does not mean it was actually commanded: no retained target exceeded58. The old fast gate needed both normalized levels>=400 and difference<=80 for300ms; brief qualification plus speed ramping/correction/scan intervals did not yield sustained fast cruising here. Profile1 also had its own broader60mm/s boost, so these profile numbers were not guaranteed ordering of average speed.

Profile1 had543 active retained rows, no turn/search/loaded-capture phases, no invalid encoder transitions, and ended on the manual GP21 stop. Twenty left-wheel and21 right-wheel entries reached300PWM. At4.095s the right target stayed50mm/s while measured speed fell to4.5mm/s and PWM went179->300. At2.661s it stayed50 while PWM went176->300. These match the inherited near-stationary breakaway helper, not a request to stop. Other sharp optical corrections also exist; this is not proof that all jerk comes from one cause.

The video shows the straight approach and manual stop near the cross; it does not establish mat friction or exact motor load. The user confirmed the USB cable was loose and no bracket/housing was catching. Calibration spans3294/3540 are valid with substantial broad-surface contrast, though that does not guarantee consistent coverage everywhere. No evidence in this retained run shows an external coordinator takeover or a control-late stop. The legacy breakaway helper is present in V12.2 too, so a changed mat/load/coverage can expose the issue without a new ownership bug. There is no same-mat V12.2 A/B run establishing the integration as the cause.

## Changes

### Ordinary drive (profiles1 and3)

`drive_smooth.h` keeps the existing reference-speed ramp and PI calculation. For ordinary visible-line forward drive, it suppresses the immediate legacy breakaway pulse and instead slews physical PWM up at<=900permille/s and down at<=600permille/s (about+9/-6 counts per10ms). Each wheel qualifies persistent underspeed against its own target for60ms, then gradually builds retained load trim up to90PWM counts. Trim releases slowly near target and faster with overspeed, instead of disappearing as soon as the breakaway pulse ends.

It does not add a forced minimum or raise the existing30%/42% ordinary-drive caps. Zero, intentional very slow inner-wheel/coast, and a strong target reduction bypass the slew immediately. Deliberately slower wheels are not boosted towards the other wheel's target. Added load trim is not saved between runs. Regular assist code0 still means the normal drive path; it does not mean no internal trim.

The wrapper is bypassed for profile2 ramp mode, planned junction/U turns, line search/recovery, loaded capture and all external API maneuvers. These retain their existing controllers/caps. Thus V1.2 does not change Buddy5's generic reverse/turn execution power.

### Cruise qualification without new colour thresholds

`tracking_cruise.h` examines the history of requested steering and measured relative wheel turning, not raw ADC values or equality of front levels. It filters normalized steering ratio and its chatter plus encoder speed-difference ratio. After400ms settled forward tracking it gradually permits profile1 up to60mm/s or profile3 up to135mm/s. It maintains the optical steering differential; it never locks a heading or ignores a departure. Non-fast ordinary common speed is around45mm/s, and common acceleration/deceleration is bounded at+80/-240mm/s^2. Departing motion/chatter revokes fast qualification with hysteresis; urgent optical corrections, loss/search, planned turns and scan approach bypass this stage immediately. Barcode approach retains the existing45mm/s policy.

The new qualification contains no ADC or level comparison, and tests demonstrate identical results for different level values with identical control/motion history. The existing calibrated optical follower still supplies visibility and urgent-departure decisions; it necessarily still has calibrated classification thresholds. This is not a universal guarantee for arbitrary mats, sensor height, slip, or unseen geometry. It introduces control-history tolerances, not new hard black/white numbers. Encoder steadiness is an estimate and cannot prove ground heading in the presence of slip.

### Diagnostics and records

`h` identifies V1.2. `v` includes `DRIVE_SMOOTH` with per-wheel trim and tracking qualification state. `p`, `j`, `a` and `z` remain. Saved motion record20 uses the same64848-byte layout but identifies the changed behavior; old records import. Old V1.1/V12.2 binaries cannot read newly saved record20. Note the CAL references before reverting and recalibrate if needed after downgrade. No centre capture or new barcode calibration was added.

## Verification and limits

All retained optical, decoder, turn, search and interface tests remain. New tests check PWM bounds/slew/zero/steering bypass, profile3 cap420, twelve varying-stiction models, motion-history qualification with unequal brightness, actual-turn/hunting rejection, and the composed pipeline with analogue A/F/Z in both directions followed by cross dispatch.

The 20ms retained profile1 input fixture is replayed at its recorded timestamps through old and new drive calculations. Positive PWM jumps>80 counts fall from22 to0; on near-steady targets they fall from18 to0. This is a counterfactual open-loop command comparison using recorded speed as input, not a prediction of the new physical speeds or an exact10ms reproduction. The simple load models converge near the requested55mm/s, but their total cumulative PWM variation is sometimes higher with adaptive trim; the tests do not claim improvement of every smoothness metric or physical tracking.

The old9,000-tick V12.2 routing-parity test still tests the unchanged baseline/provider routing. It does NOT claim V1.2's new cruise/PWM outputs are identical to V12.2. New behavior is covered separately. See BUILD.md for the current executed test count, ARM sizes and UF2 hash.

## First physical test

1. Flash `firmware/buddy23_interface_v1_2.uf2`; `h` must show v1.2. Verify valid front calibration at the current mounting height. Keep the present calibration if it is still correct; no new centre measurement is required.
2. Select `1`, then `k`. Use the same straight and slack cable as the previous run. Start normally, stop after roughly10s, wait five seconds and send `p` and `z`. A short `v` capture can show the trim but is optional.
3. Check whether actual start-stop lurches and PWM jumps have reduced, not merely whether the average is larger. Then test a bend and A barcode. Full optical correction/loss behavior still has priority.
4. Only after profile1 improves, repeat the same straight on profile3. It can qualify with unequal levels now, but its135mm/s target remains conditional and is not guaranteed on a short or unsteady straight. Leave profile2 for the separate ramp experiment.

If motion becomes too sluggish or overshoots, retain the short log and compare to the saved V1.1 build on the same mat. Do not transfer the raw PWM measurements from this robot to another vehicle as universal feedforward constants.
