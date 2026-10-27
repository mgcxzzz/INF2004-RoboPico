# Interface V1.3 forward-effort experiment

## Reason for the change

The fixed45/45mm/s probe was already smooth. The user explicitly does not need to repeat it. V1.2's physical run was worse: an outer correction could switch from147 to410 PWM, then ordinary output stayed high while measured speed overshot. See V1_2_PHYSICAL_REGRESSION.md for the log evidence and limitations.

V1.3 starts from the V1.1 hardware adapter and keeps its startup clock fix, coordinator APIs and V12.2 optical targets/speed qualification. It removes V1.2's extra cruising qualifier and separate load-trim integrator. It is not a speed increase or new black/white threshold experiment.

## Active forward controller

`forward_effort.h` is enabled only for autonomous ordinary C_FOLLOW in profiles1/3 with a visible line, outside capture, search and planned manoeuvres. The existing CircuitLoad selector still qualifies the outer wheel from measured underspeed. Its selection now changes the permitted output ceiling rather than installing pivot feedforward and resetting the controller. Both sides keep the same PI state across this ordinary/assisted handover.

- Existing per-wheel feedforward and Kp1/Ki3 are used, with one bounded integral. No additional load integrator or forced continuous PWM floor.
- Reference acceleration100 and deceleration400mm/s^2. Output rises at at most1800 PWM counts/s (18counts per10ms), including starting assistance. Reductions have no output slew delay.
- After60ms of genuine near-stall with target/reference>=20mm/s and speed<8mm/s, one starting-aid request up to300 PWM is permitted for at most100ms. It ends early at measured20mm/s, falling target, low target or overspeed. Because output still rises gradually, it need not reach300 before ending. It does not retrigger until a zero-target episode or controller-mode reset.
- Starting effort hands over through the same bounded PI state. Overspeed>8mm/s above the lower of requested/ramped speed clears positive retained I; >20 above the final target coasts immediately. A deliberately slow inner target<=15 also coasts while >5mm/s too fast. These are wheel-speed conditions, not new optical brightness thresholds.
- Ordinary profile1 cap300; profile3 cap420 when the original fast qualifier selects that wheel profile. Qualified sharp-correction assistance permits420; it does not force420. Existing final hardware caps still apply.
- Zero target is immediate. Manual stop and generation-checked ownership cancellation are unchanged. Planned A/F/Z turns, post-turn capture, line-loss search, external Buddy5 primitives and profile2 ramp retain their previous controllers. Handover from those modes is matched to preceding output within the ordinary cap and integral bounds; it is not a claim of identical torque.

The new state is12bytes per wheel on the tested compiler: float duty plus deficit16bits (0..60000us), pulse17bits (0..100000us), prior target8bits (ordinary range0..180) and three1-bit flags. The old controller supplies the sole PI state. Added24B BSS relative to V1.1;24B less than V1.2. No public request/audit format changes. Motion record21 imports earlier calibration/logs; older firmware cannot import a newly saved21 record. Record CAL references before a downgrade.

## Verification and its limits

New checks reproduce the147PWM-to-assistance handover, remove drive immediately for the logged39target/84.7actual overspeed case, verify zero/slow inner-wheel behavior, one-shot starting aid and output caps/rising bounds. Six simple changing-load wheel models settle within3mm/s of45mm/s. Six combined kinematic/sensor/follower/load-selection models keep the stripe and exercise the ordinary/assisted path. These are regression models, not proof that the real mat, slip, friction or sensor optics are represented accurately.

The retained API, clocks, atomic stop/commit, storage, Buddy5 workflow, optical/Code39/turn/search tests pass. Four V1.2-only tests are still run under an explicit ARCHIVED label for historical diagnosis; their smoothness/cruise wrappers are not in the V1.3 executable. Provider-routing parity tests refer to unchanged optical demands and the legacy PI reference, not identical V1.3 motor output.

Default Pico W, unpacked measurement and opt-in Buddy5 example builds pass warnings-as-errors. Physical following, scanning, bends and ownership takeovers still need on-robot confirmation. The first experiment should isolate ordinary following; do not infer that more aggressive PWM or a faster profile is automatically better.

## Test on the robot

1. Flash `firmware/buddy23_interface_v1_3.uf2`; `h` must show Interfacev1.3 and valid CAL FRONT. Existing V1.2 calibration imports; no new calibration is required solely because of this upgrade.
2. Select `1`; use the usual startup (`g` on USB gives2s countdown). Run on the same straight then bend. No repeated four-second probe is required.
3. Stop; wait5s. Save `p` for target/speed/PWM/assist comparisons. `j` alone cannot diagnose drive oscillation. `v` shows FORWARD_EFFORT live but is optional; default live remains off. Existing `z` coordinator audit remains separate.
4. If smoother, test the A barcode and junction, then profile3. If still jerky, send the same-straight log and observations; do not make a simultaneous speed/calibration/mounting change that obscures the comparison.

V1.1 source72e11ce/its preserved artifact remains the comparison baseline; V1.3 is a test candidate until physically validated. Original V1 has the startup bug. V1.2 remains archived as a failed physical experiment.
