# Circuit with barcode V13: smooth visible drive and stationary recovery

Built2026-10-26 for Pico W/RP2040 with Pico SDK2.2.0, ARM GCC14.2.1,
CMake/Ninja Release, C11 -Wall -Wextra -Werror.

- Artifact: firmware/buddy23_circuit_with_barcode_v13.uf2,250880 bytes,
  490 sequential RP2040 UF2 blocks; padded end0x1001ea00.
- SHA256:2437fa97483a029d2e497145ad4c8d04f7c5295696c6c95d3b51037c87e1ca96
- Linker end0x1001e90c, below raw bank0x101b0000.
- ELF text129288/BSS215592; BSS end0x20035d68, stack limit0x20040000,
  leaving41624 bytes between those symbols.
- Saved record remains64848 bytes, trace56/summary40. V1..V13 load with
  version-specific CRC size; unknown14 rejects. Raw format/banks unchanged.
- UF2 family/sequence/addresses/trailers/markers/package equality pass;
  published V3..V12 binary hashes unchanged.

## Scope and evidence

School run:116.596s,3397.3mm,29.14mm/s average; manual GP21 stop, no accepted
A/F/Z frames, no invalid encoder transitions. Front calibration supplied
separately: white176/175,black3483/3498,valid1,span3307/3323. The retained
motion window begins97.719s; earlier rows were overwritten, so the first
barcode pass cannot be correlated to steering in this file.

The retained readings exhibit abrupt large optical side changes and repeated
PWM300 jumps. V12's strong-error filter bypass and maximum breakaway helper
can amplify these. This does not prove worn tape is the sole cause or prove
that every barcode failure was caused by steering.

V13 extends target smoothing over strong visible errors, preserving sustained
bend targets, raw optical loss thresholds and junction/decoder logic. Its
ordinary visible-drive controller uses gradual PWM and deficit assistance
instead of repeated maximum kicks. See README.md and visible_wheel.h for
gains, eligibility, phase handover and limits. Profile2 ramp drive, planned
junction turns and post-turn capture retain their previous controllers.

User also requested1.5s white qualification followed by stationary searching.
V13 uses signed counter-rotation at100mm/s,25 near encoder endpoints, with
35/60/90/120/150deg alternating offsets and80ms measured-still reversals.
Possible line sightings stop both outputs for optical confirmation. Search
is bounded to12s,1000mm accumulated wheel travel,30mm mean-wheel drift or
the final endpoint. Active-search flashes do not reset the budget. Signed
output and600/750 physical-PWM envelopes are wired through both control
commit and the independent ISR. U-turn power/geometry remain V12/V11.

## Executed validation

- test_visible_wheel: per-wheel/profile caps, gradual positive-output changes,
  sustained load authority, zero/off, previous-output handover, phase gating,
  and a simple sticky-wheel speed plant with temporary extra load. Both
  simulated wheels settle near45mm/s without repeated restarts. This is not
  measured motor/chassis performance.
- test_smooth_follow: mirrored noise attenuation, sustained strong-bend
  convergence, stop/recovery/junction isolation, faint400 boundary, profiles.
- replay_visible_log:929 retained visible rows,30 entries to PWM300 from below
  250,30 optical side reversals. Target total variation1864 recorded vs1465
  V13 on the same readings(21.4% less). Input replay does not predict the new
  closed-loop physical path. File is a general optional host diagnostic;
  the user's local PuTTY file is not committed.
- test_line_search: exact1.5s wait/hysteresis, measured-angle endpoints in
  both directions, settled reversals, no-tick timeout, travel/drift limits,
  stop-on-sighting and opposite-sensor acquisition, pending command/no-cross
  while lost, right/left mirror involution, enlarged alignment budget. Ideal
  counter-rotation model reacquires an opposite-side line with zero centre
  translation; physical slip/scraping cannot be excluded by this model.
- test_uturn: complete analogue Z both sizes/directions, two-quarter geometry,
  original-line return, lagged wheels, bounds, next junction after reverse,
  signed output and saved-record integrity across V1..V13.
- test_barcode: generic Code39, historical A captures, queue/CRC and12
  alternating A/F turns/profile; also compiled/executed without search macro
  to check shared-header fallback behaviour.
- test_analog, test_barcode_sensitivity: full A/F/Z frames in both directions,
  both sizes/ratios, low ADC contrast, ambiguity/missing-bar rejection.
- test_maneuver_power: both signs/wheels, sustained600/bounded750 power,
 180ms independent clamp, early motion cutoff, zero/off and output pin mapping.
- test_ramp_profile, test_start(192 cases), test_front_calibration,
  test_barcode_raw, and six Python raw-parser cases passed.

Physical V13 success is not claimed. Test profile1 on a short straight and
one barcode first, collecting logs immediately; then test stationary recovery.
The previous stationary-turn drag issue may still limit counter-rotation.
