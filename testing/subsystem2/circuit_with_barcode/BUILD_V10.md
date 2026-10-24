# Circuit with barcode V10: aggressive profile-2 ramp experiment

Built 2026-10-25 for Pico W/RP2040, Pico SDK 2.2.0, GNU Arm 14.2.1,
CMake/Ninja Release, C11 with -Wall -Wextra -Werror.

Artifact: firmware/buddy23_circuit_with_barcode_v10.uf2

- 220672 bytes; 431 sequential RP2040 UF2 blocks.
- SHA256: 18aba75df836187437c3cd7adff2ab01e682d679173f14900ce88986b603cf05
- Padded end 0x1001af00; linker end 0x1001aebc.
- ELF text 114360/BSS 215276; BSS end 0x20035c2c, stack limit 0x20040000.
  The gap between these symbols is 41940 bytes.
- Flash record addresses and 64840-byte motion record/56-byte trace/40-byte
  turn summary are unchanged. Main versions 1-10 load; unknown 11 rejects.
- Package/build equality, UF2 addresses/sequence/family/trailer and markers
  pass verification. Published V3-V9 UF2 hashes are unchanged.

## Changes

Only profile 2 ordinary visible-line drive uses ramp_profile.h. Stable
unmarked cruising requests180mm/s, compared with45/90 in profile1/old profile2.
The existing stable-centred gate and V9 target smoothing remain. Scan/pending
approach and corrections still lower requested speed through existing optics.
The stronger wheel controller is available for these visible corrections too.

Each wheel uses its own ramped target, physical-PWM feedforward, Kp3.5,
Ki10 and integral bounds +/-180. Reference acceleration is at most400mm/s^2;
lower requests apply immediately and overspeed coasts. Extra qualified effort
rises2000permille/s to220 and decays1600permille/s when unnecessary.

PWM is bounded to500 normally and650 during a qualified1.5s load window.
A deficit larger than max(12mm/s,30% of ramped target) must persist40ms.
An unresolved stall cannot renew the deadline. A wheel must return close to
its own target for200ms to re-arm. Intentional zero switches the wheel off
and clears state. No historical20% output multiplier is applied again.

The control commit and independent1ms ISR share an integer-only650/500
envelope. Entry/exit clears incompatible PI/boost state. Junction centring,
turn/alignment, both-white search and profiles1/3 retain prior controllers.
Profile1 retains V9 steering and barcode behaviour. Manual/USB/control-late/
direction/invalid-encoder checks remain; no-progress is still warning-only.

j adds a compact RAMP2 summary with configured limits and recorded PWM peaks
per wheel from retained motion rows. Actual speed remains an encoder estimate.
There is no IMU ramp detection, new blind heading hold or line-loss extension.
This is a physical power/traction experiment, not a guarantee of climbing or
of preserving IR visibility when the orange ramp rolls the chassis.

## Executed tests

- test_ramp_profile: independent left/right underspeed response; full650
  effort reached under sustained deficit;40ms qualification;1500ms expiry
  enforced at the ISR envelope; stalled feedback cannot extend deadline;
  recovery permits a later boost; deliberate slow/off targets; falling
  targets/overspeed coasting; visible-line eligibility; loss/junction bypass;
  forward PWM pin mapping; profile1 wheel-state/output parity.
- test_smooth_follow: V9 alternating-noise smoothing, mirror symmetry,
  sustained bends, urgent/recovery/junction bypass, weak400 boundary,
  45/180/135 cruising and correction ceiling after faster cruise.
- test_barcode: PDF/historical A captures, framing, queues/crosses, unchanged
  record ABI;12 alternating left/right turns per profile through the drive
  wrapper, including profile2's new180 target.
- test_analog, test_line_search, test_start, test_front_calibration,
  test_barcode_raw and test_barcode_sensitivity pass. They cover analogue
  decoding and rejection, mirrored turns, search bounds/effort,192 startup
  combinations, calibration, raw integrity and low-amplitude features.
- Six raw-dump Python parser tests pass.
- Pico W build and UF2 verification pass. Optional wireless/BLE SDK modules
  are unused by this standalone USB firmware.

User-confirmed physical baseline: V9 was smoother and read both A and F on
the first attempt. V10 profile2 has not yet been driven on the orange ramp.
Select2/k, check flat response, then test with20-30cm run-up and clear space
after descent. Send j and a description of where/how it stopped if applicable.
