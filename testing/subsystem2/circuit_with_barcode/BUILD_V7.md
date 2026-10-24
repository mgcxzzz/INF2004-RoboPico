# Circuit with barcode V7: alternating rolling search

Built2026-10-24 for Pico W/RP2040, Pico SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, application C11 with -Wall -Wextra -Werror.

- UF2:190464 bytes,372 sequential RP2040 blocks, padded end0x10017400.
- SHA256:3033548bffcdda8954f5758bf372b885f23542ffa738c79603f7666a28edf804.
- Linker end0x100173d4. ELF text99280/BSS150680, BSS end0x20025f98,
  stack limit0x20040000. Image remains below all calibration/log banks.
- Record64840 bytes, trace56, summary40. Versions1-7 load; unknown8 rejects.
  Search state is runtime-only, not added to the flash record. Existing trace
  decision/assist bits identify the new recovery stages and loaded wheel.
- Packaged/build UF2 equality, block magic/family/payload/address/index/count/
  trailer, version/search/start markers verified. V3-V6 UF2 hashes unchanged.

## Change and scope

CIRCUIT_SWEEP_SEARCH is private to the new executable. Existing targets retain
their original recovery code. Visible-line steering, low-contrast correction,
speed profiles, front calibration and barcode readers are unchanged.

The150ms bridge precedes five alternating rolling arcs lasting350/700/1050/
1400/1750ms. Start toward the remembered recovery side; unknown starts left.
Each arc requests0/45 or45/0; while confirming a return, use30 on the moving
wheel. Search expires after5.25s or150mm additional path, including with zero
encoder ticks. This cannot guarantee stationary position or angular coverage.

The existing40/80/120ms strong/medium/faint acquisition checks end search.
Transient signals do not reset the timer. The moving wheel receives the
existing loaded-turn controller up to42% PWM; prepare it without a48% peak.
The other wheel remains off. Normal driving PWM is unchanged. Pending A/F is
retained when a crossing is rejected during line loss; navigation dispatch
cannot start while the follower is in recovery. Manual/USB/control/direction/
invalid-encoder and junction bounds remain. Encoder-progress is warning-only.

## Executed validation

- test_line_search: both initial directions, every stage boundary, confirming
  speed, zero-tick timeout and distance bound; short-gap handling;20ms return
  rejection without clock reset; opposite-sensor-only acquisition; visible
  line steering parity; invalid calibration; loaded effort<=420 with no peak;
  off-wheel zero; pending command retained and dispatch blocked while lost.
- test_start:192 combinations of non-optical startup conditions; all three
  modes; old reason11 placement rejection removed; no-history start waits
  then searches, and expires even with zero motion.
- test_barcode: captured large-A replays and following crosses, A/F scales/
  directions, framing rejection,12 alternating turns per profile, mirrored
  acquisition, no-command behavior, flash versions/CRC/record layout.
- test_analog: raw ADC A/F sizes/directions/ratios and speed changes; missing/
  malformed/noisy signal rejection; unsupported/conflicting frames; crossing
  dispatch and4 alternating turns per profile.
- test_front_calibration passes with target span50; shared-default100 version
  also passes. Original test_circuit without the opt-in passes its12-turn,
  recovery, effort, feedback, worn-gap and mirror regression suites.
- Pico cross build and UF2 verification pass. Optional SDK wireless/BLE
  submodule warnings do not affect this USB-only executable.

Physical search behavior remains to validate after lowering and recalibrating
the sensors. Search cannot repair insufficient optical contrast. Geometry
still uses the old92mm offset-ahead value until a new measurement is supplied.
