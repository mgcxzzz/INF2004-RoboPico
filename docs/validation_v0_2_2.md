# Software evidence: provisional Buddy 1–4 RTOS v0.2.2

This release builds on main v0.2.1 and the hardware-tested Step 5 v0.1.5.
No robot was connected during development. Host checks and linked-image checks
do not prove successful physical bends, braking, recovery or junction turns.

## Checks performed

- Ten focused native C programs passed: original continuous steering/follow,
  original line search, new bend assistance, stationary search, drive diagnostic
  packing, junction sensing, release exports, release bridge and RTOS barcode
  integration. The legacy and stationary search programs were rerun after the
  final compact state-field change.
- Eighteen Python tests passed: five passive wireless-capture checks and thirteen
  binary line-export checks.
- Fourteen Node tests passed for existing fault codecs and the new junction/drive
  frames. The separate actual-C-to-browser release fixture check also passed.
- The headless browser check passed new diagnostic panels, old-packet compatibility,
  stale-state handling and complete binary/CSV/JSON stopped-export integrity.
- The final ARM release build passed: text 494824 bytes, data 0, BSS 195408.
  The existing linker RWX-segment warning remains. Packaging checks enforce flash
  bounds, required RTOS/feature symbols, RAM flash routines and stack-frame limits.
- Static RAM end is 0x2002fd90, 496 bytes above v0.2.1. This leaves 66160 bytes
  before dynamic kernel/task allocations, not a measured stack high-water margin.
- Exact source checks preserve 92 protected executable files from Step 5 plus the
  existing main supervision policies. Six existing files have explicitly reviewed
  release fingerprints for the intended bend/search/junction change boundary.
  Both decoders, calibration inputs and IMU/ramp algorithms are unchanged.

C7 adds a 20-byte junction snapshot; C8 adds an 18-byte drive snapshot at 1 Hz
when telemetry is enabled. Existing runtime packets total 226 application bytes/s
at that cadence, excluding other telemetry and MQTT/TCP framing. Controller history
keeps its 56-byte row size, using two spare bits for bend phase. Continuous control
scratch retains float32 precision; new categorical fields and wire values use
bounded bit fields/masks. This is scoped representation evidence, not a claim of
whole-firmware completion of the Tribunal requirement.

## Required hardware acceptance

Follow [the v0.2.2 guide](drive_refinements_v0_2_2.md): short straight with
Pause/Resume/Stop, one troublesome sharp bend, supported stationary recovery
including Stop, and the previously missed junction. Preserve failed runs and
video. No unchanged ramp/reconnect sequence was repeated just for this release.

The old v0.2.1 artifacts remain available as rollback. Buddy 5 is not integrated.
