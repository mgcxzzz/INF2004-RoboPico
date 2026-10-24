# Circuit with barcode V5: measured lower-contrast front trial

Built2026-10-24 for Pico W / RP2040 / Pico SDK2.2.0 / GNU Arm14.2.1,
CMake/Ninja Release, C11, application `-Wall -Wextra -Werror`.

- Artifact `firmware/buddy23_circuit_with_barcode_v5.uf2`,188,928 bytes,
  369 sequential RP2040 UF2 blocks. Padded end0x10017100, linker end0x100170bc.
- SHA-256 `0ae311b7a76960d21e2735d5764f984a43a8679e968bba9b4f2f24b4b02793b0`.
- ELF text98,488 / BSS150,632; BSS end0x20025f68, stack limit0x20040000.
  Record ABI remains64840 bytes, trace56 and summary40. Version5 uses the
  existing V4 analogue trace tail; records1-4 still load. Version6 rejects.
- UF2 block magic/family/payload/address/count/index/trailer, version/log/
  auto-reader/min_span markers and build/package equality verified. Image
  remains below probe slots at0x101c0000 and barcode record bank0x101d0000.
- V3 and V4 UF2 hashes checked unchanged by verify_firmware.py.

## Evidence and change

User calibrated white138/152 and black199/229 (spans61/77). Independent held
black samples:197/232,197/230,200/231,199/230,197/231. Held white samples:
138/153,137/153,137/153,136/154,139/156. These reported stationary ranges do
not overlap. This supports an experimental lower cutoff, not a moving-robot
accuracy claim. The raw reference values are not hardcoded in firmware.

Shared calibration predicate now has LINE_CAL_MIN_SPAN, default100. Only
the V5 driving target defines it as50. This applies consistently to saving,
restoring, startup and follower validity; it is not merely a save bypass.
Normalized0..1000 levels, steering gains/thresholds, speeds/PWM and analogue
barcode logic remain unchanged. h prints min_span and measured spans.
Existing other targets retain their historical100-count default.

## Executed tests

Native C11, warnings-as-errors via Zig0.13 Windows cc:

- test_front_calibration compiled/executed with default100 rejects the measured
  low-span references; compiled/executed with50 accepts them. With50, all five
  white/black pairs normalize to white<=54 and black>=950, reject all-white
  startup, accept black startup, and command balanced45/45 on the supplied
  black samples. Version5 low-span saved record validates and CRC corruption
  rejects. Below-limit, missing-capture and reversed-reference cases reject.
- Original test_barcode suite passes with target50: measured large-A replays,
  frame rejection, queue/cross semantics,12 alternating turns per profile,
  mirrored handover, compatible saved records and CRC checks.
- test_analog passes with target50: raw A/F scales/directions/ratios/speed
  changes, reduced narrow-bar amplitude, malformed/missing/noise rejection,
  unsupported/conflicting votes and4 alternating turns per profile.

Physical moving-line performance at the new height remains to test. Use
profile1 on a plain straight first and recapture w/l/r/k after flashing.
Junction geometry is still the old92mm side-ahead value; the new measurement
has not been supplied and was not guessed or changed.
