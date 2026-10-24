# Circuit with barcode V4 automatic A0 build evidence

Built2026-10-24 for Pico W / RP2040, SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, C11, application `-Wall -Wextra -Werror`.

- Artifact `firmware/buddy23_circuit_with_barcode_v4.uf2`,188,928 bytes,
  369 sequential RP2040 blocks. Padded end0x10017100, linker end0x10017014.
- SHA-256 `7719f10c2b1f7c451bc104be6ac3e7b6a10c0c75864fd1a2db7a108d5022f2c5`.
- ELF text98,320, BSS150,632; BSS end0x20025f68, stack limit0x20040000.
- Every UF2 block's magic/family/payload/address/count/index/trailer verified;
  required auto-analogue/event/log markers and build/package equality checked.
  Binary remains below probe slots0x101c0000 and barcode bank0x101d0000.
- V3 UF2 SHA remains9946f4d3aaafea3769803a8ad6a406c3ce20be4adb0344a4f2ae9721356ad6bd.
- Record ABI remains64840 bytes, trace56 and summary40. V4 accepts/prints
  versions1-3 and versions its reinterpreted four-byte motion tail. Unknown
  version5 and corrupted records reject. Existing firmware UF2s unchanged.

## Executed verification

Native host C11, warnings-as-errors via Zig0.13 Windows cc:

1. Original test_barcode suite passes: supplied PDF A/Z, generated A/F sizes/
   directions, measured large-A fixtures and time-interpolated decode/cross
   replay, colour bias/jitter, invalid frame/expiry rejection, digital edge
   capture overflow, queued command semantics,12 alternating turns per profile,
   mirrored acquisition, no-command straight and CRC/record compatibility.
2. New test_analog suite passes raw ADC A/F at two scales, two directions,
   ratios2/3 and changing speed; narrow black amplitude1800 vs wide black3600
   on white220; no manual side calibration. It rejects missing narrow bars,
   incomplete stop and noise, ignores unsupported Z for routing, rejects
   contradictory threshold groups, qualifies the separate cross by distance,
   and completes4 alternating A/F turns per profile in navigation simulation.
3. Probe C calibration/CRC tests and6 Python transport-audit tests pass.

Source review verifies sole main-loop ADC owner and GP27/ADC1 routing, first
conversion discarded after channel switching, GP19 unused, no ADC in timer
ISR, no operator side calibration/start gate, unchanged follower/PWM geometry,
quiet live status by default, bounded existing guards and paced p/j dumps.
The first compiler pass flagged a misleadingly indented ADC sampling loop;
it was split explicitly and the warnings-as-errors rebuild passed.

Threshold search requires two adjacent readers with matching complete Code39
frames, not an assumed A. A newly selected stop bar cannot be a junction.
Physical response on the141mm barcode remains unverified; inverted/overlapping
narrow-feature signals can still fail every threshold and will not be forced
to a route command. No decode was inferred from the incomplete raw PuTTY logs.
