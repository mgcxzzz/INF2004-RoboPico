# Circuit with barcode V6: remove optical startup placement gate

Built 2026-10-24 for Pico W / RP2040, Pico SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, application C11 with -Wall -Wextra -Werror.

- Artifact: firmware/buddy23_circuit_with_barcode_v6.uf2,188928 bytes,
  369 sequential RP2040 blocks. Padded end0x10017100; linker end0x1001708c.
- SHA256:67e7e459ea00077d30093e9530f89a35fa00d58258338f8eac30ee0151130773.
- ELF text98440, BSS150632; BSS end0x20025f68, stack limit0x20040000.
- Saved record64840 bytes, trace56, summary40. Records1-6 load; unknown7
  rejects. Analogue trace layout and CRC unchanged. UF2 remains below all
  calibration/log banks. Packaged UF2 equals build output; V3/V4/V5 hashes
  verified unchanged.

## Change

The reported START reason11 was the profile5 front normalized-black>=650
placement gate. V6's target-local barcode_start_block removes that gate.
Active, settling, valid calibration, STOP, ready and HOLD-mode conditions
retain their previous priority. Rejection diagnostics show front_valid and
front levels, replacing the unrelated legacy side-calibration field.

This does not change line steering or introduce the groupmate's algorithm.
Front minimum span50 remains; calibration must still represent the normal
resting mounting height. On a genuinely all-white start, the existing follower
can stop immediately because it has no remembered search direction. No blind
startup travel, new recovery window or changed motor effort was introduced.

All A0 threshold readers, pin assignments, profile speeds, PWM, line/turn
checks, rolling turns and the old92mm side-ahead geometry remain unchanged.

## Validation executed

- New test_start:192 combinations of active/settling/valid/STOP/ready/HOLD
  across three modes match the old non-optical checks and their priority.
  Latest reported references137/153,303/1461 plus earlier readings197/230
  reproduce reason11 in the old gate and pass the new gate. These measurements
  are from different captures; this is a regression fixture, not a claim that
  they characterize the current mounting. Existing all-white/no-direction
  running stop remains exercised.
- test_front_calibration passes at target minimum50 and shared default100.
- test_barcode and test_analog pass, including captured large-A replays,
  complete-frame rejection/acceptance, both scan directions/scales, mirrored
  junction handover, repeated queued turns, record compatibility and CRC.
- Cross compilation and UF2 verification pass. Physical driving with the new
  mounting is not yet validated. Missing optional SDK wireless/BLE submodules
  do not affect this USB-only test executable.
