# Hump line V1 build, 2026-10-22

Artifact: `firmware/buddy23_hump_line_v1.uf2`

- SHA-256: `77668cdb7e401766d78880efc2ea06b9e27b5647cb12192f9dbf79460da602f3`
- 107520 bytes; 210 sequential UF2 blocks, 256-byte RP2040 payloads.
- Payload range `[0x10000000, 0x1000d200)`.
- ELF `__flash_binary_end`: `0x1000d1f4`.
- Penultimate hump-record sector `0x101fe000` and final V9-record sector
  `0x101ff000` are outside all UF2 writes.
- Arm size: text 57840, data 0, BSS 12104 bytes (includes ELF non-flash sections).
- `stdio_usb_init` present; `stdio_uart_init` absent.
- Pico SDK 2.2.0, Arm GCC 14.2.1, Pico W/RP2040 Release, Ninja,
  `-Wall -Wextra -Werror`; USB stdio, no UART stdio.

Host C11 checks passed with warnings as errors:

- Baseline and assistance-disabled outputs equal V11 speed PI/breakaway
  over target transitions on both wheels, including bridge levels.
- Intended target slowdown and healthy wheel tracking do not add load effort.
- Sustained independent underspeed adds effort before the retained 500 ms
  stall interval; extra <=150, total PWM <=350, no reverse output.
- Overspeed, recovery, zero request, and loss of optical eligibility remove
  assistance as specified; bridge sweep respects both selectable caps.
- CRC validation and legacy V9/V11 front-calibration imports; simulated
  adjacent-sector save preserves the legacy final sector.
- Existing optical loss, button/release, late-loop and start-placement guards.
- `verify_firmware.py` validates UF2 identity, block sequence, address range,
  banner, SHA-256 and equality to the compiled artifact.

This establishes software behaviour only. No physical hump V1 run has yet
been received. Actual loaded drive, supply behaviour, traction, and sensor
readings on a one-wheel hump require the staged flat/small-hump tests.

Only this new test directory is changed. V9 junction and V11 analog source
and firmware remain the baselines; load assistance is not integrated there.
