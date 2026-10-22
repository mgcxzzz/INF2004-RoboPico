# Hump line V2 build, 2026-10-22

Artifact: `firmware/buddy23_hump_line_v2.uf2`

- SHA-256: `fd26ed434630a4b0ac03f6b6f1c1f556bc7083af834af0c0e098342f2b78b94b`
- 108032 bytes; 211 sequential RP2040 UF2 blocks with 256-byte payloads.
- Payload range `[0x10000000, 0x1000d300)`; ELF end `0x1000d244`.
- Hump record `0x101fe000` and junction record `0x101ff000` excluded from writes.
- Arm size: text57920, data0, BSS12104 bytes.
- USB stdio symbol present, UART stdio absent.
- Pico SDK2.2.0, Arm GCC14.2.1, Pico W/RP2040 Release, Ninja,
  C11 with `-Wall -Wextra -Werror`.

Changes: assistance eligibility floor30 ->20 mm/s, matching the existing
blocked-wheel guard. No speed-target, steering, sensor, PWM-ceiling, or
stall-window increase. Existing V1 calibration/log layout retained; the
formerly zero-filled padding now identifies the recorded firmware revision.
Restored V1 runs retain their V1 label until a new run replaces them.

Host tests passed with warnings as errors:

- Stalled target20 and target32 ->20 on either wheel receive output above250
  within500 ms; extra remains <=150 and PWM <=350. Healthy other wheel is
  unaffected. This reproduces the near-crest eligibility failure from V1.
- Deliberately slowed wheels following their requests do not gain assistance.
- Baseline and invisible-line operation remain equivalent to the V11 controller.
- Overspeed, recovery, below20 requests, zero requests, and loss of eligibility
  remove assistance as specified; bridge output remains forward only and capped.
- V1/V2 saved-record CRC/revision validation, legacy calibration imports,
  adjacent-sector preservation, and existing optical/button/late-loop guards.
- UF2 block identity, address sequence, banner, artifact/build equality and hash.

Physical validation remains pending. The left wheel also stalled at35% in the
latest V1 log; fixing right-wheel assistance does not establish available
traction, supply capacity, or freedom from sensor/chassis drag.

V1 artifact and proven V9 junction/V11 line-following sources are preserved.
