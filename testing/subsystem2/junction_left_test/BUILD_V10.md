# Junction left V10 build, 2026-10-22

Artifact: `firmware/buddy23_junction_left_v10.uf2`

- SHA-256: `4396efdc11dde83e2adb94164e9301f64fa2f3facded6b8c18f7cefc0ca9b652`
- 124416 bytes;243 sequential RP2040 UF2 blocks,256-byte payloads.
- Payload `[0x10000000,0x1000f300)`; ELF end `0x1000f22c`.
- No image writes in hump-record0x101fe000 or junction-record0x101ff000 sectors.
- Arm size: text66088,data0,BSS12232 bytes. USB stdio present;UART stdio absent.
- Pico SDK2.2.0, Arm GCC14.2.1, Pico W Release,C11,warnings as errors.

V10 is V9 with a1.20x final PWM multiplier in all drive phases. Physical limits:
forward300,sustained turn420,start pulse480 permille. The pulse still ends at
the existing150ms maximum or earlier measured-speed termination. Both main
commit and independent1ms ISR use scaled caps; a stale peak is bounded at its
original deadline and repeated ISR calls cannot multiply the output again.
Zero/off wheels and signed bridge polarity are preserved. Logs show physical
PWM. PI, speed requests, phase logic, geometry,6s alignment/180mm allowance,
10s follow, and stop guards are unchanged. No hump-specific assist is added.

Passed: complete existing `test_junction.c` V9 regression suite, plus
`test_boost.c` comparing exact1.20x output over straight/turn/reverse/zero/
alignment/exit transitions; matching PI state; bridge/polarity sweep;
300/420/480 caps; peak boundary at deadline; repeated timer clamping.
UF2 verifier checks family, addresses, banners, artifact/build equality and hash.

Record ABI is unchanged; V9 calibration/log remains readable and retains its
old revision label. New V10 runs are labelled10. A later V9 rollback needs
recalibration after a V10 save because the old validator rejects revision10.
Physical V10 tests are pending.

V9 artifact retained with unchanged SHA-256:
`4a2e3367d3a86020cc98af9fcc2ca532b2c3fd27a07b7057d482b9a57d35e871`.
