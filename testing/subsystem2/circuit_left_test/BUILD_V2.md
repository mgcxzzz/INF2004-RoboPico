# Circuit Left V2 build evidence

Built 2026-10-23 for Pico W / RP2040, SDK 2.2.0, Arm GNU toolchain,
CMake/Ninja Release C11, `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_left_v2.uf2`

- Size: 143,872 bytes; 281 sequential UF2 blocks.
- SHA-256: `65f708a5d1157ab8e58e1d9c1256f6f1c747a99ae11639f59cf35e03ed02de71`
- Padded payload end: `0x10011900`; linker flash end `0x1001187c`.
- All payloads remain below the circuit log at `0x101e0000`.
- ELF size: text 75,896 bytes, BSS 129,904 bytes. BSS end `0x20020d50`,
  stack limit `0x20040000`; USB serial init/write symbols verified.
- V1 UF2 preserved unchanged: SHA-256
  `214bb3fbb8acab7df1be4a5c71889619520400f2f94245b22eb50138330ecdb6`.

## Changes

Higher profiles no longer multiply correction targets. Their fast cruise is
qualified by both normalized levels >=400 and difference <=80 for 300 ms.
All ordinary steering thresholds and correcting targets are unchanged from
profile 1. Correcting uses the same profile 1 wheel PI / 30% PWM cap; cruise
compensation is cleared on exit. Requested speed ramp is capped at 50 mm/s
on that transition; actual measured velocity can take time to fall.

After the original rolling-turn optical branch qualification (old line left,
new stripe visible for 30 ms within 75–110 encoder degrees), normal profile 1
curve following begins immediately. At least one front level >=400 for 150 ms
confirms handover. Normal bounded line-loss recovery replaces V1's special
150 ms both-white alignment stop. A six-second / 180 mm handover bound remains.
Explicit junction turn speeds remain progressively faster by profile.

Circuit record layout is unchanged. Both version 1 and version 2 records pass
validation. A restored V1 run retains its V1 label when printed/saved; a new run
is labelled V2. Calibration, profile selection and whole-run summary persist.

## Executed checks

```powershell
gcc -std=c11 -Wall -Wextra -Werror test_circuit.c -lm -o test_circuit.exe
./test_circuit.exe
python verify_firmware.py 1
python verify_firmware.py 2
```

Host result:

```text
PASS:12 turns/profile; full steering error sweep; centred cruise dwell/exit; one-sensor branch handover; bounded recovery; detection, PWM, V1/V2 records and ring wrap.
```

The steering sweep exercises every integer normalized error -1000 through
+1000 at all three profiles against the original profile 1 target function.
It verifies fast cruise only inside the original 80-count straight band after
dwell, no fast cruise for balanced weak signals, immediate boost removal,
bounded both-white recovery, and one-sensor handover followed by the opposite
curve correction. Existing repeated-turn, side gate, speed guard, peak cap,
CRC corruption, calibration import and log ring checks still pass.

Arm build and UF2 checks passed; delivered V2 matches the build output byte
for byte. Simulated wheel response is idealized. Physical braking, oscillation,
curve capture and full-course completion still require the user's lab test.
