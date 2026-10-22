# Circuit Left V1 build evidence

Built 2026-10-22 for Raspberry Pi Pico W / RP2040 using Pico SDK 2.2.0,
Arm GNU toolchain, CMake/Ninja, Release C11, `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_left_v1.uf2`

- Size: 136,192 bytes; 266 sequential UF2 blocks, 256-byte payloads.
- SHA-256: `214bb3fbb8acab7df1be4a5c71889619520400f2f94245b22eb50138330ecdb6`
- Padded image end: `0x10010a00`; linker flash end: `0x1001099c`.
- Circuit log reserved at `[0x101e0000,0x101f0000)`.
- Earlier hump and junction records at `0x101fe000` and `0x101ff000` are untouched.
- ELF size: text 72,088 bytes, BSS 129,888 bytes. BSS end `0x20020d40`,
  stack limit `0x20040000`; sufficient space remains below the stack.
- USB serial symbols `stdio_usb_init`, `tud_cdc_n_write`,
  `tud_cdc_n_write_available`, `tud_cdc_n_write_flush` verified in ELF.
- Delivered UF2 equals the CMake build output byte for byte.

## Checks executed

Host:

```powershell
gcc -std=c11 -Wall -Wextra -Werror test_circuit.c -lm -o test_circuit.exe
./test_circuit.exe
```

Result:

```text
PASS:12 repeated turns/profile, long follow, edge/clearance gates, approach guard, branch timeout, PWM caps, legacy/CRC and ring wrap.
```

The motion simulation runs twelve left turns per profile, crossing the old
single-test time and distance ceilings. It verifies completion and rearming
over three four-turn route cycles. It uses ideal wheel response and synthetic
optical readings; it does not establish physical braking, friction, sensor
geometry or full-course detection performance.

Arm build completed with warnings treated as errors. UF2 verification:

```powershell
python verify_firmware.py
```

Result: all sequential blocks are RP2040 family `0xe48bff56`, with expected
V1 banner and summary strings; no payload reaches the reserved flash area.

## Build configuration

From this directory, with the toolchain on PATH:

```powershell
cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 8
```

Set `PICO_SDK_PATH` to the installed SDK and `picotool_DIR` if required by
the local toolchain. Existing working logic is included directly from sibling
test folders; build from the complete repository.

Physical lab validation remains pending. Start with profile 1 and verify
both visits to the same junction before moving to profiles 2 and 3.
