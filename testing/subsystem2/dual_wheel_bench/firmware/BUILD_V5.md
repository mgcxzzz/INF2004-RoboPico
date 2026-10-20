# V5 build verification — 20 October 2026

- Target RP2040 Pico W/WH, Pico SDK 2.2.0, Arm GCC 14.2.1, Release.
- Full compile/link/UF2 conversion passed with -Wall -Wextra -Werror.
- Native helper tests passed: existing geometry/PI/stop/offline tests plus
  boot two-button hold/release/one-shot/USB cancellation and CRC known vector
  with corruption detection. These tests do not execute physical flash hardware.
- picotool confirms buddy2_dual_wheel_v5, pico_w and USB stdin/stdout.
- Symbol check: USB stdio present, no UART stdio initialization.
- ELF text 50,704 bytes; data 0; BSS 10,748.
- Firmware end 0x1000b614, below reserved log start 0x101ff000.
- Latest complete run saved after settling, outputs off, one erase/program.
- Saved format includes version, bounds check and CRC32; invalid records rejected.
- Core 1 unused. Flash writes mask interrupts only while motor outputs are off.

SHA-256 of buddy2_dual_wheel_v5.uf2:

```
2be99b7a82f857409a95b877b3237ff35b9f4f1ee7106e112d66421442f662e2
```

Source: adjacent main.c, dual_logic.h, log_store.h, CMakeLists.txt and the V3
quadrature helper. Build with CMake/Ninja using the same SDK paths as V4.
Physical power-bank button start and flash recovery are awaiting user testing.
V4 UF2 and its BUILD.md are historical and unchanged.
