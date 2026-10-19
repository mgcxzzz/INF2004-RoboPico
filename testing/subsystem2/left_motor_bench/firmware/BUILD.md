# Firmware build verification

Built on 19 October 2026 for RP2040 Pico W/WH.

- Pico SDK: `2.2.0`, commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`.
- TinyUSB: `86ad6e56c1700e85f1c5678607a762cfe3aa2f47` (SDK-pinned submodule).
- Arm GNU toolchain: 14.2.Rel1 / GCC 14.2.1.
- CMake 4.4.4; Ninja 1.13.2.
- Official picotool: 2.2.0-a4 Windows package.
- Target: `pico_w`; release build; USB stdio only; no wireless library linked.
- Full CMake compile and UF2 generation: **PASS** (88 build steps).
- Application compiler checks: `-Wall -Wextra -Werror`.
- Native logic tests: **PASS**, MinGW GCC 9.2.0, C11, warnings treated as errors.
- `picotool info -a`: confirms family `rp2040`, board `pico_w`, USB stdin/stdout.
- ELF size report: text 40,728 bytes; data 0 bytes; BSS 4,088 bytes. This is
  firmware build information, not a before/after minimum-bit RAM comparison.
- UF2 file length: **73,728 bytes** (includes UF2 framing).

SHA-256 of `buddy2_left_motor_bench.uf2`:

```text
4ac98d3b7ee67397e4655a2442414c9d0711af743848605118c88df32cfe5e44
```

The UF2 was copied from the build of the adjacent `main.c`, `bench_logic.h` and
`CMakeLists.txt`. Rebuild and update this record when changing the source; the
checked-in UF2 does not update itself. SDK configure warnings about absent BLE,
CYW43 and lwIP submodules are expected for this USB-only diagnostic.

**No robot was flashed or actuated by Codex. Hardware timing, voltage, wiring,
motor operation and encoder behaviour remain unverified.** Record those results
in `../TEST_RESULTS.md` with actual serial logs.
