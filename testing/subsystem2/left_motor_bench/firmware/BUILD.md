# Firmware build verification

Built on 19 October 2026 for RP2040 Pico W/WH.

- Pico SDK: `2.2.0`, commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`.
- TinyUSB: `86ad6e56c1700e85f1c5678607a762cfe3aa2f47` (SDK-pinned submodule).
- Arm GNU toolchain: 14.2.Rel1 / GCC 14.2.1.
- CMake 4.4.4; Ninja 1.13.2.
- Official picotool: 2.2.0-a4 Windows package.
- Target: `pico_w`; release build; USB stdio only; no wireless library linked.
- Initial full CMake compile: **PASS** (88 build steps). Version 3 CMake
  reconfiguration, compile and UF2 generation: **PASS**.
- Application compiler checks: `-Wall -Wextra -Werror`.
- Native logic tests: **PASS**, MinGW GCC 9.2.0, C11, warnings treated as errors.
  Version 2 includes elapsed-time/first-stop-cause assertions for GP20, GP21
  release, timeout, serial stop and USB disconnection.
  Version 3 additionally tests both wheel selections, the unselected motor
  staying off, duty bounds, and invalid drive selections.
- `picotool info -a`: confirms family `rp2040`, board `pico_w`, USB stdin/stdout.
- ELF size report: text 42,736 bytes; data 0 bytes; BSS 4,164 bytes. This is
  firmware build information, not a before/after minimum-bit RAM comparison.
- Version: `Buddy 2 SINGLE WHEEL bench v3`; 3-second maximum pulse, default 25%
  PWM; selectable 15%/25%/40%; left or right, never both.
- UF2 file length: **77,312 bytes** (includes UF2 framing).

SHA-256 of `buddy2_left_motor_bench.uf2`:

```text
c83b87708cdc64a550da3c0126bdb3278165842397419378ae5083fdef08f25c
```

The UF2 was copied from the build of the adjacent `single_wheel_v3.c`, `bench_logic.h` and
`CMakeLists.txt`. Rebuild and update this record when changing the source; the
checked-in UF2 does not update itself. SDK configure warnings about absent BLE,
CYW43 and lwIP submodules are expected for this USB-only diagnostic.

**Version 3 has not been tested on the physical robot.** The user supplied
version 1 serial logs showing both encoder channels responding after correcting
the GP0 connection, opposite count directions for the two drive directions, and
drive returning off. Version 2 logs also reported GP21 release at 526 ms and
GP20 stop at 265 ms. Those logs do not verify version 3, the right motor path,
electrical ratings or precise stop latency. No robot was flashed or actuated by Codex. Record real
results in `../TEST_RESULTS.md` with actual serial logs.
