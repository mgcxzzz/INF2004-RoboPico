# v4 build verification, 20 October 2026

- Target: RP2040 Pico W/WH, standalone Pico SDK, release build.
- Program: `buddy2_dual_wheel_v4` / banner `DUAL FEEDBACK bench v4`.
- Pico SDK 2.2.0, commit a1438dff1d38bd9c65dbd693f0e5db4b9ae91779.
- SDK-pinned TinyUSB; Arm GCC 14.2.1 (14.2.Rel1), CMake 4.4.4, Ninja 1.13.2.
- Full firmware compile, link and UF2 conversion: **PASS**, 88 build steps.
- Application compile flags include `-Wall -Wextra -Werror`.
- Native C11 helper/model tests with warnings treated as errors: **PASS**.
- Tested: measured geometry/sign conversions, driver polarity and PWM cap,
  command targets, hold/rest/stop/time-limit/control-stale guard, offline arming
  one-shot and cancellation paths, bounded PI output and simple model tracking.
- `picotool info -a`: family `rp2040`, board `pico_w`, USB stdin/stdout.
- Linked symbol check: `stdio_usb_init` present; no `stdio_uart_init` or `uart_init`.
- ELF size report: text 48,528 bytes, data 0 bytes, BSS 6,648 bytes. This is build
  metadata, not a minimum-bit before/after comparison.
- UF2 file size: 89,088 bytes, including UF2 framing.

SHA-256 of `buddy2_dual_wheel_v4.uf2`:

```text
029d00db6cac401288acd7aa3813f6d2553bf8e5eec7f49ac2c3c494068d50ad
```

Built from adjacent `main.c`, `dual_logic.h`, `CMakeLists.txt` and the existing
v3 quadrature decoder in `../left_motor_bench/bench_logic.h`. If source changes,
rebuild the UF2 and update this record. Source/version is frozen by the commit
containing this binary. BLE/lwIP/CYW43 submodule configure warnings are expected
for this USB-only build.

**No physical v4 test has been performed.** The model is not evidence of motor
tuning, electrical safety, run accuracy, stop latency or final RTOS compliance.
Perform raised-wheel checks first; preserve actual logs and external distance/
angle measurements. V3 hardware evidence is recorded separately and does not
validate this new controller or offline mode.
