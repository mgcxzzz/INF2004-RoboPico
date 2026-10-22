# Isolated IMU V1 build

Built 22 October 2026 for Pico W / RP2040.

- Artifact: `buddy4_imu_isolated_v1.uf2`
- SHA-256: `aeaf0d70c2d56de6e952426fd16f2c9c044d226585b32fc509dd16d85cdc6a9f`
- 88576 bytes, 173 sequential RP2040 UF2 blocks, 256-byte payloads.
- Payload range `[0x10000000,0x1000ad00)`; ELF flash end `0x1000ac1c`.
- Reserved hump and junction sectors `0x101fe000` and `0x101ff000` excluded.
- Arm size output: text 48152, data 0, BSS 4300 bytes.
- Pico SDK 2.2.0, Arm GCC 14.2.1, C11 Release, `-Wall -Wextra -Werror`.
- USB stdio enabled; UART stdio disabled. `stdio_usb_init`/`i2c_init` present;
  `stdio_uart_init`, PWM setter and flash range erase/program symbols absent.
- Source inspection: GP8–GP11 are initialized to SIO LOW at application start;
  no subsequent motor GPIO writes, motor command paths, PWM initialization,
  line ADC reads, encoder callbacks, servo or network tasks.

Host test command (MinGW GCC, warnings as errors):

```text
gcc -std=c11 -Wall -Wextra -Werror -O2 test_imu.c -lm -o test_imu
./test_imu
```

Passed:

- Recognized IDs and exact configuration readback; unknown IDs receive no
  configuration writes, partial failures leave the other part testable.
- Accelerometer little-endian sign/scale and magnetic big-endian X,Z,Y layout.
- Magnetic overflow marker and acceleration clipping checks.
- Data-ready versus no-new-data and failed burst reads; lock-only magnetic
  drains are not counted as new samples.
- Welford statistics, gravity tilt geometry and reference acceptance/rejection
  for stable, moving, insufficient, zero-gravity and flagged-error samples.
- `verify_firmware.py`: artifact/build equality, banner, RP2040 identity, all
  UF2 block numbers and address bounds, byte count and SHA-256.

No physical module was accessed by Codex. Hardware ID, gravity response, actual
sample rates, USB longevity and interference remain to be checked by the user.

Rebuild with the configured Pico SDK and Arm toolchain:

```text
cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Paths for Pico SDK, Ninja and picotool must match the local installation. Copy
the built UF2 into `firmware/` and run `python verify_firmware.py` before delivery.
