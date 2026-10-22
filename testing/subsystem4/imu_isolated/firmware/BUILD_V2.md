# Isolated IMU V2 build

Built 22 October 2026 after the V1 log demonstrated no successful IMU communication.

- Artifact: `buddy4_imu_isolated_v2.uf2`
- SHA-256: `ce310a120bad2e8b5ace6a9a1c30d89a158ca1a53510f0ef77271570ed265a87`
- 92672 bytes; 181 sequential RP2040 blocks; payload ends at `0x1000b500`.
- ELF flash end `0x1000b49c`; final hump/junction record sectors excluded.
- Arm size: text 50328, data 0, BSS 4316 bytes.
- Same Pico SDK 2.2.0 / Arm GCC 14.2.1 / Pico W Release configuration, warnings
  as errors, USB stdio and no UART stdio, PWM setter or flash range-write symbols.

Changes: explicit ID-read flags, no-device state, initialization error reporting,
I2C failure phase and return code, scan error classes, literal NA for invalid or
non-finite output, NO_SAMPLE for never-populated raw data. Register settings,
I2C pins, absence of actuator control and saved-calibration protection unchanged.
V1 UF2 is retained with its original hash.

Host tests passed for all previous decoder, configuration, lock/freshness and
reference cases, plus missing-ID flags and invalid/NaN/infinite formatter inputs.
The formatter emits NA without passing non-finite values to snprintf. UF2 block,
banner, identity, hash, compiled-artifact equality and flash-bound checks passed.
These software checks cannot establish the physical reason for a missing module.

No V2 hardware result is available yet. Check physical wiring/power before
attempting any gravity reference capture.
