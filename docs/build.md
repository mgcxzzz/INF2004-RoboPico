# Build main's Buddy1–4 RTOS release

Ready-to-flash UF2 and full bundle are in
firmware/integration/buddy1_4/artifacts/v0_2_2_rtos/. Building is optional.
Main uses micro T-Kernel SMP with the Pico SDK's low-level USB/radio/lwIP
libraries; it is not the SDK CMake superloop. The build manifest pins actual
compiler and SDK commits. Populate the SDK's USB/Wi-Fi/lwIP submodules.

With ARM GNU gcc/make available and PICO_SDK_PATH set, from build_make/:

```bash
make -j4 APP=rtos_buddy1_4 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0
```

Output: `build_make/robopico_buddy1_4_rtos_v0_2_2.uf2`.
The Windows workspace helper is `tools/rtos_migration/build_release.sh` from
Git Bash; it uses the existing sibling .buddy2-tools installation. Other
machines can use the make command with their own toolchain/SDK paths.

APP=rtos_imu_wifi remains the USB-supervised Step5 checkpoint; the production
profile additionally defines APP_RTOS_RELEASE. The package gate requires its
wireless export/runtime symbols, source fingerprints, reserved-flash bounds,
static RAM headroom and checked stack frames. No embedded Wi-Fi password.
Settings and calibration reside beyond the code's enforced1MiB flash ceiling.

[The old SDK/CMake build](sdk_reference_build.md) is retained for comparison.
The SDK artifacts at v0.1 are historical, not the current main runtime.
