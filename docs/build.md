# Build RoboPico RTOS

## Current release — v0.4.3

Use the [download and build manifest](../firmware/releases/v0.4.3/).
Check out the manifest's source commit. Use Arm GNU GCC 14.2.1 and the pinned
Pico SDK commit with its TinyUSB, CYW43 and lwIP dependencies. Set
`PICO_SDK_PATH`, then run from `build_make/`:

```bash
make -j8 APP=rtos_buddy1_5 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0
```

On the project Windows host, run
`bash tools/rtos_migration/build_release.sh APP=rtos_buddy1_5` from the root.
Compare the resulting `robopico_buddy1_5_rtos_v0_4_3.uf2` with the published
SHA-256. See [release verification](../testing/integration/logs/2026-11-21_v0.4.3/README.md)
and the [test index](../testing/README.md).

## Archived v0.2.13 release

For the v0.2.13 reference build, see the
[archived UF2 and dashboard bundle](../firmware/integration/buddy1_4/artifacts/v0_2_13_combined/).
The manifest records the Arm compiler and Pico SDK revision. Populate that
SDK's USB, Wi-Fi and lwIP submodules before rebuilding.

From `build_make/`, with GNU Arm GCC and make on PATH and `PICO_SDK_PATH` set:

```bash
make -j4 APP=rtos_buddy1_5 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0
```

The result is `build_make/robopico_buddy1_5_rtos_v0_2_13.uf2`. On the project
Windows build host, use the pinned toolchain in Git Bash from the repository root:

```bash
bash tools/rtos_migration/build_release.sh APP=rtos_buddy1_5
```

`APP=rtos_buddy1_4` builds the Buddy 1–4 v0.2.12 profile. It reproduced the
published rollback UF2 byte for byte in validation.

To package v0.2.13 on the Windows build host, commit the reviewed source first,
build the combined profile, then run:

```bash
python tools/rtos_migration/package_combined.py
```

The packager checks the C standard and formatter, linked features, reserved
flash bounds, application stack frames, kernel allocation budget and ZIP
checksums. Saved Wi-Fi and calibration sectors are above the 1 MiB code limit.

[Offline test procedures](testing.md) · [Historical SDK/CMake build](sdk_reference_build.md)
