# Build the Buddy 1 network proof image

The supplied UF2 is named `firmware/buddy123_utk_network_v1.uf2`. Its build
profile is the dual-core Pico W network image with the USB console and the
micro T-Kernel Wi-Fi/MQTT service enabled. The beginner path below uses the
workspace's prepared Windows tools; it does not need the Pico connected to
build.

## Prepared Windows workspace

Requirements:

- Windows with Git for Windows / Git Bash (`bash` and `cygpath`).
- The workspace dependency folder `.buddy2-tools` beside the repository, with:
  - `pico-sdk/` (contains `pico_sdk_init.cmake` and its required SDK
    submodules, including the Pico W CYW43 and lwIP sources),
  - `arm-gcc/bin/arm-none-eabi-gcc.exe` and its companion binutils, and
  - `make-msys/usr/bin/make.exe` plus a host C++ compiler for the local
    `elf2uf2` helper.

From Git Bash at the repository root, run:

```bash
cd testing/subsystem2/interface_utk_v1/source/build_make
bash ./build_wifi_msys.sh
```

The helper checks for the SDK and compiler, sets `PICO_SDK_PATH` and `PATH`,
then invokes `make -j8` with this exact profile:

```text
WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0
```

The make profile defaults `TEST=0`. The build output keeps a profile-specific
name, normally
`source/build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`. To stage it at
the generic distribution path used by the Windows flashing instructions:

```bash
mkdir -p ../../firmware
cp mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2 ../../firmware/buddy123_utk_network_v1.uf2
```

Run the copy commands from `source/build_make`. Keep the profile-specific
artifact when comparing builds; the generic name identifies its intended
network-proof role, not the compiler flags.

## Building without the prepared tool folder

Install the Pico C SDK and its submodules, GNU Arm Embedded toolchain
(`arm-none-eabi-gcc` and binutils), Git Bash, GNU make, and a host C++ compiler.
The build helper expects `.buddy2-tools` at the prepared-workspace location
above. For another installation, export `PICO_SDK_PATH`, add the compiler and
make directories to `PATH`, then build from `source/build_make` with:

```bash
make -j8 WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0
```

The build also needs the host C++ compiler to produce `tools/elf2uf2` on its
first run. See [`source/README.md`](source/README.md) for Pico SDK setup
details. Network SSID/password and broker configuration are provisioned
separately with `Configure WiFi.cmd`; they are not compiled into the UF2 and
are not needed for compilation.

## Kernel timeout capacity note

The timer-capacity check reported **core 6 / capacity 10** in this snapshot;
the teammate copy reported **core 7 / capacity 11**. Both configurations
reserve `MEMP_NUM_SYS_TIMEOUT = LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4` lwIP timeout
slots. The one-slot absolute difference comes from the two builds' different
lwIP `INTERNAL` counts; compare the
relative reserve and each build's dependency version when reproducing the
measurement. This is a bounded lwIP timeout-pool setting, not a claim about
all micro T-Kernel timer objects.

## Host checks and release verification

From this folder in PowerShell, run:

```powershell
py -3 run_tests.py
py -3 run_lwip_timeout_test.py
py -3 release.py --package
py -3 release.py
```

The protocol/settings tests use the prepared Zig compiler and Node.js. The
allocator regression uses `CC` or the installed GCC/Clang plus the pinned
Pico SDK (`PICO_SDK_PATH` can override its location). This release used
32-bit MinGW GCC 9.2.0 for that test to match RP2040 pointer size and
four-byte pool alignment. The test executes actual SDK allocator/timeouts
code and unrelated timer callbacks are stand-ins; it does not simulate radio
or validate a real handshake. Both the baseline failure and fixed success
must pass. The optional browser test is described in dashboard/README.md.

Packaging requires Git provenance objects, the rebuilt ELF/UF2 and completed
test logs in `build/`; it records upstream/patch hashes, builds the ZIP,
checks RP2040 blocks and reserved flash bounds, and verifies source/bundle
hashes. `BUILD_METRICS.json` records exact compiler and SDK versions. There
are no embedded network credentials. Hardware connection remains untested
until the Pico is flashed and checked against the laptop broker.
