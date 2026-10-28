# Build the UTK Drive V1 image

The release artifacts are the drive image
`firmware/buddy123_utk_drive_v1.uf2` and the accompanying
`firmware/buddy123_utk_drive_v1.zip` bundle. The build helper produces the
profile-named UF2; the release staging step copies it to the generic filename.

## Windows prepared workspace

Use Git Bash and the prepared `.buddy2-tools` directory beside the repository.
The helper needs:

- `pico-sdk/` with `pico_sdk_init.cmake` and its required CYW43/lwIP
  submodules,
- `arm-gcc/bin/arm-none-eabi-gcc.exe` and companion Arm binutils,
- `make-msys/usr/bin/make.exe`, and
- Git Bash (`bash`, `cygpath`, `/usr/bin`) with a host C++ compiler for the
  local `elf2uf2` converter.

From Git Bash at the repository root:

```bash
cd testing/subsystem2/interface_utk_drive_v1/source/build_make
bash ./build_wifi_msys.sh
```

The helper configures `PICO_SDK_PATH` and `PATH`, then runs the build with
these exact settings:

```text
WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0
```

The expected output is
`source/build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`. From
`source/build_make`, stage the generic UF2 name with:

```bash
mkdir -p ../../firmware
cp mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2 ../../firmware/buddy123_utk_drive_v1.uf2
```

The helper does not create the release ZIP. Use the supplied bundle for the
Windows release workflow. Firmware building does not require Wi-Fi
credentials: the existing WCF1 settings sector at `0x101AA000` is reused by
the running image and is provisioned separately with `Configure WiFi.cmd`
only when the saved network or broker address must change.

## Other tool installations

Install the Pico C SDK and its submodules, GNU Arm Embedded toolchain, Git
Bash, GNU make, and a host C++ compiler. Export `PICO_SDK_PATH`, add the
compiler and make directories to `PATH`, and use the same settings from
`source/build_make`:

```bash
make -j8 WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0
```

See [`source/README.md`](source/README.md) for Pico SDK setup. Keep the
profile-named build output for comparisons; `buddy123_utk_drive_v1.uf2` is the
release-friendly artifact name, not a different build configuration.

## Current build status

The current firmware build completed and generated a UF2. Reported memory use
is 475,232 bytes of text, 0 bytes of data, and 225,140 bytes of BSS. The image
has not yet been flashed or tested on the physical robot. The dashboard's
schema and browser smoke checks are separate software checks and do not
validate motor or sensor behavior.
