# Buddy 3 isolated IR build from the supplied ZIP

Input: user-supplied `INF2004-RoboPico.zip`, received 19 October 2026.

## Download and run

Flash `buddy3_isolated_ir_usb.uf2` using BOOTSEL. It targets RP2040 Pico/Pico W.
Open the Pico's own USB serial port in PuTTY at 115200 baud, 8N1, no flow control.
It prints a line roughly every 250 ms beginning:

```text
S3 ISOLATED (no motor control): raw=... black=... speed_word=... diag=...
```

- `raw` values are digital levels for left GP6, right GP7, barcode GP19, in that order.
- `black` values are interpreted black/white bits in the same order. The supplied
  driver assumes HIGH means black (`DRV_IR_BLACK_LEVEL=1`); verify this with the
  actual sensors over black and white. Power GPIO-connected sensors appropriately
  for the Pico's 3.3 V inputs.
- The original S3 task and cyclic handler run. No mission-start request is sent,
  no motor controller is provided, and the original odometer hook still returns
  false. Speed proposals normally remain invalid/zero: these are not actual motor
  speeds and this image does not make the car follow a line.
- GP8-GP11 are set to output-low by the wrapper. No PWM motor commands are issued.
- No Buddy 2 v3 keyboard commands are implemented in this firmware. Flashing it
  replaces the previously installed motor test.
- Place the three sensors over white at startup for the supplied S3 self-check.
  That self-check's success depends on the assumed IR polarity being correct.

## What changed to make an isolated build

The uploaded ZIP mixes new S3 source with the old producer/consumer/monitor
template. A normal all-subsystem build is incompatible: app.h declares ER-returning
S3 entry points whereas the new module uses void, and template consumer/monitor
files refer to removed statistics objects.

1. Added `app_program/buddy3_isolated.c`, a small S3-only entry point with sensor
   logging and inactive motor outputs.
2. Added a `BUDDY3_ISOLATED=1` conditional source list to
   `build_make/mtkernel_3/app_program/subdir.mk`. It excludes the unrelated template
   application and compiles only the wrapper plus S3 core, IR driver, barcode
   decoder and weak hooks.
3. Added `-DPROHIBIT_DEF_SIZE_T` for `subsystem3.c` only. That source includes
   standard stddef.h before kernel headers, which otherwise redefine size_t.
   The kernel already supplies this compatibility guard.

**All nine supplied `app_program/subsystem3/*.c` and `*.h` files were verified
byte-for-byte unchanged against the ZIP using SHA-256.** No new motor controller,
Navigation Coordinator or simulated odometer was added. Existing GitHub branches'
application sources were not modified by the compilation.

`isolated_build_additions.zip` contains the added entry point and modified make
fragment with their relative paths. Overlay it on the original supplied project
to reproduce this isolated build.

## Build profile and verification

```sh
cd build_make
make -j8 BUDDY3_ISOLATED=1 SMP=1 CONSOLE=usb_cdc WIFI=none TEST=0 CRASH_REPORT=0
```

Requires Arm GNU toolchain, host C++ compiler, GNU Make and Pico SDK with TinyUSB.
`PICO_SDK_PATH` must point to the SDK. Used Pico SDK 2.2.0 with SDK-pinned TinyUSB,
Arm GCC 14.2.1, GNU Make 4.4.1 and host MinGW GCC 9.2.0. Original build output
`mtk3pico_smp1_usb_cdc.uf2` was copied to the descriptive delivery filename.

- Build/link/ELF-to-UF2 conversion: **PASS**.
- `picotool info`: family **rp2040**.
- USB CDC enabled; UART console/mirror disabled; Wi-Fi disabled.
- ELF size: text 63,847 bytes; data 36 bytes; BSS 22,224 bytes.
- UF2 file size: 128,000 bytes.
- No hardware flash or physical test was performed by Codex.
- The original kernel configuration has a 10 ms timer period while S3 requests
  5 ms polling. Timing behaviour needs review on hardware; it was not changed
  or certified by this compilation.

UF2 SHA-256:

```text
d26603e8e101f7fb9de079ebbd2ef86805e0bdf14e705005bfe54a4bdf21b0e3
```
