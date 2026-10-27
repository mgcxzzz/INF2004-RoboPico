# Interface V1.3 test build — 2026-10-27

V1.3 is a test candidate. V1.2 failed physically; V1.1 is the comparison baseline. See FORWARD_EFFORT_V1_3.md. Tests establish functional/model checks, not physical smoothness.

- 24 host executables passed:20 current/retained baseline checks and4 explicitly labelled archived V1.2 diagnostics. New forward checks cover ordinary/assisted transitions, overspeed, immediate zero/inner coast, bounded one-shot aid, six changing-load wheel models and six motion/optics/load-selector models. Retained API/ownership/clock/storage/Buddy5/provider/Code39/turn/search checks pass.
- Default, unpacked-reference and opt-in Buddy5 example Pico W Release builds pass `-Wall -Wextra -Werror`. Pico SDK2.2.0, ARM GCC14.2.1, local CMake/Ninja, USB CDC.
- `verify_firmware.py` checks57 snapshot files/four documented harness/record adaptations, immutable V12.2/V1/V1.1/V1.2 artifacts, current UF2 SHA256/blocks, embedded markers and flash bounds.

| ARM build | text | data | BSS |
|---|---:|---:|---:|
| OriginalV12.2 |127128|0|215408|
| V1.1 comparison |150832|0|222064|
| V1.2 failed experiment |155432|0|222112|
| V1.3 packed |154120|0|222088|
| V1.3 same-feature unpacked |152912|0|227048|
| V1.3 Buddy5 example |156848|0|222120|

Shared-field packing saves4960B BSS like-for-like. Two ForwardEffort states add24B overV1.1. This is partial minimum-bit evidence, not whole-firmware compliance.

`__flash_binary_end=0x10024a0c`; padded UF2 end `0x10024b00`. `__bss_end__=0x20037818`, `__StackLimit=0x20040000` leave34792B between symbols; this is not a measured worst-case heap/stack margin. Independent coordinator0x1af000/raw0x1b0000/motion0x1d0000 log banks remain above the image.

Artifact: `firmware/buddy23_interface_v1_3.uf2`

300544bytes; 587 RP2040 blocks, contiguous256-byte payloads.

SHA256 `80a00750ee132df59d749203c404a62c0e5bcb70db1bddf1c9976b8c4cae55e4`.

Motion record21 retains the64848-byte layout and imports older calibration/records. Downgrades cannot read a newly saved21 record; verify calibration. Public API/audit wire formats are unchanged. The CMake target remains `buddy23_interface_v1`.

The first physical comparison is profile1 on the same straight and bend, then barcode/junction and only later profile3. The established fixed-speed probe need not be repeated. No physical improvement is claimed before this test.
