# Motor-disabled IR probe v1 — isolate integration console freeze

Use `../firmware/buddy23_ir_sensor_probe_v1.uf2` temporarily to verify raw IR
readings before any combined moving test. This is NOT the coordinator test.

User evidence: integration V2 restores the COM port and initially prints Ready /
Idle sensor lines, but printing later stops. Flow control is None. Reported
inability to see typed h is not conclusive because local echo is disabled.
No root cause of this second freeze is established yet. Do not calibrate from
an old frozen raw=0 value or assume the sensor is defective.

This small image uses the same Pico SDK2.2.0 USB console path as the working
V6 motor bench. It omits the kernel, encoder/barcode GPIO interrupts, motion,
nav and S3 tasks. GP8–11 are SIO outputs driven LOW before USB initialization
and remain low. There are no movement commands; buttons cannot start a run.
GP6=L, GP7=R, GP19=side/barcode, raw electrical levels only. Internal sensor
pulls are disabled so the powered modules provide the readings.

## First check

1. Power bank disconnected; flash through computer USB using BOOTSEL.
2. Open current COM port in PuTTY at115200, flow None.
3. Confirm BUDDY23 IR SENSOR PROBE v1 / MOTOR OUTPUTS HELD LOW banner.
4. Wait60 seconds WITHOUT adjusting the sensors. seq and ms should keep rising.
5. Send h if desired: it prints help; the typed letter need not echo.
6. If continuous, keep normal mat height and compare rawL on white and black.
   Then repeat rawR. Send actual pairs without interpreting 1 as black yet.
7. If it freezes, note last seq/ms, whether h returns help, and whether COM
   disappears. This distinguishes sensor/physical USB from integration-specific
   scheduling/interrupt/console causes; success here alone doesn't locate the
   integration bug.

Adjust the sensor potentiometer only with current readings updating, and in
small steps. Don't adjust lift height to force a particular level. No moving
test until black/white response, left/right mapping and line geometry are agreed.

## Build record

Pico W/WH RP2040, SDK2.2.0, GCC14.2.1, CMake/Ninja Release. Full build with
-Wall -Wextra -Werror PASS. Picotool verifies pico_w / USB stdin+stdout.
ELF text37072, data0, BSS4028. No physical freeze/stability result yet.
UF2 SHA256:

```
5629fc9e6bd0c782edb0f49ddc82ebb6307b8ba7179eb953af33a26fadc4c66f
```

Build from this directory with CMake/Ninja, supplying PICO_SDK_PATH,
PICO_TOOLCHAIN_PATH and picotool_DIR as for the dual-wheel SDK bench.
The integration V2 code remains available, pending freeze diagnosis.
