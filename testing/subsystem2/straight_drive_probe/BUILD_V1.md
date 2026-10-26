# Straight drive probe V1 build evidence

Built2026-10-26 for Pico W/RP2040, SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, C11 -Wall -Wextra -Werror.

- Artifact firmware/buddy23_straight_drive_probe_v1.uf2:179712 bytes,
 351 sequential RP2040 blocks, padded end0x10015f00.
- SHA256:a55f741d3ae48af543565ebd66b8e444cd09f46b993c82e5121df2f7e18ddee8
- Linker end0x10015e5c; text93784/BSS215548; BSS end0x20035cfc,
 stack limit0x20040000. Uses existing flash banks/64848-byte V15 record.
- UF2 magic/family/sequence/addresses/trailers/markers/package equality pass.
- Normal V15 rebuilt after adding conditional adapter hooks and matched
 published SHA25642143adba24e31670ae145cb53d3145ad2b4991e7d2c0834ab66b416b4fbf1b2.
 Prior V3..V14 binaries also passed their unchanged hash checks.
- test_probe passed: fixed45/45 across six optical states and three selected
 profiles; no optical recovery/route/ramp/straight-PWM smoothing; invalid
 calibration rejected;4s deadline boundary; manual GP20 stop; original
300-PWM normal breakaway on either wheel. Actual physical results pending.

The diagnostic prints its own help, forces profile1, logs decision25 and saves
key=D. It preserves normal runtime guards and boot motor-OFF. Its STOP on
WHY_LIMIT is labelled4-second fixed-speed diagnostic complete. Direction and
invalid-encoder faults can still end it earlier; a stationary encoder warning
alone does not stop the drive. No line signal is required after launch.

An initial SDK configuration attempted to rebuild picotool with the system
host compiler and failed; reconfiguration to the already packaged picotool
completed the final build. No failed/dependency build tree is committed.
