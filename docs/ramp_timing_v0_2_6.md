# Ramp snapshot timing correction v0.2.6

**Provisional Buddy 1–4 micro T-Kernel SMP release. Buddy 5 is not integrated.**

## What changed

The control task used its earlier loop-start time to judge the age of an IMU
snapshot copied later. Core0 could publish a valid sample in between. Unsigned
subtraction then made that newer sample appear extremely old, ending ramp
assistance with a false STALE reason.

The ramp bridge now copies the protected IMU snapshot first and reads the current
hardware timer afterward. Its API no longer accepts a caller-supplied clock.
Both timestamps retain their existing full64-bit precision. The original60ms
assistance and200ms speed-hold stale limits remain in effect for real data loss.

The 20-degree-per-second turn check, acceleration-quality checks, height
acceptance, ramp gains, motor caps, speed modes, pickup thresholds, barcode and
line-following algorithms are unchanged. This fix addresses assistance timing;
it does not force an uncertain height to be accepted.

## Install

- [Complete bundle](../firmware/integration/buddy1_4/artifacts/v0_2_6_rtos/robopico_buddy1_4_rtos_v0_2_6_bundle.zip)
- [UF2 only](../firmware/integration/buddy1_4/artifacts/v0_2_6_rtos/robopico_buddy1_4_rtos_v0_2_6.uf2)

Stop the robot and use BOOTSEL to copy the UF2 to RPI-RP2. Saved Wi-Fi and front
calibration are preserved. Keep it flat/still for IMU calibration. Reapply the
RAM barcode map after reboot. No w/l/r/k recalibration is required for this fix.

The v0.2.5 dashboard remains compatible: flashing only the new UF2 is enough to
apply the correction. The complete bundle includes the matching v0.2.6 dashboard
label, helpers and instructions. Hard-refresh if using the new dashboard files;
the car's reported firmware version should say0.2.6 in either dashboard.

Powerbank use and dashboard Start remain supported without USB serial or
Capture connected test. The laptop broker must stay running and the motors need
their normal working power supply. See [setup](setup.md) for Windows/Mac commands.

## Focused physical check

1. Use Baseline45 and confirm applied readback. Keep the dashboard open before
   Start. Wait for fresh calibrated IMU data.
2. Run a short flat approach, cross the same right-wheel hump, then drive on
   level ground for2–3seconds before Stop. Use the previously troublesome edge
   alignment to check the fix. Do not hold or push the robot during the crossing.
3. Before another Start/Reset/reboot, save **Logs & exports → Export all frames
   CSV**, **Export Buddy4 CSV**, **Export event CSV**, **Messages → Save log** and
   stopped **Diagnostics CSV + JSON**. ZIP large files. Add a video showing the
   right wheel and ramp exit. No laptop USB connection is needed.
4. Check that assistance does not repeatedly end STALE while fresh valid IMU
   samples continue. Check the accepted/rejected hump result independently.
   A genuine quality rejection should remain visible. Report any pickup stop,
   control timing fault or new acquisition error.

One successful pass is not a repeatability claim. If the edge pass still rejects
the height, use the new capture to assess acceleration/quality grace after the
timing correction, rather than removing turn or measurement-quality guards.

## RTOS, interfaces and representation

The fix stays in the RTOS ramp adapter. It uses the existing lock-protected
snapshot, shared nav API/coordinator approval and existing motor-output owner.
No IMU or network task writes motor PWM. Stop/Pause/ESTOP and the pickup latch
retain their outer safety gates.

No structs, fields, bit widths, telemetry layouts, queues or log capacities are
added or expanded. Existing explicit wire encoding and reserved-zero bits remain.
The new clock value is a local full-range timestamp, not persistent state or a
new serialized field. This preserves the Tribunal1 data-representation approach;
it is scoped evidence, not a completed whole-project padding/packing audit.

The focused production-C tests cover the newer-sample interleaving, uninterrupted
assistance, exact real stale deadlines, the low32-bit clock boundary, coordinator
ownership/output caps, junction removal and halt. The build manifest records
linked RAM/stack checks and exact source fingerprints. Those software checks do
not certify physical hump acceptance or physical stop latency.
