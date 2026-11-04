# Step5 v0.1.3 — isolate IMU scheduling and retain startup calibration evidence

Experimental micro T-Kernel migration build, Buddy1–4 only. Main is unchanged.

The current main Buddy1–4 integration UF2 is a Pico SDK build (CMake imports
the SDK, links pico_multicore and pico_cyw43_arch_lwip_poll). Its IMU ran in
the core0 control superloop while network work ran on core1. Earlier RTOS
network/motor steps passed, but they did not establish complete IMU/ramp
schedulability. Step5 initially put IMU below USB/Wi-Fi on core0; that changed
the execution conditions despite keeping the original processing rules.

v0.1.2 still recorded17/20 late acquisitions in two settled windows, with
recent maxima34.831/37.950ms. Reads, timeouts, FIFO and logging were clean in
those windows; main control remained100Hz. There was also a stationary cold
calibration timeout, recovered by Reset. Short logging locks alone did not
resolve jitter. This build tests task isolation and gathers direct evidence
for any remaining scheduling/calibration issue.

## Changes

- Sole I2C1/IMU task on core1 priority16. Safety6, navigation10 and motion14
  retain priority over it. Core0 USB/Wi-Fi retain their ownership/priorities.
  IMU_EXEC information is included in IMU_CAL as actual core/configured priority.
- Per-epoch IMU_CAL progress/best, exact range/speed restart counts/reasons,
  rejected spans and read/timing exclusions are retained after fault. This
  avoids losing all startup evidence when serial capture opens late.
- Exact restart reasons come from passive RTOS-only hooks in the original
  calibration decision path; algorithm/threshold decisions are unchanged.
  A stationary report does not prove noise-free samples; the new spans can
  distinguish measured fluctuations from an unexplained scheduling deadline.
- NET_SERVICE radio/link, lwIP timer and MQTT bridge durations are measured
  by the core0 kernel owner and copied into a diagnostic mailbox. Durations
  include preemption/waits, rather than representing exclusive CPU time.
- Capture summaries retain calibration faults/epochs and network timing.

Line following, front calibration, barcode/junction decisions, ramp detector6,
motor limits, nav/API routing, USB supervision and existing binary MQTT
formats are retained. No IMU auto-retry or automatic movement is introduced.
The20ms period,30ms timing guard,100-sample calibration window and10second
calibration deadline are unchanged.

## Checks and retest

37 production-C regression programs plus Python capture/export/provisioning
checks. Actual service/processor tests cover exact axis/speed restarts,
read interruption, successful calibration, retained10s timeout evidence and
fresh recovery. ARM build, stack/RAM, UF2 flash bounds and source audit apply.
The processor/service/driver still preprocess identically to main for SDK
builds with the RTOS-only option disabled. Hardware timing remains unverified.

Use START_HERE.md for a stationary cold boot before any Reset, followed by
two settled windows and a stopped Reset. Confirm IMU_CAL core=1 priority=16,
NET_SERVICE core=0 and main100Hz timing. Record any initial fault before
Reset; the retained diagnostic counters should show why progress was lost.
Send the entire rtos_connected_session folder. No driving/video/export tests
yet; a clean stationary result precedes the flat and one-sided ramp tests.
