# Step5 v0.1.2 — logging critical sections and recent timing

Experimental migration-branch test. Buddy5 remains absent; main is unchanged.
Perform the two-window stationary test in START_HERE.md before driving/ramp.

v0.1.1 hardware results established zero new read failures, timeouts, aborts
and FIFO overruns across two settled windows, and successful Reset calibration.
However,9/3053 and10/3101 acquisitions exceeded the unchanged30ms timing guard.
The old158ms maximum predated those windows and could not describe recent gaps.

A concrete scheduling concern was identified in the console: tm_printf held
an interrupt-preserving SMP lock throughout formatting and fragmented output.
This could delay higher-priority work on the same core, including IMU/Wi-Fi.
The capture did not prove every gap came from this path; hardware retest is
required to determine whether this correction resolves the remaining jitter.

## Correction

- Step5's USB-only formatter now uses a768-byte local buffer and preserves
  the caller's interrupt state during formatting and CRLF expansion. One
  completed record is copied under the USB ring lock. No host wait is added.
- A full/oversized message is dropped whole and counted, preserving record
  integrity. No byte prefix is silently presented as a complete diagnostic.
  Existing application formats, including the475-byte line-export header,
  fit this bound. UART/other RTOS profile paths retain the original code.
- LOGGING reports atomic mode, record/drop/oversize counts and maximum ring
  copy critical-section time. This duration excludes formatting and lock wait.
- IMU_TIMING adds last acquisition gap and last late gap/read number. Current
  and previous10-second buckets expose maxima, reads and late counts. They
  continue across Reset; use settled-window deltas and subsequent buckets.
  Bucket counters use9bits, enough for at most501 releases at50Hz. Lifetime
  counters and maxima remain available and are not silently cleared.
- Capture summaries retain timing buckets and logging diagnostics.

The v0.1.1 I2C correction, sole IMU bus owner/task priority, fresh stopped
Reset/calibration, main100Hz motor owner, nav/API routing, kernel-only MQTT,
IR calibration, barcode decisions, ramp logic and motor limits are retained.
No timing guard is loosened and no automatic movement is introduced.

## Software evidence and next hardware check

36 production-C programs plus Python capture/export/provisioning checks.
The actual formatter and ring policy are exercised for nested producers,
CRLF conversion, wrapping, exact capacity, full/oversize whole-record drops
and no whole-format lock dependency. Actual transport tests exercise recent
bucket rotation and500-read count boundaries, along with prior timeout tests.
Both changed console C files preprocess identically to v0.1.1 with the Step5
option disabled. The package checks flash bounds, RAM, stack frames and
required linked functions. These checks do not measure real hardware latency.

Download the new complete bundle for its updated capture summary. BOOTSEL
flash the v0.1.2 UF2. Wi-Fi/front calibration sectors are retained. Start the
dashboard and connected capture, wait10seconds for calibrated Idle and
LOGGING atomic=1, record60seconds, click stopped Reset once, wait for the new
ready_epoch and10seconds settling, then record another60seconds. No Start,
IR recalibration, video or stored line/barcode exports are needed yet.

Send the full rtos_connected_session folder. Failed/timeout/abort/FIFO-overrun
counters and settled LOGGING dropped/oversize counts should stay constant.
Late_gaps should stop growing; if they do grow, the IMU_TIMING fields now
record recent durations so we can identify the next scheduling action.
