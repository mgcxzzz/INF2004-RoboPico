# Step5 v0.1.1 stationary acquisition update

This is an experimental migration-branch test package, not a main release or
completed Buddy1–5 integration. Use the stationary retest in START_HERE.md
(or this folder's README.md). No new front calibration or driving test yet.

The v0.1 capture showed calibrated Idle, CLOCK PASS and approximately49.91Hz
IMU processing, but104 failed acquisitions and118 additional combined missed
counts over225seconds. The main motor-control timing stayed healthy. An
accepted stopped Reset retained the old idle IMU calibration and counters.

The unchanged driver was reproduced rejecting a completed transaction when
simulated RTOS preemption occurred between a pending-status read and the
2ms deadline check. This establishes a software race; the old capture cannot
prove every failure had that cause because no abort/timeout-stage evidence
was recorded.

Changes:

- RTOS-only I2C transport rechecks status before declaring timeout, uses
  bounded2ms waits per hardware phase and never queues commands after a
  failed wait. Abort source is captured before clearing it. No long critical
  section or interrupt masking surrounds I2C work.
- Fixed20ms release waits at core0 priority13, above application bridge and
  reporting, below USB and Wi-Fi. The100Hz core1 motor owner is retained.
- `IMU_ACQ` keeps full32bit lifetime failed-read, late-gap and FIFO-overrun
  counters, max gap/wake lateness, init epoch and calibrated ready epoch.
- `IMU_I2C` distinguishes TX/STOP/RX timeout, hardware abort/source/address,
  rescued deadline checks and maximum transfer wall time. Expected optional
  magnetometer probes can NACK during init; compare settled-window deltas.
- A stopped dashboard Reset/USB x now physically reinitialises the IMU and
  recalibrates even from Idle. The new epoch/ready_epoch report completion
  separately from the accepted command. Full diagnostics remain lifetime
  counters across this reset; the legacy six-bit MQTT field can wrap/restart.
- New capture summaries retain acquisition/transport snapshots. Existing
  binary MQTT formats, dashboard, line-following calibration, barcode logic,
  ramp settings, motion limits and nav/API routing are retained.

Software validation:35 production-C regression programs, Python capture/
export/provisioning tests, actual transport preemption/NACK/stuck-bus tests,
actual task/service reset/calibration test, ARM build, UF2/RAM/stack/source
audit. Both changed reference IMU source files preprocess identically to the
main SDK baseline with APP_RTOS_IMU disabled. Hardware success is pending.

Keep the v0.1 bundle for rollback. The new bundle includes the v0.1.1 UF2,
source, dashboard, updated capture scripts and instructions. Flashing the UF2
preserves the saved Wi-Fi and front calibration sectors; reprovision only if
the network or broker address changed. No automatic movement is introduced.
