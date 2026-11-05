# Buddy4 IMU and ramp assistance on micro T-Kernel

Experimental Step5, based on Step4 v0.1.2. Physical acceptance of both the
new gate and this IMU port is pending. Buddy5 and powerbank driving remain
deferred. SDK main and the published Step4 artifacts are unchanged.

Step5 v0.1.4 moves the sole IMU task to core0 priority2, ahead of USB3
and kernel Wi-Fi4. Core1 safety6, navigation10 and motor14 are unchanged.
v0.1.3's core1 priority16 placement passed stationary tests but starved IMU
under motion. `IMU_CAL core` reports the actual executing core and priority.
The 20ms release target remains50Hz. Hardware timing acceptance is pending.

FIFO acquisition has an8ms aggregate elapsed deadline checked on successful
progress as well as wait paths. Initialization has a separate20ms budget;
controller reset-ready polling is bounded2ms. Expiry prevents further I2C
commands and follows existing acquisition/init failure handling. These limits
are not a hard cap on the whole tick: conversion and bounded32-sample
processing follow the read. `IMU_PHASE` separates read/processor/tick wall
time and FIFO batch; `IMU_BUDGET` records exhaustion, excess time, reset
timeouts and whole ticks over8ms. No estimator/ramp threshold was relaxed.

`SIDE_SERVICE` in the line service profiles every due side ADC attempt,
including polls between motor ticks, and decoder/planner work. uint32 totals
wrap modulo32bits (about71.6minutes of accumulated microseconds); own
recording overhead is excluded. All timings include waits/preemption, not
exclusive CPU. Snapshots are copied before printing. See
[compact field domains](../../docs/data_representation.md) and the Step5
build manifest for static RAM growth; binary MQTT payload sizes are unchanged.

Step5 v0.1.2 removes whole-format interrupt masking in the USB-only console:
each tm_printf formats/expands CRLF in local768-byte storage, then uses one
short ring lock to enqueue the record. Full or oversized records drop whole
and increment diagnostics. It never waits for the host. Other RTOS profiles
and UART console builds retain the previous formatter. `LOGGING` reports
record/drop/oversize counts and maximum masked copy time in microseconds
(not formatting time, lock wait, or total task runtime).

Step5 v0.1.1 corrects the v0.1 stationary acquisition failure. The RTOS-only
transport rechecks completion before timing out, retains bounded2ms waits
for each hardware phase, records abort sources and stops queuing commands
after failure. Interrupts/dispatch remain enabled. The reference SDK driver
transport is retained behind the opposite compile-time branch.

`ri_task` owns the GY-511/LSM303 on I2C1 GP2/GP3, core0 priority2. It runs
the existing `b4v23_tick` service every20ms, using its400Hz FIFO, axis mapping,
2second stationary calibration, powered acceptance and detector6 parameters.
The `imu_proc.c` numeric/state decisions, `b4_config.h`, FIFO processing/
configuration and binary packers are unchanged. RTOS-only passive hooks
record acquisition outcomes and exact calibration restart reasons.
The shared register shim selects the verified migration TIMER instead of
the SDK timer API; SDK builds retain their original branch. RTOS diagnostics
replace the SDK service's optional console printing.

The100Hz motion task remains on core1 and owns encoder drains/ADC/PWM.
It copies odometry, interval/invalid-transition quality, speeds and session
state into a spinlock-protected mailbox. Buddy4 never drains encoders.
The IMU task computes observation age and Start/Resume/Stop/Reset edges;
it copies its latest tilt/quality view back under the same short lock.
No I2C, packet enqueue or formatting occurs while the mailbox lock is held.
An accepted stopped Reset/USB x requests physical IMU reinitialisation and
a fresh2second calibration, including when the sensor was healthy/idle.
`IMU_ACQ epoch` increments when reinitialisation starts; `ready_epoch`
catches up only after calibrated Idle is observed. Acceptance alone is not
calibration completion. No front IR calibration changes. Acquisition counters
are lifetime32bit counters and remain across Reset; the legacy six-bit
telemetry error count can wrap or restart after IMU reinitialisation.

The task sleeps to the next fixed20ms release (rebases after a full missed
period), ahead of core0 USB/Wi-Fi; core1 motor/safety/navigation stay separate.
`IMU_ACQ` separates failed FIFO reads, gaps>30ms and sensor FIFO overruns,
with max gap/wake lateness. `IMU_I2C` reports TX/STOP/RX timeout counts,
abort count/source/address, rescued deadline checks and maximum transfer
wall time. Snapshot copies are protected; formatting occurs outside locks.
`IMU_TIMING` adds last acquisition gap, last late gap and its lifetime read
number. Ten-second buckets expose current and previous read/late counts and
maximum gap, so startup maxima cannot obscure current intervals. `win_seq`
increments on bucket rotation; buckets continue across Reset, and counters
are copied into the same mailbox. The bucket counts use9-bit fields (at most
501 releases per10seconds at50Hz); lifetime counters retain32-bit wrap.

`IMU_CAL` retains per-initialisation epoch evidence after a fault: progress
and best of100 stable period means, total calibration age, axis/speed restart
counts, last rejected axis spans, failed/empty reads, late gaps and overruns.
`last_reason` bits1/2/4 are forward/lateral/up range>40mg; bit8 is the existing
estimated-speed stationary guard. Multiple bits can occur on one restart.
These observations do not relax the2second/10second/40mg requirements.
Each physical reinitialisation resets this diagnostic epoch only; fault
evidence remains visible until an explicit Reset/reinitialisation.

`NET_SERVICE` is a copied snapshot measured by the kernel Wi-Fi owner on
core0, covering radio/link work, lwIP timeout servicing and MQTT bridge work.
Durations include preemption and any waits: they are elapsed wall time, not
exclusive CPU usage or proof that one phase caused a particular late sample.
Application reporting never calls network stack APIs to obtain these values.

The motion owner calls the pure `ri_ramp_request` adapter with that copied
tilt. It uses the reference RampAssist detector6: four valid20ms samples at
|height|>=4mm; bounded20% feedforward addition (40PWM cap), stall push,
35mm/s minimum-speed hold, integral300 and3x Ki. The original budgets,
invalid/stale handling and release ramps remain. These settings are the
baseline's one-sided height model; they do not introduce a general two-wheel
longitudinal ramp detector.

All requests go through `nav_request_ramp_assist(MOTION_BUDDY4,...)`, TTL50ms,
and `nav_get_ramp_assist` readback. This is a modifier: Buddy3 retains line
ownership. `motion_apply_ramp_profile` in the motion controller applies only
the approved request. No IMU code writes motor GPIO/PWM. Ordinary corrections,
barcode scanning, the420PWM cap and1200/s slew remain. Search/junction/Pause/
Stop remove eligibility; when leaving continuous drive, added ramp effort is
removed from the legacy controller handover seed. Controller/session reset
paths still dominate all outputs.

Reference IMU faults/shocks are diagnostic warnings rather than unconditional
global drive stops. Invalid/stale tilt expires/removes assistance using the
existing timers; it does not authorize driving without USB/network supervision.
No automatic restart after a stop or fault is added.

Sources2/4 reuse the original binary packers. Source15 ramp and C1 approved
target packets keep their layouts and share `rn_next_sequence(15)` with peak
diagnostics. All producers use copied nonblocking TX buffers; only the kernel
Wi-Fi task calls lwIP/MQTT. USB IMU/IMU_TILT/RAMP snapshots are copied before
formatting. The existing dashboard displays these sources and identifies
Step5 using an extra bit within the connected motion profile marker.

Software checks include the actual IMU service with fake FIFO and RTOS TIMER
port, stationary calibration,10mm side hump acceptance, reset/fault/odometry
quality, powered acceptance, detector6 ramp/plant tests, coordinator TTL and
profile tests. These do not measure real I2C/task cadence; compare CLOCK,
IMU missed/cost and heartbeat/ADC timing during the physical test.
