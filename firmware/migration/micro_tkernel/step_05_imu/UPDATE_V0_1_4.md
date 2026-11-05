# Step5 v0.1.4 — bounded prioritized IMU acquisition during driving

Experimental micro T-Kernel SMP migration image, Buddy1–4. Main is unchanged;
Buddy5 remains a blank integration point. This continues the same RTOS,
shared API/nav-coordinator and compact binary telemetry design.

v0.1.3 passed stationary cold boot/Reset and functional tracking/Pause/Resume/
Stop. Under motion, however, its lower-priority core1 IMU task fell to about
15–20 reads/s while the higher-priority motor/barcode work stayed near100Hz.
The ramp window added151 late gaps and16 FIFO overruns. Normal motor power
crossed the ramp, with no accepted Buddy4 height or assist activation.

## Scheduling correction and bounds

The sole IMU task now runs on core0 priority2, ahead of USB3 and kernel Wi-Fi4.
Motor14, navigation10 and safety6 remain on core1 with their original ownership.
The earlier core0 IMU attempt used priority13 below USB/Wi-Fi; this version
explicitly reserves higher priority for short sensor work instead of relying
on CPU time left over by either core's background work.

The FIFO read has an8ms aggregate elapsed deadline, checked across transfer
progress as well as wait paths. Init has a separate20ms aggregate deadline
and a2ms controller reset-ready wait. Expiry prevents further commands,
records the reason and follows existing failed-read/init warning handling.
It does not grant movement or change steering/ramp quality thresholds.

The8ms FIFO deadline is **not a hard cap on the whole IMU task tick**. Conversion,
bounded32-sample processing and copied telemetry follow the read. Full ticks
over8ms are counted diagnostically. Hardware validation must check both IMU
timing and the USB/network work now below it in priority. No long interrupt
masking or host/network wait is added to the IMU path.

## Focused diagnostics

- IMU_PHASE separates FIFO read and processor wall time, whole-tick wall time,
  maxima and FIFO batch size. These include preemption/waits, not exclusive CPU.
- IMU_BUDGET separates FIFO/init deadline exhaustion, excess duration beyond
  each limit, controller reset timeout and diagnostic full ticks over8ms.
- SIDE_SERVICE counts all due side ADC attempts and decoder/planner work,
  including polls between10ms control updates. Totals/maxima are wall time;
  their own copy-lock bookkeeping is excluded. Lifetime uint32 totals wrap.
- Existing calibration epochs/reasons, recent10second timing buckets,
  NET_SERVICE, LOGGING and motor/control diagnostics remain.
- Capture summaries preserve these new diagnostic rows. Existing MQTT
  requests/telemetry payload layouts and sizes are unchanged.

## Representation and concurrency

FIFO batch0–32 uses6bits; last failure phase0–6 uses3bits; device address
uses7bits. Existing9-bit window counters,7-bit calibration progress and
packed reason/state fields are retained. Lifetime event counters/timing
deltas use their full uint32 modulo domains. Constant budget/priority values
are configuration rather than additional wire fields. See the extended
docs/data_representation.md table and linker measurements in BUILD_MANIFEST.

Only the IMU owner writes acquisition diagnostics, then publishes a copied
mailbox snapshot under a short lock. The motor owner similarly records side
poll counters after I/O. Formatting and hardware I/O remain outside those
locks. New profiling adds RAM, not binary MQTT message bytes; this does not
claim completion of the outstanding whole-firmware Tribunal evaluation.

## Validation and next test

Checks are targeted at the changed transport budgets, real IMU service/task/
calibration, actual ramp coordinator route and capture/export compatibility,
plus the ARM build and package flash/RAM/stack/source checks. Unchanged
decoder/steering suites do not need another broad rerun for passive timing
instrumentation. Hardware timing remains unverified until the new capture.

The six selected C programs and24 selected Python capture/export tests passed.
The ARM image built successfully; package checks verified required RTOS/motor/
network/IMU symbols, RAM flash helpers and application stack frames below2048
bytes (maximum944). The default SDK preprocessing tokens of imu_service.c,
imu_proc.c and lsm303_driver.c match the main reference. This checks source
isolation, not physical timing equivalence.

| Same build profile | v0.1.3 | v0.1.4 | Change |
| --- | ---: | ---: | ---: |
| ELF static BSS bytes | 189424 | 189576 | +152 |
| Linker static RAM end | 0x2002e630 | 0x2002e6c8 | +152 bytes |
| RAM before dynamic kernel allocations | 72144 | 71992 | -152 bytes |
| Existing binary packet sizes | baseline | unchanged | 0 bytes |

These are linker/static measurements, not runtime task-stack high-water marks.

Use START_HERE.md. Begin with a short stationary check, then one short flat
Start/Pause/Resume/Stop run with the connected capture open. Confirm actual
IMU core0/priority2, approximately50Hz through motion, no new late gaps/FIFO
overruns/budget failures, main100Hz and responsive USB/heartbeat. Preserve
the original calibration. Send the full rtos_connected_session folder before
repeating the ramp; no video or stored exports are required for this latency
test unless there is an additional physical tracking problem.
