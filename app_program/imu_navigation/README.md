# Buddy4 IMU and ramp assistance on micro T-Kernel

Experimental Step5, based on Step4 v0.1.2. Physical acceptance of both the
new gate and this IMU port is pending. Buddy5 and powerbank driving remain
deferred. SDK main and the published Step4 artifacts are unchanged.

`ri_task` owns the GY-511/LSM303 on I2C1 GP2/GP3, core0 priority18. It runs
the existing `b4v23_tick` service every20ms, using its400Hz FIFO, axis mapping,
2second stationary calibration, powered acceptance and detector6 parameters.
`imu_proc.c`, `b4_config.h`, FIFO driver and binary packers are unchanged.
The shared register shim selects the verified migration TIMER instead of
the SDK timer API; SDK builds retain their original branch. RTOS diagnostics
replace the SDK service's optional console printing.

The100Hz motion task remains on core1 and owns encoder drains/ADC/PWM.
It copies odometry, interval/invalid-transition quality, speeds and session
state into a spinlock-protected mailbox. Buddy4 never drains encoders.
The IMU task computes observation age and Start/Resume/Stop/Reset edges;
it copies its latest tilt/quality view back under the same short lock.
No I2C, packet enqueue or formatting occurs while the mailbox lock is held.
An accepted stopped Reset/USB x resets the IMU, not the front IR calibration.

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
