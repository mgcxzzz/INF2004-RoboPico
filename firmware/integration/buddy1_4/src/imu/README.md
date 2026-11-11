## Active release v0.2.3: lift/overturn safety

The production IMU FIFO additionally feeds an independent integer lift guard,
using calibrated gravity projection and sustained steep orientation. It can stop
while tracking, turning, paused or in the Start countdown, independently of hump
quality/odometry. The acquisition task copies state; core1 safety cancels through
the Buddy 4 motion API/coordinator. It never writes PWM from acquisition or MQTT.
A qualified event is sticky; fresh upright evidence plus explicit Reset is required.
Old Impact enum values remain compatible, but production collision classification
is disabled; transient high dynamics still invalidate terrain measurements.
See docs/imu_end_run_v0_2_3.md for thresholds, limits, setup and purposeful tests.
The sections below describe the retained terrain/height baseline.

# IMU and ramp estimation — Buddy 4

Main v0.2.0 uses the micro T-Kernel production profile. Active task/ownership
adapters: [app_program/imu_navigation](../../../../../app_program/imu_navigation/README.md).
See [RTOS architecture](../../../../../docs/architecture.md) and
[current setup](../../../../../docs/setup.md). The shared logic below remains;
references to the standalone SDK harness describe the historical adapter.

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Acquisition and coordinate system

The GY-511/LSM303 accelerometer uses I2C1 GP2/GP3 at400kHz with bounded register-transfer timeouts. Sampling uses400Hz FIFO data; `imu_service.c` processes a period every20ms. Raw axes are mapped to car forward/up/lateral coordinates through `b4_config.h`.

The application supplies a read-only copy of already-drained encoder observations: cumulative wheel travel, speed, heading, invalid-transition counters and age. Buddy4 does not drain/reset the encoders itself. Dirty, stale or non-finite odometry is excluded.

## Processing and state

`imu_proc.c` implements initialization, still calibration, ready/tracking, hump and fault processing. Reset authorizes sensor reinitialization and a fresh stationary calibration. Start/Stop/Resume edges follow the coordinator session rather than an independent motor state machine.

Each period checks FIFO/timing quality, gravity consistency, spread, acceleration, turns, reversing, impacts and movement quality. Rejected samples carry explicit reason bits. The published tilt view includes sequence number, timestamp, validity, tracking and exclusion reasons so consumers can detect stale data and distinguish a short acceleration exclusion from sensor loss.

The current ramp estimate is lateral right-minus-left height from side tilt. It describes one wheel being higher than the other; it is not a general longitudinal pitch-to-height measurement for both wheels climbing together. Hump acceptance uses the estimator's motion/quality and episode budgets. The powered-acceptance changes retained from the teammate address quantized heading and brief acceleration-only exclusions.

## Boundary with navigation and motion

The estimator emits measurements and telemetry only. The application ramp decision adapter turns those observations into an expiring `nav_request_ramp_assist` request. The coordinator approves eligibility and the motion module applies its fixed control profile. No IMU file writes motor GPIO/PWM or private wheel-controller fields.

The current SDK integration retains IMU diagnostic faults in source4. They are not automatically mapped into an unconditional global driving stop. This is an explicit limitation/policy boundary: adding a safety policy later requires agreeing warning versus stop behavior. An invalid/stale tilt still removes ramp assistance according to the existing bounded timers.

Core0 polls the IMU just after a barcode sample where possible, with alignment preference700µs and scheduling slack2ms. I2C work remains synchronous and bounded; moving it into an RTOS task is not part of this SDK refactor. ADC/control gaps must be measured on new hardware acceptance runs.

## Evidence

The archive includes the original Buddy4 standalone host log, its branch test ledger, and the combined ramp successes/failures. The standalone log predates this integration. Current tests cover service startup/reset, copied odometry validity, packet decoding, powered acceptance and the ramp-assistance state machine. New hardware calibration should occur with the robot still; clearing the IMU calibration does not recalibrate the front IR sensors.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
