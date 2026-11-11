# Buddy 1–4 RTOS v0.2.3: IMU end-of-run stop

This provisional main release implements the professor's revised requirement:
IMU collision detection is replaced by lifted/overturned detection. Once qualified,
all motor motion stops and stays stopped until an explicit eligible Reset and Start.
Buddy 5 is not integrated. This is a new hardware acceptance build.

## Retained baseline

The v0.2.2 7–30 mm junction qualification is retained. The user reports that the
new minimum stopped missing junction turns. Bend assistance is still untested on
the robot. The user reports recovery does not visibly sweep; without that run's
logs, recovery constants/logic are unchanged here. Profile 1, saved front IR and
Wi-Fi settings, both barcode decoders, ramp height/assist, nav APIs, motor ownership
and network supervision remain. No recalibration is required for this update.

## Detection and stopping

The existing IMU task reads the same 400 Hz acceleration FIFO every 20 ms.
The guard examines car-frame acceleration before ramp-height quality rejection.
It uses the existing flat calibration gravity vector, independently of wheel
odometry and whether a hump-height sample is accepted.

- Pick-up cue: projection along calibrated upward gravity reaches 1.4 g for
  30 ms of consecutive fresh samples.
- Overturn/steep-tilt cue: more than 60 degrees from calibrated upright for
  50 ms, with acceleration magnitude between 0.75 g and 1.25 g.
- A single bump does not accumulate either duration. Failed, empty, stale or
  overrun batches reset qualification dwell. They cannot complete a gesture.
- The first qualified event is latched in a protected snapshot. The 1 ms safety
  task cancels any pending Start and routes a Buddy 4 Stop through the existing
  motion API/coordinator. Motor commit gates also reject blocked IMU safety.
  PWM is set to zero, without a normal deceleration ramp. Physical wheel stopping
  time is not measured by these packets and is not guaranteed to be zero.
- Loss of fresh guard data for over 100 ms blocks Start and stops an active run
  or pending countdown. This is reported separately from lift/overturn.

The filter durations are evidence qualification times, not delays added after
recognition. End-to-end hardware stop latency still needs measurement. Thresholds
are conservative first-trial settings, not certified discrimination against every
ramp/bump. Existing historical collision enum values stay reserved for packet
compatibility; production no longer enters the old Impact classifier.

An accelerometer measures gravity and acceleration, not contact with the floor.
A very slow, perfectly level lift may be indistinguishable from resting. For the
demonstration use one deliberate lift with a clear tilt of a little over 60 degrees,
held for about 0.2 seconds. Do not shake the car or use a special sequence.
Normal ramps near or above the tilt limit are inherently ambiguous; the tested
10 mm one-sided ramp is far below that angle. See ST's
[tilt measurement note](https://www.st.com/resource/en/application_note/an4509-tilt-measurement-using-a-lowg-3axis-accelerometer-stmicroelectronics.pdf)
for the gravity/acceleration sensing model.

## Flash and start

1. Extract the complete new bundle. Stop the robot before flashing
   `firmware/robopico_buddy1_4_rtos_v0_2_3.uf2` in BOOTSEL mode.
2. Start the new **Start dashboard.cmd**, or on Mac run
   `python3 tools/dashboard/run_dashboard.py` from the bundle root. Hard-refresh
   the browser. Existing Wi-Fi setup can be reused if the laptop IP is unchanged.
3. Keep the car flat and still for IMU calibration. Check v0.2.3, car-online,
   calibrated IMU and fresh **IMU end-of-run safety** data. Apply/read back mappings.
4. Powerbank operation remains available. Use the working motor supply too.
   No USB serial/capture helper is required. Dashboard Start retains its countdown.

## Focused robot checks

Use enough mat, and hold the chassis clear of the wheels when lifting.

1. Run a short normal straight. Check no new IMU safety trip. Then lift and tilt
   as above. Verify both motors stop and dashboard records a lift or overturn stop.
2. While still tilted, try Start/Resume once: neither may energize the motors.
   Putting the car back down must not restart it.
3. Put it flat and still for at least 0.5 seconds. Click Reset; wait for IMU
   recalibration/readiness. A new explicit Start is required. Reset cannot prove
   ground contact: follow the put-down instruction even if holding it level.
   An unavailable IMU with no qualified lift latch may accept Reset to retry
   initialization; motor outputs remain blocked until fresh calibration succeeds.
4. Check the same normal ramp once for a false positive from the new guard.
   This has a purpose: it checks this safety addition against the existing ramp.
   No barcode threshold sweep, network reconnect marathon or controller retuning
   is required for this build. If the bend/recovery issue appears, preserve that run.

Telemetry-off must not disable safety or its C9 snapshot. A real detected event
must remain observable and latched even while ordinary telemetry is off.

## Send useful evidence

After Stop, preserve RAM histories before Reset, another Start or power loss.
Send dashboard **all frames CSV**, **event CSV**, Messages **Save log**, plus
stored **Diagnostics**, **Navigation**, and **Raw sensors** CSV/JSON/BIN if the
run also involved line/barcode behaviour. A short video covering the lift and
wheel stop is useful for this new feature. For a failed gesture, record how it
was lifted and whether the safety panel showed stale/not-ready, qualification,
or a latched event. Include broker.log only if a network loss also occurred.

The always-on C9 snapshot contains event time/count, latest sample time, calibrated
upward acceleration projection, magnitude, qualification dwell, reset eligibility
and the current run ID. Run ID is the run current when the packet is copied;
all-frames/events provide the history. No direct MQTT/lwIP calls are added to the
IMU subsystem, and no wire text is used for these application values.
