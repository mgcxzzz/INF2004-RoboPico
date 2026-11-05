# Provisional Buddy 1–4 RTOS v0.2.0 status

Main is intentionally promoted before the remaining full-circuit checks, per
user instruction. Buddy 5 remains a blank integration point, not a working feature.

| Feature | Current evidence |
|---|---|
| Micro T-Kernel clocks/tasks | Step 1 and subsequent captures passed verified clocks and about 100 Hz control |
| Buddy 1 network/dashboard | Step 3 baseline/reconnect and connected controls passed; release adds network-supervised powerbank use |
| Buddy 2 motion/encoder API | Motor/Stop/ESTOP and flat Pause/Resume passed; normal control algorithm retained |
| Buddy 3 line/barcode/junction | Straight, U-turn, decode/mapped dispatch passed; full L/R rejoin and sharp bends pending |
| Buddy 4 IMU/ramp | Step 5v0.1.5 flat0 humps; same 10 mm one-sided ramp1 accepted/13 mm maximum; assistance active; clean measured motion timing |
| Main powerbank policy and wireless stored exports | Focused production software/browser/build checks; first physical powerbank check pending |
| Buddy 5 avoidance | NOT integrated |

The successful ramp was one run. User declined another repeat before main
promotion. The13 mm estimate differs+3 mm from the nominal 10 mm object; this
is not general accuracy/repeatability evidence. No IR recalibration or gain
changes were made for this release. Stored calibration priority remains own
CRC-valid RTOS save, SDK record28, then fallback. Expected current car values
are white122/138, black246/267; verify actual readback.

Powerbank policy: no USB-DTR requirement, but fresh kernel MQTT/link/app service
is required and recent TCP publish-ack activity must be within 3 seconds. Link/
service loss or stale delivery stops motion; reconnect never restarts it.
This acknowledges TCP delivery activity, not application-level execution.
Heartbeat/release metadata continue when ordinary telemetry is off.

Before tuning: short untethered Start/Pause/Resume/Stop and network-loss check,
then the circuit with enough mat for sensors/wheels throughout bends and turns.
Preserve failed runs and export before another Start. Further tuning versions
can branch from this main baseline without changing the archived checkpoints.
