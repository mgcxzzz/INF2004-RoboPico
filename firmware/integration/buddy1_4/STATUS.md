# Provisional Buddy 1–4 RTOS v0.2.1 status

Main is intentionally promoted before the remaining full-circuit checks, per
user instruction. Buddy 5 remains a blank integration point, not a working feature.

| Feature | Current evidence |
|---|---|
| Micro T-Kernel clocks/tasks | Step 1 and subsequent captures passed verified clocks and about 100 Hz control |
| Buddy 1 network/dashboard | Step 3 baseline/reconnect and connected controls passed; release adds network-supervised powerbank use |
| Buddy 2 motion/encoder API | Motor/Stop/ESTOP and flat Pause/Resume passed; normal control algorithm retained |
| Buddy 3 line/barcode/junction | Straight, U-turn, decode/mapped dispatch passed; full L/R rejoin and sharp bends pending |
| Buddy 4 IMU/ramp | Step 5v0.1.5 flat0 humps; same 10 mm one-sided ramp1 accepted/13 mm maximum; assistance active; clean measured motion timing |
| Main powerbank policy and wireless stored exports | v0.2.0 user powerbank runs/downloads confirmed; v0.2.1 added diagnostics need one physical check |
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

Before tuning: collect the v0.2.1 same-route diagnostic run,
then the circuit with enough mat for sensors/wheels throughout bends and turns.
Preserve failed runs and export before another Start. Further tuning versions
can branch from this main baseline without changing the archived checkpoints.

## Diagnostic update v0.2.1

User wireless runs of v0.2.0 confirmed actual powerbank driving. Run2 ended
with encoder reason9/bit0x40 during left-reverse U-turn. Later run3 decoded
Y/Z/F and completed all actions; later third groups often had14 bars, one had17.
Run10 ended by GP21 Stop after an F group was rejected. Raw export covered its
first Y only. Run19 ended with network reason12 after the turn returned to
tracking; the broker showed a real delivery gap and later keepalive timeout.

v0.2.1 adds latest reportable raw capture, first encoder-trip and first network
stop snapshots, compact wire codecs and dashboard exports. It leaves the
causes untuned. Physical validation of these new diagnostics is pending one
same-route run; full-circuit/sharp-bend tuning and Buddy5 remain separate.
