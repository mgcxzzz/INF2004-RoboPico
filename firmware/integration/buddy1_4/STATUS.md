# Provisional Buddy 1–4 RTOS v0.2.5 status

Main is intentionally promoted before the remaining full-circuit checks, per
user instruction. Buddy 5 remains a blank integration point, not a working feature.

| Feature | Current evidence |
|---|---|
| Micro T-Kernel clocks/tasks | Step 1 and subsequent captures passed verified clocks and about 100 Hz control |
| Buddy 1 network/dashboard | Step 3 baseline/reconnect and connected controls passed; release adds network-supervised powerbank use |
| Buddy 2 motion/encoder API | Motor/Stop/ESTOP and flat Pause/Resume passed; normal control algorithm retained |
| Buddy 3 line/barcode/junction | Straight, U-turn, decode/mapped dispatch passed; full L/R rejoin and sharp bends pending |
| Buddy 4 IMU/ramp | Step 5v0.1.5 flat0 humps; same 10 mm one-sided ramp1 accepted/13 mm maximum; assistance active; clean measured motion timing |
| Main powerbank policy and wireless stored exports | v0.2.0 user powerbank runs/downloads confirmed; v0.2.1 diagnostic runs received; v0.2.2 drive changes await physical acceptance |
| Buddy 5 avoidance | NOT integrated |

The successful ramp was one run. User declined another repeat before main
promotion. The13 mm estimate differs+3 mm from the nominal 10 mm object; this
is not general accuracy/repeatability evidence. No IR recalibration or gain
changes were made for the original Step 5 promotion; v0.2.2 adds selective
bend assistance as described below. Stored calibration priority remains own
CRC-valid RTOS save, SDK record28, then fallback. Expected current car values
are white122/138, black246/267; verify actual readback.

Powerbank policy: no USB-DTR requirement, but fresh kernel MQTT/link/app service
is required and recent TCP publish-ack activity must be within 3 seconds. Link/
service loss or stale delivery stops motion; reconnect never restarts it.
This acknowledges TCP delivery activity, not application-level execution.
Heartbeat/release metadata continue when ordinary telemetry is off.

Next: test v0.2.2 on a straight, troublesome bend, supported in-place recovery
and the previously missed junction, with enough mat for sensors/wheels.
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
causes untuned. Subsequent user runs supplied these diagnostics and identified
the borderline junction width below; full-circuit acceptance and Buddy5 remain separate.

## Drive refinements v0.2.2

User run on November 6 decoded F/Z/Y/Z/F; four manoeuvres completed and the
last F remained pending. Diagnostic reconstruction estimated a 10.4 mm black
stripe against the 11 mm minimum; black ADC exceeded its learned threshold.
This motivates a 7 mm minimum while retaining fresh-white and width-maximum
checks. Exact candidate/rejection width is now exposed through C7.

Bend assist and stationary-search constants are conservative first-trial choices,
not hardware-tuned values. Core timing, IMU/ramp, decoder acceptance and stop
cutoffs remain unchanged. C8 shows bend/recovery state. Straight parity and
focused native/browser/build checks are software evidence; supported-mat sharp
bends, reverse search effort/reacquisition and junction acceptance still require
the next physical run. See ../../../docs/drive_refinements_v0_2_2.md.

## IMU end-of-run safety v0.2.3

Professor revision removes IMU collision detection and requires lifted/overturned
stopping. Release-only raw acceleration qualification now sets a sticky safety
condition, handled by Buddy 4 through the existing coordinator Stop path. New
C9 diagnostics remain available with ordinary telemetry off. Hardware acceptance
of the lift/overturn gesture, output stop and ramp false-positive checks is pending.

User update: v0.2.2 junction width change never missed a junction in the reported
test. Bends were not tested. Recovery reportedly did not visibly sweep; no logs
are currently available, so this release leaves those algorithms unchanged.

## IMU reporting clock correction v0.2.4

User screenshot after flat/still Reset showed armed,500ms upright, no candidate
dwell and a false not-ready label. The actual service test reproduced mixed-clock
freshness failure with137ms kernel/hardware offset. C9 now owns its hardware-clock
read. It retains both-direction offset regression coverage. This correction does
not certify the actual cause of every failed Start; full frames/acknowledgment
are needed if motion remains blocked after the display becomes consistent.

## Demo console / drive modes v0.2.5

The approved Demo overview is connected to live telemetry. Stopped-only45/60/75
cruise selection uses the shared coordinator; default45, calibration, barcode
decoders and junction/recovery policies are preserved. Added bounded per-wheel
load reserve does not replace stop/stall guards. IMU tilt45degrees/100ms and
protected-clock ordering replace the previous60degrees/50ms check; upward
400mg/30ms remains. Retained peak diagnostics are since explicit Reset.

**Hardware acceptance pending:** revised pickup stop, ramp false-positive check,
faster cruise behavior and load reserve. The previous successful ramp was on
Step5v0.1.5; do not present it as validation of these new changes. See the
[v0.2.5 checks](../../../docs/demo_console_v0_2_5.md).
