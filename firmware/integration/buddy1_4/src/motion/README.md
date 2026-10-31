# Motion control — Buddy 2, with coordinator-approved ramp help

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Ordinary control

Motion consumes coordinator-approved wheel-speed targets and encoder feedback. Signed encoder movement becomes wheel distance; measured velocity is filtered before use. The Continuous V2 controller runs roughly every10ms. It keeps per-wheel reference, integral, actual previous output, and a one-use startup-aid latch.

For a forward segment, the reference changes at up to+150/−250mm/s². Provisional feedforward is150 PWM left and180 right at45mm/s, scaled linearly through zero. Speed error and an integral term correct the effort. Normal integral magnitude is limited to100. Applied physical output is clamped0–420 and slewed at1200 PWM units/s. Anti-windup tracks actual output when the actuator is constrained, rather than integrating effort the motor never received.

A stopped wheel may receive one bounded starting aid per positive segment after100ms stillness: up to300PWM for at most150ms, ending early when rolling. Ordinary target changes do not repeatedly re-arm it. Explicit Stop uses the separate immediate hardware-off path. Zero steering demand on a ramp is a distinct case described below.

Special turns, line searches and external distance/angle manoeuvres select their established controllers. `motion_executor.c` tracks finite-command progress, deadline, direction and settling. A command being accepted does not mean it completed; callers query its ID/result.

## Ramp profile

`ramp_assist.h` is the retained pure decision state machine. The IMU supplies timestamped valid/invalid lateral-height observations. After four consecutive valid20ms samples with |height|≥4mm, ordinary AUTO following may request:

- Feedforward fraction+20%, capped40PWM per wheel, ramped in/out;2.5s assisted budget.
- If both wheels remain under5mm/s for200ms while assisting, bounded+60PWM stall push. It releases when moving again and has a1.5s per-episode push budget. Push intervals do not count against the main assisted-time budget.
- A ramped minimum target35mm/s per wheel, integral limit300 and integral-gain multiplier3. This speed hold can persist after the feedforward boost ends, until200ms flat,5s budget, stale tilt>200ms or prolonged invalid tilt≥500ms. Short exclusion handling is retained.

The coordinator receives a canonical bounded request every control period with a50ms expiry. It checks requester Buddy4, AUTO ownership/context and bounds. It clears on Stop, takeover, ineligible following or expiry. Buddy3 remains owner during assistance. `ramp_drive.h` applies only the approved request to private controller fields. The IMU module never writes those fields or PWM.

The hold can turn a zero **steering** target into forward travel toward35mm/s. Explicit Stop/Pause/ESTOP and leaving following remove the request through different outer gates. PWM cap and output slew remain unchanged. Payload quantization is1milli for fraction/hold and0.01PWM for push; the original numeric control constants remain.

## Diagnostics and evidence

The detailed stored drive log includes requested target, slewed reference, speed, integral and actual hardware PWM. Source15 subtype0xC1 adds requested vs hold-constrained targets, approved boost/push/hold and owner. The constrained target is not the instantaneous slewed reference and not measured speed.

The teammate's three10mm right-wheel ramp crossings on the old combined build succeeded with accepted12/11/13mm estimates. The preceding fixed300PWM floor failures are retained to explain why speed hold replaced them. Host models verify bounds, release and zero-assist numerical parity; they do not establish that this restructured image has already crossed a physical ramp.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
