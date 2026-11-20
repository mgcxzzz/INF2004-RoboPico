# Buddy 5 RTOS integration (APP=rtos_buddy1_5)

Experimental candidate on the v0.2.12 release. **Not physically validated.**
`APP=rtos_buddy1_4` is byte-identical to a clean v0.2.12 build (same
compiler). It keeps the v0.2 integration (width failures hold, line-steered
300→200 mm approach under the Buddy 5 lease) and replaces the short width
probe with the SDK `width_long_probe` v0.2.

The Buddy1–4 release keeps its tasks, coordinator, line follower, barcode
route, ramp assistance, saved Wi-Fi and IR calibration. This folder adds the
Buddy 5 adapter and mission; the obstacle algorithms are ported SDK modules in
`firmware/integration/buddy1_4/extensions/buddy5_obstacles/` (provenance in
[docs/buddy5_rtos_integration.md](../../docs/buddy5_rtos_integration.md)).

## Sequence (one Buddy 5 lease from the takeover to the hand-back)

1. **Guard** (line following, Buddy 3 owns AUTO): servo centred, one ping per
   60 ms. **Straight-only takeover** (`b5_straight.c`): the takeover is
   requested only while Buddy 3 has been following a straight for the last
   200 mm and 300 ms with fresh line tracking and no junction, turn, loss or
   recovery. The v0.2.15 candidate limits encoder heading change to 6° across
   the full window and 3° across roughly 40 mm of recent travel. The optical
   bend-assistance labels remain visible in the line telemetry.
   Takeover also requires three consecutive clean echoes
   ≤ 330 mm from pings requested after the gate opened. The v0.3.0 candidate
   counts those echoes early, then requests the stop on a qualifying echo
   ≤ 295 mm. Losing the gate discards the count. Three echoes ≤ 300 mm while the
   gate is closed stop the robot and hold `NOT_STRAIGHT`. Stop ends the hold. The
   gate decides where the manoeuvre may start; sonar still treats any close
   reflector, including a person's leg, as an obstacle. After a hand-back
   the front must read clear (> 330 mm or no return) for 1 s before the
   guard re-arms, and the gate needs fresh
   straight evidence again.
2. **Width policy, then width probe** (SDK width_long_probe v0.2
   with RTOS retry hooks). Once stopped, a fresh stationary triplet (three clean echoes
   within 15 mm, two attempts) decides the policy (E5 byte 19):
   - inside the probe's own REF window 270–330 mm → the probe is attempted.
     Its REF rule, clearance-limited diagonal per side (20 mm steps, 220 mm
     ceiling, 12 steps, ±46° servo, first-FAR stop, retrace, stationary retry,
     return corroboration), D1–D5 telemetry and the rule-2 on-Pico estimate
     still apply. The RTOS candidate can use the existing single stationary
     retry if two in-window reference echoes agree within 15 mm and the third
     clean echo is within 15 mm of the reference window. The replacement must
     pass the original REF limits. v0.3.1 also permits one stationary resample
     at an outward pose with a corroborating in-window echo, at most one
     missed rise, and at most one clean echo up to 5 mm below the normal
     near limit. The fresh triplet goes through the original classifier
     and the first triplet is recorded in D4. A complete run with an unresolved edge may show a labelled
     width assumption. **Any
     non-COMPLETE end holds (`WIDTH_ABORT`, module reason = end code),
     `REFERENCE` included**: END_REFERENCE alone is not evidence of a clean
     skip. The probe's D3 (150 mm `PROJECT_RANGE_ASSUMPTION`) stays the
     displayed number — policy `failed`.
   - clean but outside 270–330 mm → deliberate **skip**: the probe is not
     started, nothing moves for it, the readout is the labelled 150 mm
     assumption (basis 3, status INVALID), and Buddy 5 holds because there is no
     scan evidence for a side. E5 bytes 57–58 give the deciding range.
   - faulty or inconsistent triplet → HOLD `RANGE` (never a skip).
3. **Direction and alignment.** Choose the side with the shorter obstacle
   extent from the long scan. Two corroborated edge brackets can establish
   the offset or a centred default of RIGHT. If one edge is beyond reach,
   the near edge must be corroborated and the return pass must confirm that
   the obstacle extends farther on the other side. Ambiguous or contradictory
   evidence holds before the approach. After choosing, correct lateral drift
   above 6 mm with one clearance-checked forward diagonal of up to 30 mm,
   then restore the original heading. Require the original tape sensor to
   see the tape for 150 ms within 6° of the initial heading. The bounded
   search checks −4°, +4°, −6° and +6°. Failure holds.
4. **Approach** (sonar-controlled, line-steered): fresh stationary range
   (three clean echoes within 15 mm). If > 215 mm, a Buddy 5 finite drive at
   20 mm/s by at most `range − 200` (≤ 150 mm), pinging every 60 ms; stop on a
   clean echo ≤ 205 mm; re-measure; ≤ 3 legs. During each leg the mission
   runs Buddy 3's own edge law (`analog_demand()`, identity calibration on the
   already-normalised levels) each fresh observation and turns its wheel
   differential, scaled to 20 mm/s (same turn per distance), into a heading
   setpoint for the drive (`nav_steer_drive`, ±8°, 0.1° steps, lapses after
   100 ms without refresh). The executor keeps its own heading hold, goal,
   timeout, settle and 12° fault; ownership never returns to Buddy 3. More
   than 40 mm with no line evidence, or sonar faults → HOLD (unchanged); the
   follower's own both-black/white stop timers are not used.
5. **Bypass.** Use the existing rectangular route and a fresh stopped range
   within the 80–220 mm planning window. Modelled heading and lateral pose
   come from the scan plus the completed
   alignment and approach moves. Depth allowance remains 150 mm and the
   lateral leg stays 250 mm. Require confirmation on the same tape sensor
   before handing back to Buddy 3.
6. **Hand-back** only through `nav_resume_line(…, false)`; the pending route is
   kept and an interrupted junction is refused (existing rule).

The calibrated width is display-only. Navigation uses the scan's corroborated
edge observations and the fixed route margins. The display is the raw edge midpoint − 10 mm, limited to
100–200 mm; E5/D3 carry all three values.

## Ownership

| Context | Core / priority | Buddy 5 work |
|---|---|---|
| IO_IRQ_BANK0 (board_port.c) | core1 IRQ | GP17 ECHO edge timestamps (`rb5_echo_isr`) |
| Motion task (control.c) | core1 / 14 | Line observation and one mission step right after `motion_update()`; the private signed wheel controller committed through the existing `motion_commit()` |
| Buddy 5 task (`rb5_task`) | core1 / 20 | Servo slew, TRIG/ECHO transaction, copied telemetry, `o` report |
| Kernel Wi-Fi | core0 / 4 | Unchanged lwIP/MQTT owner; Buddy 5 only copies frames into the bounded TX queue (TMO_POL) |

All Buddy 5 contexts are on core1, so DI/EI protects the echo, servo and
telemetry queues. The Buddy 5 task cannot pre-empt the motion task; it drops a
queued record only after the bridge accepted its copy and only if the queue
head is still that record. Stop/E-stop/GP20/GP21/network loss go through
`bringup_stop_from()`, which also calls `rb5_cancel()`. A network Pause during
avoidance terminates the run through Stop. A mission HOLD is reported as Safe
Stop and requires explicit Stop to release it.

Servo requests only change the slew target (`servo_slew.h`): repeated
requests never restart its 20 ms cadence. The reported servo angle is the
applied PWM command, not a measured horn angle. Limit ±46°.

## Telemetry

Source 15, all dispatched by exact length and marker byte: E5 mission status
(version 4, 61 bytes: v2 plus the live width-probe state/side/phase/step and
the takeover gate reason and echo count;
every 500 ms and on stage change; the dashboard still decodes the 54/59-byte
v1/v2 layouts), width D1/D2 v10, D3 v3, D4 v3 and the D5 v2 per-side limits
frame (sent after D2),
BE/BF direction (v2 layout, version 3 = 25 mm near allowance), B5 bypass events
(unchanged v1). One record frame per 25 ms. The MQTT consumer assigns the
source-15 header sequence in dequeue order, so a retry cannot send an old
sequence after a newer frame. A gap measures loss after queue admission. See
`tools/telemetry/b5_rtos_decode.py` and the dashboard Buddy 5 panel.

## Tests

`testing/rtos_migration/test_b5_mission.c` runs this mission with the real
coordinator/executor and the ported modules against a kinematic plant, an
analog edge-reflectance line model and a geometric sonar model (logic
evidence only); `test_nav_steer.c` checks the coordinator steering extension. See the integration document for
all checks and their limits.
