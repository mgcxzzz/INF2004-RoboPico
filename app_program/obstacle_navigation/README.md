# RTOS obstacle navigation · Buddy 5

Buddy 5 is active in [RoboPico v0.4.3](../../firmware/releases/v0.4.3/) with `APP=rtos_buddy1_5`. It uses a servo-mounted ultrasonic sensor, acquires a coordinator lease to stop and probe, estimates obstacle width, selects a side, and performs a bounded bypass and line rejoin. Buddy 3 resumes ordinary line following after hand-back.

`b5_mission.c` stages the mission in the motion task. `b5_straight.c` checks takeover eligibility. `obstacle_service.c` and `board_obstacle.c` serve sonar, servo and copied telemetry. The [shared algorithm modules](../../firmware/integration/buddy1_4/extensions/buddy5_obstacles/) supply probing and bypass logic.

The [circuit evidence](../../testing/system/logs/2026-11-21_run_4/README.md) shows detection, reported width, bypass and rejoin on v0.4.2. The v0.4.3 stopped-range warning was checked with [release replay and dashboard tests](../../testing/integration/logs/2026-11-21_v0.4.3/README.md).

## Sequence (one Buddy 5 lease from the takeover to the hand-back)

1. **Guard** (line following, Buddy 3 owns AUTO): servo centred, one ping per
   60 ms. **Straight-only takeover** (`b5_straight.c`): the takeover is
   requested only while Buddy 3 has been following a straight for the last
   60 mm and 300 ms with fresh line tracking and no junction, turn, loss or
   recovery. v0.4.0 allows up to 6° of encoder heading change across
   the full window and 3° across roughly 30 mm of recent travel. The optical
   bend-assistance labels remain visible in the line telemetry.
   Takeover also requires three consecutive clean echoes
   ≤ 330 mm from pings requested after the gate opened. The mission
   counts those echoes early, then requests the stop on a qualifying echo
   ≤ 295 mm. Losing the gate discards the count. Three echoes ≤ 300 mm while the
   gate is closed normally stop the robot and hold `NOT_STRAIGHT`. During an
   active left, right or U-turn, including line reacquisition, v0.4.1 suspends
   Buddy 5 guard pings and clears prior echoes. Guarding resumes with fresh
   pings after the turn. A straight-through junction uses ordinary guarding.
   During straight settling at 286–300 mm, v0.3.12 can give the follower another
   200 ms at a 25 mm/s target limit to qualify the straight. If it still sees
   the obstacle when settling runs out or reaches 285 mm, it stops and checks
   three fresh echoes. It resumes line following if all three show clear space
   and tape evidence permits hand-back. As of v0.4.2, a stationary batch of
   three echoes within 270–330 mm and no more than 15 mm apart starts the
   normal width probe if the tape evidence from the stop is still present.
   v0.4.3 also accepts one echo down to 260 mm with a range warning when the
   other two are in the nominal window, the group still agrees within 15 mm,
   and the nearest echo passes the turn clearance check. For that mission,
   the probe can use the same rule when it acquires its own fresh reference.
   A mixed batch gets one stationary retry. Stop ends a HOLD. The
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
     still apply. The mission can use the existing single stationary
     retry if two in-window reference echoes agree within 15 mm and the third
     clean echo is within 15 mm of the reference window. The replacement must
     pass the original REF limits. At each remaining scan pose, v0.3.6
     allows one stationary recheck when at least one echo is in-window
     and another is short or misses its rise. The shortest echo reduces the
     modelled clearance by the full range deficit, and the existing
     front clearance margin must still fit. The fresh triplet goes through
     the original classifier. D4 records the first triplet. The final
     rejected D1 is retained in RAM and periodically republished until the
     next width scan. A complete run with an unresolved edge may show a labelled
     width assumption. **Any
     non-COMPLETE end holds (`WIDTH_ABORT`, module reason = end code),
     `REFERENCE` included**: END_REFERENCE alone is not evidence of a clean
     skip. The probe's D3 (150 mm `PROJECT_RANGE_ASSUMPTION`) stays the
     displayed number — policy `failed`.
   - A clean range up to 15 mm outside 270–330 mm gets one stopped recheck
     after 120 ms. The probe starts if that batch passes the normal window.
     If the range stays outside or starts farther beyond the window, the width
     policy shows **skipped**, displays the labelled 150 mm assumption and
     holds. E5 bytes 57–58 give the deciding range.
   - faulty or inconsistent triplet → HOLD `RANGE` (never a skip).
3. **Direction and alignment.** Choose the side with the shorter obstacle
   extent from the long scan. If one edge is beyond reach, the near edge must
   be corroborated and the return pass must confirm that the obstacle extends
   farther on the other side to support a specific choice. Starting with
   v0.3.10, a completed scan with valid reference, return and servo checks
   defaults to RIGHT whenever its edge readings give no clear preference.
   The dashboard labels the selection as a default. After choosing, correct
   lateral drift above 6 mm with one clearance-checked forward diagonal of up to 30 mm,
   then restore the original heading. Require Buddy 3's strong tape
   evidence (at least one IR level of 400 or more) for 150 ms within 6° of
   the original heading. The bounded search checks −2°, +2°, −4°, +4°,
   −6° and +6°.
4. **Approach** (sonar-controlled, line-steered): v0.3.11 uses the average
   of three valid stopped forward echoes. If the average is above 215 mm,
   the nearest echo limits the 20 mm/s approach leg. A reading at or below
   205 mm with an average above 215 mm requests a fresh stopped batch.
   v0.3.12 allows up to three stopped batches, 120 ms apart after rejections.
   Each leg is limited to 150 mm, with a live sonar stop at 205 mm or below.
   The mission rechecks range after each leg and allows up to three legs. During each leg the mission
   uses Buddy 3's symmetric steering (`symmetric_targets()`) on each
   fresh pair of calibrated IR levels and turns the wheel
   differential, scaled to 20 mm/s (same turn per distance), into a heading
   setpoint for the drive (`nav_steer_drive`, ±8°, 0.1° steps, lapses after
   100 ms without refresh). The executor keeps its own heading hold, goal,
   timeout, settle and 12° fault. During the first 12 mm of each leg, more
   than 4 mm without tape evidence requests a stop. Once stopped, Buddy 5
   measures the range again and can try one clearance-checked realignment for
   the mission, using in-place turns only. That recovery applies to startup
   loss and to loss after verified tracking. Repeat loss and failed
   clearance checks cause HOLD.
5. **Bypass.** Recheck the original tape sensor pattern and heading while
   stopped. Use the existing
   rectangular route and the averaged stopped range within the 80–220 mm
   planning window. The nearest echo must satisfy first-turn clearance.
   The farthest echo sets how far to pass the obstacle before turning back.
   Modelled heading and lateral pose
   come from the scan plus the completed
   alignment and approach moves. Depth allowance remains 150 mm and the
   lateral leg stays 250 mm.
6. **Return and rejoin.** v0.3.8 keeps the lineward heading at 20 mm/s
   until both sensors verify a stripe or the 300 mm search limit is reached.
   The encoder estimate only sets the earliest plausible scan position.
   The measured stripe centre and sensor lead determine where to turn.
   The robot then checks for fresh tape evidence near the original heading.
   If it cannot confirm the line, v0.3.9 pivots in place toward the stronger
   sensor contrast, or toward the route if neither sensor gives a clue.
   It sweeps up to 28° either side and can confirm the tape with either
   sensor. It stops if the sweep ends without stable tape evidence.
7. **Hand-back** only through `nav_resume_line(…, false)`; the pending route is
   kept and an interrupted junction is refused (existing rule).

The firmware reports `MISSION_RETRY` outcomes and `RANGE_BATCH` entries
with the three echoes in the run ZIP, plus `RETURN_START`, `STRIPE_FOUND`, `POSITIONED`,
`RESTORED`, `CONFIRMED` and `HANDBACK`. A missing stripe or failed alignment
ends the maneuver with `NO_LINE`. The forward sonar stop applies throughout
the return and any advance to the measured stripe centre.

The calibrated width is display-only. Navigation uses the scan's corroborated
edge observations and the fixed route margins. The display is the raw edge midpoint − 10 mm, limited to
100–200 mm; E5/D3 carry all three values.

## Ownership

| Context | Core / priority | Buddy 5 work |
|---|---|---|
| IO_IRQ_BANK0 (board_port.c) | core1 IRQ | GP17 ECHO edge timestamps (`rb5_echo_isr`) |
| Motion task (control.c) | core1 / 14 | Line observation and one mission step right after `motion_update()`; signed wheel demands through the shared wheel controller and `motion_commit()` |
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
