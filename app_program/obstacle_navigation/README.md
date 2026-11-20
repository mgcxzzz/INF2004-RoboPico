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
2. **Width policy, then width probe** (SDK width_long_probe v0.2,
   unchanged). Once stopped, a fresh stationary triplet (three clean echoes
   within 15 mm, two attempts) decides the policy (E5 byte 19):
   - inside the probe's own REF window 270–330 mm → the probe is attempted.
     Its REF rule, clearance-limited diagonal per side (20 mm steps, 220 mm
     ceiling, 12 steps, ±46° servo, first-FAR stop, retrace, stationary retry,
     return corroboration), D1–D5 telemetry and the rule-2 on-Pico estimate
     still apply. The RTOS candidate can use the existing single stationary
     retry if two in-window reference echoes agree within 15 mm and the third
     clean echo is within 15 mm of the reference window. The replacement must
     pass the original REF limits. A complete run with an unresolved edge shows its labelled
     fallback and the mission continues. **Any
     non-COMPLETE end holds (`WIDTH_ABORT`, module reason = end code),
     `REFERENCE` included**: END_REFERENCE alone is not evidence of a clean
     skip. The probe's D3 (150 mm `PROJECT_RANGE_ASSUMPTION`) stays the
     displayed number — policy `failed`.
   - clean but outside 270–330 mm → deliberate **skip**: the probe is not
     started, nothing moves for it, the readout is the labelled 150 mm
     assumption (basis 3, status INVALID) and the mission continues to the
     approach — policy `skipped`, with the deciding range in E5 57–58.
   - faulty or inconsistent triplet → HOLD `RANGE` (never a skip).
3. **Alignment**: trim the probe's encoder heading residual (≤ 12°), then
   require stationary line evidence for 150 ms on a sensor that saw the line at
   the takeover. Otherwise up to six slow turns (−4, +4, −8, +8, −12, +12°
   about the trimmed heading), 400 ms per pose. No evidence → HOLD.
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
5. **Direction probe** (edge_probe v0.1.1 motion and rules, near allowance
   25 mm as in edge_scan v0.1.1, BE/BF version 3). LEFT/RIGHT from evidence;
   a complete healthy run with equal sides uses the **default side RIGHT**
   (v0.6.x deterministic fallback), reported as DEFAULT; anything else holds.
6. **Bypass** (automatic v0.6.1 route, return, sensor-guided fourth turn,
   stationary confirmation, bounded search, hand-back): planned from a fresh
   stationary range (80–220 mm) and the probe's encoder heading/lateral
   offsets. Depth 150 mm, no reverse, 50 mm margin, side 250 mm (unchanged).
7. **Hand-back** only through `nav_resume_line(…, false)`; the pending route is
   kept and an interrupted junction is refused (existing rule).

Width is a display estimate. It never sets clearance, authorises motion or
clears a hold. The display is the raw edge midpoint − 10 mm, limited to
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
