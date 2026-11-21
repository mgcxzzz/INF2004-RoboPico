# Buddy 5 RTOS candidate on v0.2.12: long width probe and straight-only takeover (APP=rtos_buddy1_5)

**Status: built and host-checked; not flashed, not driven, not physically
validated.** Base: `barr-c-cleanup` v0.2.12 `81a994f57e3828364492cc2806fcbea124dfa18a`
(caster assist). It carries the Buddy 5 RTOS integration v0.2 (checkpoint
`buddy5-rtos-integration` 573689b, on v0.2.9) and replaces its short width
probe with the SDK `width_long_probe` v0.2 (UF2 71ae5473…). Candidate 0.2 adds
the straight-only takeover gate (below). `APP=rtos_buddy1_4` is byte-identical
to a clean v0.2.12 build with the same toolchain.

Build (from `build_make/`; run `make … clean` first when switching APP, the
object directory is shared):

```bash
make -j8 APP=rtos_buddy1_5 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0
# -> robopico_buddy1_5_rtos_v0_2_12_straight_gate_candidate.uf2
```

The sequence, task ownership and adapter rules are in
[app_program/obstacle_navigation/README.md](../app_program/obstacle_navigation/README.md).

## v0.2.12 overlaps (resolved in favour of the release)

| Release change (v0.2.9 → v0.2.12) | With Buddy 5 |
|---|---|
| Caster assist: `rl_controller_output`, `no_progress` → `BR_REASON_STALL` stop | Kept. The stall stop is in the non-Buddy-5 power branch only; Buddy 5 moves use their own signed controller and never feed the caster model. After hand-back the follower's progress check re-anchors on the first ≥ 2 mm of travel. |
| `motor_stop_reason` in the status and the line log | Kept; Buddy 5 holds do not call `bringup_stop_from`, operator Stop/E-stop paths do (and still call `rb5_cancel()`). |
| C2 release frame 12 → 13 bytes (`info[16]`) | Kept; no source-15 marker or length collides with E5 (54/59/61) or D1–D5. |
| Safety task `else if (active && !rn_drive_online_record(true))` | Kept unchanged; network loss stops Buddy 5 moves as before. |
| `connected_control.c` clock read order, `line_service.c` caster enable | Kept (auto-merged). |
| Dashboard run ZIP (jszip), release banner | Kept; the Buddy 5 panel is additive. The Buddy 1–4 banner text in `app_main.c` is the release text; the Buddy 5 banner names this candidate. |

## Provenance of the ported algorithms

Folder `firmware/integration/buddy1_4/extensions/buddy5_obstacles/`.

| File(s) | Source | Change |
|---|---|---|
| width_estimate.c/.h, width_probe_pack.h, b5_scan_pack.h | SDK width_long_probe v0.2 (UF2 71ae5473…) | none (byte-identical) |
| width_probe.c/.h | SDK width_long_probe v0.2 | `B5WP_RUN_ARCHIVE` guards the write-only run-log copy `arch[]`. The RTOS profile omits that copy to save 3,172 B RAM. RTOS hooks permit one stationary resample at eligible REF (v0.2.15) and face-relative poses (v0.3.5) through the existing retry budget and original acceptance rules. The RTOS profile retains one terminal rejected D1 for republishing. |
| edge_probe.c/.h, edge_probe_pack.h | SDK edge_probe v0.1.1 (working direction checkpoint, UF2 2f6070fa…4a89) | near allowance 25 mm (edge_scan v0.1.1 rule), BE/BF version 3; both overridable for the v0.1.1 host checks |
| manual_bypass.c/.h, rect_route.c/.h, stripe_detect.c/.h, b5_event_pack.h | SDK automatic v0.6.1 (identical in the width v0.2 tree) | manual_bypass.h includes b5_compat.h |
| automatic_bypass.c/.h | SDK automatic v0.6.1 (source tarball 655b8e0d…) | guard, stationary scan, profiler side selection and geometry-only variant removed; `b5_auto_start(side, actual range, heading0, lateral0)` replaces begin/scan/plan; DONE state after hand-back; v0.3.8 searches straight toward the line up to 300 mm and aligns only after a two-sensor stripe crossing; v0.3.9 adds a bounded sweep up to 28° either side and allows either sensor to confirm rejoin |
| sonar_async.c/.h, servo_slew.h, profile/* | SDK width v0.2 tree | none |
| signed_continuous.h | RTOS v0.1 Buddy 5 adapter / SDK automatic | barr-c type names only |
| b5_compat.h | new | aliases the SDK names (MotionStatus…) to the barr-c coordinator types |

These ported files keep their SDK style for traceability (exception CEX-08).
New application code (`app_program/obstacle_navigation`) follows RoboPico C
Standard 1.0 and is in the mechanical/format check scope.

## Decisions and their evidence

- **Width at ~300 mm, direction at ~200 mm.** Each algorithm runs at the
  distance where it was validated; the width result is not reused for the
  direction, and the direction thresholds are not applied at 300 mm.
- **Trigger.** Three clean echoes ≤ 300 mm (detection not expanded beyond
  300 mm). With the 45 mm/s cruise the stop is expected inside the width REF
  window (270–330 mm), which is NOT loosened.
- **Width policy (v0.2, review 2026-11-19 item 1).** After the stop a fresh
  stationary triplet (clean, spread ≤ 15 mm, two attempts) decides it before
  the probe is touched. Inside 270–330 mm the probe is attempted, and any
  non-COMPLETE end — `REFERENCE` included — holds (`WIDTH_ABORT`, module
  reason = end code) with the probe's D3 150 mm assumption still displayed
  (policy 3 `failed`). A clean range outside the window is a deliberate skip:
  the probe is not started, nothing moves, the readout is the labelled 150 mm
  assumption (basis 3, INVALID), the mission continues (policy 2 `skipped`,
  deciding range in E5 57–58). A faulty/inconsistent triplet holds `RANGE`.
  END_REFERENCE is therefore never read as a clean skip. The probe's own REF
  check is unchanged and may still reject a triplet the policy accepted
  (e.g. a fault at REF) — that is a failure and holds. A skip below 270 mm
  can leave the robot nearer than 215 mm; the approach then starts the
  direction probe from there (no reverse leg; unchanged behaviour).
- **Near allowance 25 mm (direction).** Re-decoding all nine accepted
  checkpoint captures with 25 mm leaves every triplet class, side state and
  decision unchanged (`testing/rtos_migration/buddy5/direction_near25_replay.py`); edge_scan v0.1.1
  ran five centred hardware runs with it (all complete, HOLD). The two earlier
  holds it addresses had echoes 16–18 mm short of the model.
- **Default side in v0.3.10.** Buddy 5 uses the consolidated width scan.
  A complete scan with valid reference, return and servo checks and no
  lost records can choose LEFT or RIGHT from the edge evidence. If that
  evidence gives no clear preference, it chooses RIGHT and reports DEFAULT
  (E5 basis 2). Probe failures and skipped scans still hold.
- **Ownership.** One lease from the takeover to `nav_resume_line`; the probes
  re-claim through `nav_request_takeover`, which returns the same lease while
  Buddy 5 is PAUSED. Buddy 3 never runs between the width and direction
  probes, so the guard cannot re-trigger and the route cannot be handed back
  early. Consequently the 300→200 mm approach is a Buddy 5 finite drive,
  not Buddy 3's follower — in v0.2 **line-steered** (below).
- **Line-steered approach (v0.2, review item 2).** Each fresh observation
  during an approach leg runs Buddy 3's own edge law, `analog_demand()` from
  `line_logic.h` (left sensor on the strip's right edge at 500, deadband
  ±60, right-sensor guard), on the levels the line provider already
  normalised (identity calibration 0/1000). Its wheel differential d (at the
  follower's 45 mm/s) becomes a yaw rate scaled to the 20 mm/s approach
  (same turn per distance) and is integrated into a heading setpoint
  (dt ≤ 50 ms), bounded to ±8° and sent with `nav_steer_drive(BUDDY5, lease,
  deg)`. BOTH_BLACK and the follower's stop decisions give no turn; while the
  line is white the follower's seek-left differential is kept. The mission's
  40 mm line-loss hold, sonar stop, three-leg limit and every hold are
  unchanged. Refusal other than end-of-drive BUSY stops the leg and holds
  (`MOVE`). Not tuned on hardware.
- **Probe-induced drift.** The bypass adopts the direction probe's encoder
  heading (route legs and the fourth-turn target stay referenced to the
  approach heading) and lateral offset (return-distance estimate). The width
  probe's residual is trimmed before the line-evidence alignment.

## Coordinator extension `MOTION_STEERED_DRIVE` (rtos_buddy1_5 only)

`nav_steer_drive` / `motion_steer_drive(owner, lease, degrees)` goes through
the nav codec as kind `NAV_STEER` (13: lease + 8-bit 0.1° value, exact
length; > ±8.0° is rejected by the encoder and the coordinator). The
coordinator accepts it only from the lease holder while its own straight
drive is executing (mode DRIVING, executor active, not turning or settling);
otherwise BUSY/NOT_OWNER/INVALID. It sets the executor's heading offset
(`steer_mm` = degrees × 110 mm × π/180, i.e. the wheel-difference target of
the existing heading hold); the hold, goal, timeout, settle and 12°
EXEC_HEADING fault are unchanged. The offset lapses to straight 100 ms after
the last accepted call and is cleared at every new move. Per-tick BUSY at the
end of a drive is not audited (as for Buddy 3's wheel demands). Everything is
under `#ifdef MOTION_STEERED_DRIVE`, defined only for `APP=rtos_buddy1_5`;
the `#else` paths are the original text, and the rtos_buddy1_4 UF2 is
byte-identical. Historical `package_release.py` source fingerprints that list
these coordinator files will differ textually.

## Clearance audit (compact v0.6.1 route, unchanged)

Values from `rect_route.c` (configured body: 135 × 190 mm, axle 130 mm from
the front, margin 50 mm, side 250 mm, depth 150 mm, no reverse):

- First in-place turn needs the forward echo ≥ about 142 mm (sweep radius
  146.5 + margin − sonar offset); otherwise BLOCKED (`HOLD_CLEARANCE`).
- Forward pass = range + 365 mm (566 mm at 200 mm): clears a ≤ 150 mm-deep
  obstacle plus the rear-corner swing of the line-ward turn.
- Lateral: 250 − 67.5 − w/2 − (offset toward the pass side). With the
  direction probe choosing the side away from the offset the margin grows;
  a 185 mm box offset 40 mm TOWARD a default side leaves 50 mm.
- **Unverified:** boxes deeper than 150 mm or wider than 200 mm, side clutter
  (no stationary sweep exists any more; only the forward echo enters the
  first-turn check), real heading error of the probe poses, and line
  acquisition after the probes. The width estimate does not and must not
  prove clearance.

## Straight-only takeover gate (straight-gate candidate 0.2)

The width probe, the direction probe and the bypass assume a straight line
segment, but Buddy 3 is in AUTO on curves too. The guard therefore asks for
the takeover only while `b5_straight.c` reports positive, fresh evidence that
Buddy 3 is stably following a straight. **Straight eligibility controls where
the manoeuvre starts; it does not change what counts as an obstacle.** The
sonar still treats any close reflector (a box, a wall on a curve, a person's
leg) as a possible obstacle.

- **Inputs** (`rtos_line_straight_sample()`, motion task, right after
  `rtos_line_follow()`, observation only): circuit phase, busy and fault,
  follower white and recovery search, the demand decision and stop, the
  qualified junction edge and barcode cross mark, a barcode route action or
  U-turn in progress, the continuous steering's `active` and bend flags
  (dwell, candidate, confirmed, exiting), and the encoder wheel travel.
  Nothing in Buddy 3 is changed.
- **Open** only after every frame of the last 60 mm of travel (and at
  least 300 ms) was ordinary straight following: phase `C_FOLLOW`, decision
  24/25 (continuous steering) or straight/left/right/fast, no junction, turn,
  loss or recovery, and an encoder heading change of at most 6° across the
  window and 3° over roughly 30 mm of recent travel. The shorter history
  lets a straight after a bend qualify sooner. The recent check still blocks
  tight curve entry.
- **Closed** reasons (E5 v4): settling, stale (> 50 ms without a Buddy 3
  frame), unavailable (provider not running, bend monitor inactive), bend
  entry, bend, bend exit, curve, junction, turn, line lost/recovery, other.
  Any closed frame restarts the evidence.
- **Echo counting.** Three consecutive close echoes (≤ 300 mm) whose pings
  were *requested after the gate opened* trigger the takeover, still
  requested while Buddy 3 owns the motion. Losing the gate discards the
  count; an echo from before the opening, from a curve or in flight across
  the opening never counts.
- **Blocked close obstacle.** Three consecutive close echoes while the gate is
  closed can stop the robot and hold `NOT_STRAIGHT` (E5 reason 19, module
  reason = the gate reason). On a settling straight, Buddy 5 performs a
  stationary recheck. A clean 270–330 mm batch with retained tape evidence
  starts the normal width probe. v0.4.3 also admits one echo down to 260 mm
  when the other two are in-window, all three agree within 15 mm, and the
  nearest echo passes the turn clearance check. This reports a range warning
  and enables the same check for that mission's fresh probe reference.
  A clear batch allows hand-back, and an
  inconclusive batch gets one stationary retry. During a left, right or
  U-turn, Buddy 5 suspends
  guard pings through line reacquisition and discards prior echoes. It resumes
  guarding with fresh pings after the turn. Straight-through junctions use the
  ordinary guard. Stop, E-stop and network supervision remain active.

## Width in the mission (long probe)

At the ~300 mm stop (clean stopped range 270–330 mm) the mission runs the SDK
long probe unchanged: 20 mm adaptive steps, 5 mm overshoot reserve, 220 mm
outer ceiling, 12-step limit per side, ±46° servo cap; every move is bounded
by `b5_wp_clear_travel()` (front corner ≥ 50 mm from the modelled face plane
25 mm nearer than measured, 5° heading reserve). An outward pass stops at the
first all-FAR (FAR), the ceiling (CEILING), clearance (CLEARANCE), the step
limit (STEPS) or a fault (ABORT); it then retraces, with the stationary
retry and the return corroboration. Estimator rule 2 (basis 4
`CORROBORATED_EDGE_ESTIMATE` for clean-FAR evidence and adjacent centre
anchors on a corroborated side). Display: raw edge midpoint, −10 mm, limited
to 100–200 mm; an unresolved side gives the labelled 150 mm assumption, or
the empirical completion only after the CEILING. Width stays display-only:
it never changes bypass clearance, authorises motion or clears a fault.
Skip (clean range outside the window: no probe), failure (attempted probe
held) and a completed probe with unresolved edges (labelled assumption,
mission continues) are reported distinctly.

## Telemetry (source 15, schema header 0xF2)

| Marker / length | Meaning | Version |
|---|---|---|
| E5 / 61 | Buddy 5 mission status (dashboard also decodes v3/61, v2/59 and v1/54) | 4 |
| D1 / 56, D2 / 58 | width records / summary (long probe) | 10 |
| D3 / 58 | width display result, rule 2 | 3 |
| D4 / 32 | rejected triplet before a retry | 3 |
| D5 / 36 | per-side outward limits (stop reason, reach, steps, clearance allowed, min turn gap) and the policy constants, after D2 | 2 |
| BE / 48, BF / 44 | direction records / summary | 3 = v2 layout, near 25 mm |
| B5 / 32 | bypass/rejoin events | 1 (unchanged) |

Older captures stay readable under their original rules: the dashboard and
`tools/telemetry/b5_rtos_decode.py` decode the v0.1/v0.2 short-probe
protocol (D1/D2 v4, D3 v1, D4 v1, E5 v1/v2) with the original basis table,
and reject a run that mixes protocol versions. Length 36 is used elsewhere only by source 2.

E5 layout (little endian; byte offsets in the payload): 0 u32 Pico ms; 4 0xE5;
5 version 2; 6 u16 mission run; 8 stage; 9 hold/end reason; 10 module
end/reason; 11 coordinator owner b0–2, mode b3–5, session b6, stopped b7;
12 u16 flags (b0 armed, b1 re-arming, b2 ping ≤ 250 ms, b3 width attempted,
b4 width result, b5 direction attempted, b6 default side, b7 bypass, b8
backlog, b9 servo settled, b10 hold, b11 start-ready; b12–15 zero); 14 u16
last ping mm (0xFFFF none); 16 u16 ping age ms; 18 ping class b0–1, why
b2–5; 19 width policy (0 none, 1 measured, 2 skipped, 3 failed/HOLD,
4 running; zero in v1); 20/22 i16 servo requested / applied PWM command (0.1°);
24 decision; 25 basis (0 none, 1 evidence, 2 default, 3 held); 26 side states
L b0–3, R b4–7; 27 direction end; 28 side used (0 none, 1 right, 2 left);
29 width end; 30 width basis; 31 status; 32 reason; 33 retries; 34/36/38 u16
display/raw/corrected mm×100 (0xFFFF n/a); 40 width d0; 42 trigger echo mm;
44 bypass start range mm; 46 bypass state; 47 rejoin phase; 48/49 width
records produced/admitted; 50/51 direction records produced/admitted;
52 approach legs b0–3, alignment steps b4–7; 53 records of an earlier run
discarded by the car (saturating). v2 and v3: 54 i8 approach steering
setpoint last sent (0.1°, +left, |v| ≤ 80); 55 largest |setpoint| this run;
56 control frames at the ±8° bound (saturating); 57 u16 range that decided
the width policy (0 none). v3 only (61 bytes): 59 width-probe state b0–3
(IDLE CLAIM REF TURN PRE STEP OUT RETRACE RET TURN_BACK FINAL DONE HOLD TRIM
LADDER MID) | side of the last stopped sample b4–5 (0 none, 1 left, 2 right);
60 sample phase b0–2 (REF PRE OUT RET FINAL LAD_OUT LAD_RET MID) | stop step
b3–7. Byte 30 may be basis 4 in v3. A skip reports basis 3, status INVALID
and display 150.00 mm without any D1–D5 frame. v4 (same 61 bytes): flags
b12–15 = takeover gate reason (0 unknown, 1 STRAIGHT = allowed, 2 settling,
3 stale, 4 unavailable, 5 bend entry, 6 bend, 7 bend exit, 8 curve,
9 junction, 10 turn, 11 line lost/recovery, 12 other); 59 b6–7 = close
echoes counted (toward the takeover while open, toward the NOT_STRAIGHT hold
while closed); reason 19 = NOT_STRAIGHT with byte 10 = the gate reason. v3
frames must keep those bits zero.

Verification labels (`b5_rtos_decode.py --sdk-width-decoder …/width_long_probe_v0_2/decode_width_long.py`):
VERIFIED BY REPLAY only when every claimed record, D2, D3 and D5 were received
and the unchanged SDK decoder reproduces the estimate, the clearance policy and
the limits; PICO-REPORTED when anything is missing (the car's number is shown,
never repaired); MISMATCH; INSUFFICIENT; NO RECORDS. The dashboard never claims
verification: it says "Pico-reported · capture complete (replay offline)" or
"NOT independently verifiable (capture incomplete)".

"Admitted" means accepted into the car's network TX queue. The bridge may
still discard a telemetry message older than 250 ms in that queue
(`rn_tx_take_budget`, counted in its tx_dropped); that and Wi-Fi/MQTT loss are
the remaining loss paths. In v0.2.13 the MQTT consumer assigns source-15
sequence numbers in dequeue order. Expired queued frames consume a sequence.
Refused admissions are counted separately and can be retried.

## Missing-record audit (117 mm capture, width v0.2)

`testing/rtos_migration/buddy5/missing_record_audit.py`: D1 indices 8 and 9 are missing; the
width runtime's header sequence jumped 133→136 between indices 7 and 10, and
it only advances after `b5_runtime_send()` succeeded, so both records left
the firmware send path. Every other stream shows the same 3.5–4 s hole
(source-9 motion: 80 frames). The loss is downstream of the firmware publish
(Wi-Fi/MQTT/broker/capture), not enqueue or decoding. The computer replay
still rejects that run (150 mm INVALID) and the Pico value 143.85 mm stays a
non-parity result. The RTOS candidate adds E5 produced/admitted counts and
dashboard/offline completeness reports; it never fills or re-labels a gap.

## Resources (rtos_buddy1_5)

- Straight-gate candidate 0.2: `__bss_end` 0x20032628 (+240 B: gate
  window and counters), pool margin **808 B**; motion-task worst chain
  3,512 B of 4,224 B (control 1,024 + rb5_frame 152 + the width chain below;
  the gate step is 48 B on the shallow guard path). The figures below are
  for candidate 0.1.
- Static RAM end `__bss_end` 0x20032538 (56,008 B below 0x20040000; v0.2
  candidate 0x20032080; v0.2.12 Buddy 1–4 0x2002ff98).
- Kernel pool model (`testing/rtos_migration/buddy5/kernel_pool_budget.py`):
  every task stack and message buffer comes from one pool between
  `__bss_end` and the 2 KB exception stack; nothing allocates after init.
  Pool 53,960 B, boot allocations 52,912 B, **margin 1,048 B** (Buddy 1–4
  release: 13,888 B; v0.2 candidate: 1,232 B). Without the two adjustments
  below the long probe would have been 3,144 B short and the last tasks
  would fail with E_NOMEM (`INIT FAILED; motors disabled`).
  - `B5WP_RUN_ARCHIVE=0`: no 61-record run-log copy (−3,172 B).
  - Buddy 5 task stack 3,072 B instead of 4,096 B (`RB5_TASK_STACK`,
    `start_sized()` under `APP_RTOS_BUDDY5`). Deepest chain: rb5_task 8 +
    rb5_service 304 + report 424 + tm_printf 800 + tm_vsprintf 104 + output
    ≈ 1.8 KB; telemetry chain ≈ 1.0 KB.
- Motion task (4,096 B + 128 B system): worst chain control 992 + rb5_frame
  112 + b5_mission_tick 248 + b5_wp_tick 216 + finish 176 + b5_we_estimate
  576 + side_onset 840 + median4 280 = 3,440 B (v0.2: ≈ 3.0–3.2 KB). The
  64-point estimator adds ≈ 0.3 KB; no change needed.
- Capacities: 61 records per run (static assert), 64-record send queue,
  64 estimator points; D5 drained after D2 (`b5_wp_begin` refuses while it
  is pending). Transmission stays nonblocking.
- The first idle boot confirms the pool model: every `TASK … result=0`, the
  Buddy 5 line shows `stack=3072`, and no `INIT FAILED`.

## Host checks run (logic only)

Straight gate (candidate 0.2):
- `testing/rtos_migration/test_b5_straight_gate.c` (real Buddy 3 bend
  monitor, `CONTINUOUS_BEND_ASSIST=1`): straight with small corrections
  opens after 207 mm and stays open over 2.7 m; left and right R 250 mm
  curves block from bend entry through bend and exit and re-open 217 mm
  after the curve; gentle R 500/1200 mm curves without bend flags never open
  from closed and close within the heading budget when entered; every
  junction, turn, route turn, loss, recovery, stop, fault, unavailable and
  unexpected-decision state blocks and restarts the evidence; stale after
  50 ms; 923 checks.
- Mission simulation: the integrated straight-line mission completes as
  before (takeover with the gate open); a close obstacle with a bend, bend
  exit, junction mark, recovery search, unavailable or stale Buddy 3 state
  holds `NOT_STRAIGHT` about 355 mm from the face with no probe or bypass,
  and operator Stop still ends it; close echoes in a bend and an echo in
  flight across the opening never count; two qualifying echoes are
  discarded at a bend entry and three fresh ones are needed after the gate
  re-opens.

Re-run with `sh testing/rtos_migration/buddy5/run_buddy5_checks.sh <bench_tools/buddy5_sdk>`.

- SDK width_long_probe v0.2 suites against the ported files and the v0.2.12
  coordinator: near-axis estimator and recorded-fixture checks; 958
  focused integration checks (clearance, ceiling, steps, first-FAR, retrace,
  retries, 61-record maximum, D5). The SDK v0.2 decoder replays the
  RTOS-built simulation: 13 runs, 0 mismatch, 0 insufficient (same bases,
  statuses and limits as the SDK simulation; widths differ by ≤ 0.35 mm
  because the v0.2.12 executor's simulated poses differ slightly).
- With the RTOS reference retry hook disabled, width_probe.c/.h differ from
  the SDK only by the archive guard. The mission
  telemetry with the guard on and off is byte-identical.
- Frozen edge_probe v0.1.1 suite: 1298 checks (15 mm, v2) and 1298
  (25 mm, v3); near-25 replay of nine accepted captures: no change.
- Mission simulation (`testing/rtos_migration/test_b5_mission.c`, ASan/UBSan):
  the v0.2 scenarios (evidence LEFT/RIGHT, default side, failed direction →
  HOLD, faults after width REF → HOLD, REF rejected → HOLD with the labelled
  assumption, clean skip, faulty/noisy policy range → HOLD RANGE, three
  steered approach pushes, guard sonar failure, line lost in approach, Stop
  in each of eight stages) plus long-probe checks: width → align → range →
  approach → direction → bypass in one mission; D2, D5 and D3 drained;
  every outward pass ≤ 220 mm and ≤ 12 steps with a reported stop reason;
  modelled turn and front gaps ≥ 50 mm. Contact-free in the geometric model.
- Offline decoder on every simulated mission: 16 VERIFIED BY REPLAY, one
  PICO-REPORTED (the deliberately dropped D1), one INSUFFICIENT (REF
  rejected), one NO RECORDS (Stop before REF); a corrupted D3 gives MISMATCH.
- Coordinator extension (`testing/rtos_migration/test_nav_steer.c`): unchanged, passes.
- Dashboard: `test_b5_rtos.js` 13/13 (long and legacy fixtures, D5, E5 v3,
  raw/corrected/display, mixed-protocol rejection), headless panel smoke;
  the other dashboard suites fail exactly as on the clean base (missing
  host-build fixtures).
- ARM: zero compiler warnings (one pre-existing linker RWX warning);
  RoboPico C Standard mechanical and clang-format 18.1.8 checks pass.

None of this is timing, sonar-physics or vehicle evidence.
