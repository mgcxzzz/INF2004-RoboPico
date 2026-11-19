# Buddy 5 RTOS integration candidate (APP=rtos_buddy1_5 v0.2)

**Status: built and host-checked; not flashed, not driven, not physically
validated.** Base: `barr-c-cleanup` 1862e0d2ce0a5ed26687edc605a749140e1d3a4e
(v0.2.9). `APP=rtos_buddy1_4` is byte-identical to its pre-change build.
v0.2 changes only the mission's width policy, the line-steered approach (with
its coordinator extension) and the E5 status; estimator, direction rules,
bypass and every ported module are unchanged from v0.1.

Build (from `build_make/`, same toolchain/SDK as the release):

```bash
make -j4 APP=rtos_buddy1_5 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0
# -> robopico_buddy1_5_rtos_integration_v0_2_candidate.uf2
```

The sequence, task ownership and adapter rules are in
[app_program/obstacle_navigation/README.md](../app_program/obstacle_navigation/README.md).

## Provenance of the ported algorithms

Folder `firmware/integration/buddy1_4/extensions/buddy5_obstacles/`.

| File(s) | Source | Change |
|---|---|---|
| width_probe.c/.h, width_estimate.c/.h, width_probe_pack.h, b5_scan_pack.h | SDK width_calibrated_probe v0.2 (UF2 b37dd8c3…ff26) | none (byte-identical) |
| edge_probe.c/.h, edge_probe_pack.h | SDK edge_probe v0.1.1 (working direction checkpoint, UF2 2f6070fa…4a89) | near allowance 25 mm (edge_scan v0.1.1 rule), BE/BF version 3; both overridable for the v0.1.1 host checks |
| manual_bypass.c/.h, rect_route.c/.h, stripe_detect.c/.h, b5_event_pack.h | SDK automatic v0.6.1 (identical in the width v0.2 tree) | manual_bypass.h includes b5_compat.h |
| automatic_bypass.c/.h | SDK automatic v0.6.1 (source tarball 655b8e0d…) | guard, stationary scan, profiler side selection and geometry-only variant removed; `b5_auto_start(side, actual range, heading0, lateral0)` replaces begin/scan/plan; DONE state after hand-back; every other function verbatim |
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
- **Default side.** Only a COMPLETE run with 8 records, no loss, all five
  health flags, |final heading| ≤ 3° and equal known side states uses RIGHT,
  labelled DEFAULT (E5 basis 2). Incomplete, faulted, contradictory or
  unknown runs hold (basis 3). Width never overrides a hold.
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

## Telemetry (source 15, schema header 0xF2)

| Marker / length | Meaning | Version |
|---|---|---|
| E5 / 59 | Buddy 5 mission status (v0.2; dashboard also decodes the 54-byte v1 of v0.1) | 2 |
| D1 / 56, D2 / 58 | width records / summary | 4 (unchanged) |
| D3 / 58 | width display result | 1 (unchanged) |
| D4 / 32 | rejected triplet before a retry | 1 (unchanged) |
| BE / 48, BF / 44 | direction records / summary | 3 = v2 layout, near 25 mm |
| B5 / 32 | bypass/rejoin events | 1 (unchanged) |

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
discarded by the car (saturating). v2 only: 54 i8 approach steering
setpoint last sent (0.1°, +left, |v| ≤ 80); 55 largest |setpoint| this run;
56 control frames at the ±8° bound (saturating); 57 u16 range that decided
the width policy (0 none). A skip reports basis 3, status INVALID and display
150.00 mm without any D1–D4 frame.

"Admitted" means accepted into the car's network TX queue. The bridge may
still discard a telemetry message older than 250 ms in that queue
(`rn_tx_take_budget`, counted in its tx_dropped); that and Wi-Fi/MQTT loss are
the remaining loss paths. A refused push keeps its header sequence for the
retry, so source-15 sequence gaps reflect loss after admission.

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

## Host checks run (logic only)

Re-run with `sh testing/rtos_migration/buddy5/run_buddy5_checks.sh <bench_tools/buddy5_sdk>`.


- Frozen width v0.2 suites against the ported files and the barr-c
  coordinator: 337 checks (recorded-capture estimator parity incl. variants,
  ideal-plant D3 after every end) and 200 retry/timing checks.
- Frozen edge_probe v0.1.1 suite: 1298 checks with the exact v0.1.1 rule
  (15 mm, v2) and 1298 with the integration rule (25 mm, v3).
- Saved v0.2 hardware captures re-decoded: four parity OK, 117 mm MISMATCH
  retained.
- Mission simulation (`testing/rtos_migration/test_b5_mission.c`, ASan/UBSan,
  built with `-DMOTION_STEERED_DRIVE`; line model = 10 mm analog spot on a
  19 mm strip, Buddy 3 stand-in = the same `analog_demand()`): evidence
  LEFT/RIGHT bypasses, default side, failed direction → HOLD, faults after
  width REF → HOLD, **REF rejected then clean sonar → HOLD WIDTH_ABORT for
  30 s with 150 mm assumption (policy failed)**, **clean 180 mm stop → skip
  (no probe records, mission continues)**, **faulty / noisy policy range →
  HOLD RANGE (not a skip)**, **three approach pushes (+3°/−2 mm, −1°,
  +1.5°/+1 mm) corrected with steering, not without (edge error at the
  direction probe 0.6/0.5/0.1 mm vs 24.9/7.7/16.1 mm), sonar stop still
  203 mm, setpoint ≤ 8°**, guard sonar failure, line lost during approach
  (HOLD, setpoint bounded), Stop in each of eight stages (no motion
  afterwards). Contact-free in the geometric model.
- Coordinator extension (`testing/rtos_migration/test_nav_steer.c`): NAV_STEER
  codec round trip −8.0…+8.0°, exact length, > 8° refused; refused in AUTO,
  PAUSED, during a turn, after the drive, for another owner or a stale lease,
  NaN; 6° setpoint gives 5.8° odometric yaw with no fault; lapse returns to
  0.2°; a new drive starts straight; Stop during a steered drive stops.
- Dashboard: C-packed frames decoded (node test, 8 cases incl. v1/v2 E5,
  policy and steering), headless smoke of the Buddy 5 panel text
  (`dashboard/test/smoke_b5_panel.js`); existing dashboard tests unchanged
  (four pre-existing failures also fail on the base: missing external
  fixtures).
- ARM build: zero compiler warnings; worst motion-task call chain ≈ 3.0–3.2 KB
  of 4 KB (the steering path inlines into `b5_mission_tick`, 248 B).

None of this is timing, sonar-physics or vehicle evidence.
