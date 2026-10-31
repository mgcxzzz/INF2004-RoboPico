# Buddy 5 bench test — profiling and bypass-side choice (S5-C07, part 1)

Goal: check on real hardware (HC-SR04+ and servo, wheels never turn) that the conservative profiler and the side choice behave as designed before porting into the integrated firmware. Exact ruler width is secondary; what matters is: repeatable observed span, planning width ≥ 200 mm, correct side (or no side), and unknown never treated as clear.

## Firmware

`bench_tools/buddy5/buddy5_bench_sim_v0_1_settle40.uf2` (and `_settle80.uf2`, only for step 5). Built from this worktree (`buddy5` + uncommitted 10-28 profiler + scan dump), with `SYS_SIM_CAR 1`, `SYS_SIM_LINE 1`, `WIFI=none`: a simulated car and line, real sensor and servo. No Wi-Fi, no credentials. sha256 in `buddy5_bench_sim_v0_1_SHA256SUMS.txt`.

Each scan prints `[b5 profile]` (observed span, model allowance, planning width, evidence, clearance L/R), `[b5 scan]` (every angle: `angle_dd:range_mm`, `x` = no valid echo; − = left, + = right) and `[b5 decision]` (side or NONE, confidence, closest point, edges, arc-limit flags, needed clearance = car 155 + 2 × 50 = 255 mm).

## Setup

- Pico W USB to the Mac only (Robo Pico power unplugged). Car on the desk, sensor facing a clear area (nothing within ~60 cm except the test objects).
- Distances are from the sensor front to the obstacle face. The car stops (Confirm) only below **300 mm**, so test at **250 mm** and **290 mm**.
- Record actual obstacle width and depth (the planner assumes 200 mm deep).
- Log one file per case:
  `cat /dev/cu.usbmodemMTK3PICO1 | tee logs/2026-10-31_S5-C07_<case>.txt` (Ctrl-C and start a new file for the next case; no reset needed).

## How one scan works on the bench

Run starts 2 s after power-up (`Guard`). Put the obstacle in place → `Confirm` → `Halting` → `Coarse Sweep` → `Edge Refinement` → `Planning` (the three lines above). Then:
- side chosen → `Bypassing` (simulated): **remove the obstacle at once**, wait for `Guard`, put it back for the next scan;
- side NONE → `Blocked`: the simulated car reverses and rescans (each rescan is another scan in the log); after the retries it faults → press **GP21** to start again.

## Cases

| # | Case | Repeats | Expected |
|---|---|---|---|
| 1 | 100 mm box, centred, 250 mm | 3 | span repeatable (record it; it may read wide), planning ≥ 200, a side chosen |
| 2 | 150 mm box, centred, 250 mm | 3 | same |
| 3 | 200 mm box, centred, 250 mm | 3 | same; planning ≥ observed span |
| 4 | 1–3 at 290 mm | 2 each | same behaviour at the stop distance |
| 5 | 150 mm box, 250 mm, with `_settle80.uf2` | 3 | compare with case 2; > 5 % change → keep 80 ms |
| 6 | 150 mm box offset 50 mm left / right | 3 each | edges shift with the box; side still valid |
| 7 | 150 mm box + wall/box on the RIGHT ~250 mm out at about 45° | 3 | **LEFT** |
| 8 | mirror of 7 (wall on the LEFT) | 3 | **RIGHT** |
| 9 | box + walls on both sides (gap narrower than ~255 mm beyond the box) | 2 | **NONE**, stays stopped/Blocked, never Bypassing |
| 10 | wide board across the whole ±55° arc | 2 | arc-limit flags set, NONE |
| 11a | wave a hand in front of the sensor on one side during the sweep (noisy readings) | 2 | `x` (unknown) angles on that side; must not choose that side through unknown space |
| 11b | soft object (cushion/cloth) as the obstacle | 2 | record only: an ultrasonic sensor may read soft surfaces as far/open; that is a sensor limit, note it |

Pass for the logic: 7 → LEFT and 8 → RIGHT every time; 9, 10 and the unknown side in 11a never chosen; planning width never below 200 mm and never smaller than the observed span. Spans are reported, not graded against the ruler.

## After the runs

`python3 parse_b5_bench.py logs/2026-10-31_S5-C07_*.txt` prints one row per scan and a summary per case. Add rows to `TEST_LOG.md`. Bypass itself (turns, clearance, line reacquisition) is part 2, on the car after the port into the integrated firmware.

## Results and decisions (2026-10-31 evening)

**RTOS bench firmware ultrasonic fault.** After one good run (17:52), the micro T-Kernel bench image read a fixed 236 mm (ECHO high 4 µs after TRIG, low at exactly +1384 µs, or ~+150 µs) whatever the target; confirmed by IRQ and by direct pin polling; sensor unplugged → no signal; the standalone SDK tool on the same hardware reads correctly (110 / 308 mm). Trigger code is equivalent. Open issue for the RTOS port (`buddy5_bench_sim_v0_1_pingdebug*.uf2`, logs `2026-10-31_S5-C07_pingdebug*.txt`). The profiling tests therefore used an SDK bench (`bench_tools/buddy5/b5_profile_bench_v0_x.uf2`, source `testing/subsystem5/bench/b5_profile_bench/`) that runs the real `obstacle_profile.c` with the firmware's scan sequence and validity rule.

**Findings (logs `2026-10-31_S5-C07_case1_100mm_250.txt`, `_case2_180mm_250.txt`, `_v02_*.txt`, `_v03_150_110.txt`):**
- At ~250 mm both boxes read 155–235 mm wide (flat-face mirror effect: a face echoes from straight ahead over ~±20–25°, so span ≈ 2·d·tan 22° regardless of the box). At ~150 mm the 100 mm box read 98–100 mm in 2 of 3 scans; the 180 mm box read short (84–144 mm).
- 10-28 rule (unstable triplet = unknown): a side was chosen in 3 of 12 open-room scans; the unstable sample at the box edge (mixed box/miss pings) or intermittent far echoes (0.7–1 m) blocked both sides.
- Unstable triplet = occupied at its nearest reading: a side was chosen in 11 of 12; the side with a real object at ~0.8 m was never chosen; the remaining NONE had objects close on both sides.
- Offset box at 250 mm: edges/hit counts did not follow the offset (flat face). At ~150 mm a left-shifted box was seen (left clearance ~65 mm, hits leaning left → RIGHT 3/3); a right shift was not (possible servo asymmetry; user sees the sweep go further right — calibration deferred).
- With both sides open, clearances differ by only 10–20 mm (noise) and the choice flipped.

**Changes (buddy5 worktree, uncommitted):** `B5_UNSTABLE_AS_NEAREST 1` (task_obstacle.c sample_at), `B5_SIDE_TIE_MM 50` (near-tie → preferred side), `B5_LATERAL_MIN_MM` = 200 + car/2 + margin = 327 mm (obstacle_profile.c). Host suite passes; new `testing/host/test_b5_bench_replay.c` replays 8 real bench scans (31 checks: clearances identical to the bench, expected sides, planning ≥ 200, lateral ≥ 327). RTOS image compiles with 0 warnings. Stop distance unchanged (300 mm) pending a decision (data favours ~150 mm).

## Rule B — supervised open-course rules (2026-10-31, after the external review)

Decision (n1sh0): go with option B for supervised testing, with conditions. Neither the bench nor the host tests prove a bypass is safe; that still needs physical validation.

**What changed in code (buddy5 worktree, uncommitted; pre-change snapshot kept outside the repository):**

| Condition | Implementation |
|---|---|
| Measured echo vs. completed no-return vs. fault | `b5_classify_ping()` (pure, shared by RTOS, bench, host tests). ECHO must read low before TRIG (`B5_PRETRIG_LOW_WAIT_MS` 100), else FAULT stuck-high and no trigger. Rise + fall with width ≤ `ECHO_MAX` = ECHO; rise + fall with width > `ECHO_MAX` = NO_RETURN; rise without fall in `B5_NORETURN_WAIT_MS` (100) = FAULT; noise only = FAULT; no rise = FAULT. The RTOS driver no longer reports a still-high ECHO as "no target". |
| Three clean no-returns = tentative clearance, weaker evidence | `b5_combine_pings()`: 3 NO_RETURN, no fault → sample `b_weak` (range FAR). Evidence flag `WEAK_CLEAR` (0x10) when the chosen side relied on it. `confidence_pct` now counts measured echoes only; new `coverage_pct` counts all usable samples and is the gate in Planning. |
| Credible near echo blocks; mixed/incomplete stays unknown | Any echo → occupied at the nearest echo (stable only if all pings echoed within 20 mm). Echo + 1 fault → `b_partial`: blocks, nearer space weak only. Echo + ≥2 faults → `b_block_only`: blocks, otherwise unknown. No echo with any fault → unknown. Replaces `B5_UNSTABLE_AS_NEAREST`. |
| Check the actual swept footprint | `b5_check_sweep()` replays the coordinator's manoeuvre: spin dir·45° on the spot, diagonal lateral·√2, spin back, pass leg forward − lateral, spin −dir·45°, rejoin lateral·√2 + 100 mm search (cap 600). The car rectangle grown by the 50 mm margin is checked at every pose (20 mm / 5° steps). FAIL: overlaps the planning envelope, a measured return inside it, reaches behind a measured side return, or crosses scanned rays with no usable evidence. PASS_ASSUMED: parts lie where the scan cannot see (behind the sensor, outside ±55°, behind the obstacle within the 200 mm depth assumption). A side is chosen only if its gap test AND footprint pass; the 50 mm tie band applies only among passing sides. Pivot-to-front 150 mm is **ASSUMED (measure it)**. |
| Forward monitoring is an extra precaution only | Printed on every plan (`[b5 sweep] forward watch ... sides and rear during turns are not monitored`); comment on `range_once()`. |
| 150 mm provisional; stable span ≠ planning width | `B5_D_STOP_MM` stays 150. New `stable_span_mm` (span of stable near returns only) printed separately from `planning_width_mm`. |

**What the footprint check found (host tests + replay of today's scans):** from a stop at ~130–170 mm, the grown car's front corner during the 45° spin, or its inner side on the diagonal, meets the ≥200 mm planning envelope's near corner. Direct bypass from the stop point fails; the existing Blocked → reverse 150 mm → rescan path then passes (statechart scenario 13). Replay: 4 of 8 bench scans change to NONE (cases12_s1: rejoin search reaches behind a right-side return at ~655 mm; cases12_s4 at ~251 mm and d150_s9/s10 at ~167 mm: diagonal meets the envelope). Geometry needs a standoff of roughly ≥ 215 mm (45° exit, 200 mm floor, 50 mm margin, assumed pivot) — decision needed (see handoff).

**Host tests:** `make -C testing/host` passes (log `logs/2026-10-31_ruleB_host_suite.txt`). New `test_b5_ruleb.c` (46 checks): classifier, combiner, persistent ECHO-high / no-fall / disconnected / noise scans (no side, 0 % coverage), mixed near/no-return and incomplete triplets, both sides blocked, enlarged lateral exceeding the measured gap (gap test passes, footprint fails → NONE), tie band only among passing sides, unscanned footprint flagged. `test_b5_task` scenarios 10–13 (ECHO stuck high from the start, stuck high from the sweep, unplugged from the sweep, reverse-then-bypass). `test_b5_bench_replay` 46 checks incl. weak replay.

### Bench v0.7 test plan (SDK, `bench_tools/buddy5/b5_profile_bench_v0_7.uf2`, no wheels)

Same setup as above. Record with `screen -L` as before; one log per case, e.g. `logs/2026-10-31_S5-C08_v07_<case>.txt`.

1. **Sensor no-target behaviour (most important):** point at open space (> 1.5 m clear), `p` ×10. Then `w` (250 ms) and `p` ×10. Read the class: `NO_RETURN` with a width (e.g. ~38 000 µs) means three-way classification works on this sensor; `FAULT Ffall` at 100 ms but `NO_RETURN` at 250 ms means the sensor's no-target pulse is longer than 100 ms → tell me the width (wait needs raising); `FAULT Frise` in open space means this sensor gives no pulse at all with no target → weak clearance can never happen with it.
2. **Disconnected:** unplug the ECHO lead (power off first), power on, `p` ×3 → expect `FAULT Frise`; `s` → expect side NONE, coverage 0 %.
3. **Injected stuck-high / no-rise:** `i` then `s` (stuck-high), `i` `i` then `s` (no-rise) → side NONE, coverage 0 %.
4. **Box centred at ~150 mm, open room** (`5`, `a`) ×3 → expect NONE with footprint FAIL (envelope at leg 1–2): the honest result of the current geometry.
5. **Box centred at ~300 mm** (place it, `s`) ×3 → expect a side with sweep PASS_ASSUMED; note whether WEAK is set (depends on step 1).
6. **Blocked side:** box centred at ~300 mm plus a second object ~400 mm to one side → that side's sweep FAIL; the other side chosen or NONE.

`python3 parse_b5_bench.py <logs>` summarises the rule-B fields too.

### RTOS diagnostic image (`bench_tools/buddy5/buddy5_rtos_ruleb_diag_v0_1.uf2`, no wheels: SYS_SIM_CAR 1, SYS_SIM_LINE 1, Wi-Fi off)

Rule-B driver with `B5_US_DIAG 1`: every ping prints one `[us diag]` line — class/reason, width, pre-trigger low wait, the SDK-style polled rise/fall times relative to the trigger, the interrupt-captured width, and the TRIG/ECHO pad + function registers (to see whether anything else claims GP16/GP17). Flash, open the console, press GP21 (reset and start), hold a hand/box at 100–300 mm, then open space. Compare the polled and interrupt widths and the rise latency (a real HC-SR04 rises ~0.3–0.5 ms after TRIG; the old fault rose after 4 µs). `buddy5_rtos_ruleb_sim_v0_1.uf2` is the same build without the per-ping prints.

### Status update (2026-10-31 21:10) — planner frozen

Decision (n1sh0): no further planner expansion. v0.7 is a **diagnostic build** only. Goal: a simple, repeatable bypass for the prescribed demo. No new exit angle, no automatic reverse-and-rescan. The ~215 mm standoff is a **model result** until the geometry is measured.

Sequence, replacing the six-step plan above:
1. **Open-space ping behaviour** (v0.7 `p` ×10 at 100 ms, `w`, `p` ×10 at 250 ms; a known flat face at 245 mm `p` ×5). Log `logs/<date>_S5-C08_v07_ping_open.txt`.
2. **Measure the real geometry:**
   - spin centre (drive-axle midpoint) to front-most point, and to rear-most point;
   - widest point including the wheels;
   - sensor face behind the front (servo centred);
   - sensor offset from the centreline;
   - servo axis to sensor face;
   - anything sticking out.

   Then re-run the existing model with the measured values. This is a config change only.
3. **One scan at ~240–250 mm** (box centred, open room, `s`, not auto). Log `logs/<date>_S5-C08_v07_scan_245.txt`.
4. **One supervised bypass using the existing navigation controller.** Which firmware carries Buddy 5 to the real motors is still to be decided: the RTOS tree only has a simulated car, and the `main` Buddy 5 extension is not implemented.

## Stop distance: 250 mm is the next candidate (2026-10-31 21:30)

Decision (n1sh0): screen 250 mm next instead of another distance comparison. The objective is reliable obstacle profiling, a safe bypass and line reacquisition, not telling every width apart (instructor guidance 2026-10-25: estimate span and clearance, plan with the maximum expected width, report the uncertainty, show consistent safe path selection). `B5_D_STOP_MM` stays 150 in code until the screening below passes; the screening scans are taken with `s` at a placed distance, so no rebuild is needed.

### What was measured (keep all of it, including the contradictions)

Spans from each log's primary `[b5 profile]` line (full table: `logs/2026-10-31_S5-C07_span_summary.txt`; the box labels come from the test notes):

| Distance | Box | Observed span, mm | Notes |
|---|---|---|---|
| ~250 mm | 100 mm | 163, 203, 204 | `case1_100mm_250` |
| ~250 mm | 100 mm | 386 | `case1_100mm_250_b`: edges at both arc limits (surrounding objects) |
| ~250 mm | 180 mm | 159, 176, 202 | `case2_180mm_250` |
| ~150 mm | 100 mm, centred | 98, 196, 100 | `v03_150_110` scans 1–3; scan 2 does not fit the others |
| ~150 mm | 180 mm | 84–144 | per the results notes above (reads short) |
| ~150 mm | offset scans | 47–80 | `v03_150_110` scans 7–11 |
| ~250 mm | offset scans | 0 (nothing found) to 251 | `v02_offset` |

**What the data supports:** with this sensor, servo and flat-faced boxes, width discrimination between 100 and 180 mm was **unreliable** at both 150 and 250 mm. At 250 mm both boxes read 159–204 mm; at 150 mm the 180 mm box read shorter than the 100 mm box in some scans.

**Modelling assumptions, not proven:**
- **Cone hypothesis:** a flat face returns an echo only within about ±20° of its normal. The observed span would then be roughly 2·d·tan 20° (about 109 mm at 150 mm, 182 mm at 250 mm) whatever the box. This explains several results; it does not explain all of them (e.g. 196 and 386 mm).
- **Lane-visibility estimate:** the share of the bypass lane inside the ±55° sweep, with 4° allowed for servo error. Lane = x 200–455 mm beside the box, from the face to 200 mm deeper. Calculated, not measured coverage, and not a safety probability; it depends on the assumed geometry:

  | Stop | 110 | 150 | 200 | 250 | 300 mm |
  |---|---|---|---|---|---|
  | lane inside the sweep (estimate) | 27 % | 43 % | 65 % | 82 % | 94 % |

- **327 mm is a provisional planning minimum** for the sideways move: the 200 mm maximum width + half the car + margin. It is not automatically a safe fixed displacement. It assumes the maximum obstacle width and the obstacle's position relative to the car; the surrounding clearance and the actual vehicle path still decide safety.
- **The ~215 mm standoff** (bypass geometry) is a model result pending the measured vehicle geometry.

### Sequence

1. **Open-space ping classification** (v0.7 `p` ×10 at 100 ms, `w`, `p` ×10 at 250 ms; flat face at 245 mm `p` ×5). Log `logs/<date>_S5-C08_v07_ping_open.txt`.
2. **Servo direction and centre (no angle placement needed).**
   - **Sign:** hold a hand or object roughly to the LEFT and press `s`; the closest angle must be negative. Repeat on the right; it must be positive.
   - **Centre offset:** face the car squarely at a flat wall about 250 mm away. To square it, push both front corners flush against the wall, then pull the car straight back. Press `s` twice. A flat wall echoes only near its normal, so the middle of the returned arc (`edges L/R`) shows where the sensor really points when the servo reports 0. An offset beyond about ±40 dd (4°) is worth correcting with `,` / `.`.
   - Log `logs/<date>_S5-C08_v07_servo_check.txt`.
   - The ±55° end angles (scale) are not checked here. The model keeps its 4° servo allowance, and the screening scans test the behaviour end to end. If even this is impractical, skip it and report the servo angle as unverified.
3. **Measure the vehicle geometry** (stationary testing firmware: `bench_tools/buddy5/b5_profile_bench_v0_7.uf2`, which never drives the wheels):
   - spin centre (drive-axle midpoint) to the front-most point and to the rear-most point;
   - widest point including the wheels;
   - sensor face behind the front;
   - servo shaft to sensor face;
   - sensor offset from the centreline.

   The footprint check uses the pivot, so v0.7 is rebuilt with the measured value (config only, new file name) before step 4.
4. **Screening at 250 mm** (sensor face to box face 250 mm, 180 mm box centred on the car's centreline). Blockers sit **inside the proposed bypass corridor**: centre about 330 mm to the side of the centreline and 300–350 mm ahead of the sensor, face at least 80 mm wide. Record each blocker's x and y measured from the sensor face.

   | Case | Setup | Repeats | Pass |
   |---|---|---|---|
   | S1 | sides open (nothing within ~1 m) | 3 | record the side and its clearance; ideally the same side each time |
   | S2 | left blocked | 3 | never LEFT |
   | S3 | right blocked | 3 | never RIGHT |
   | S4 | both blocked | 3 | NONE every time (car stays stopped) |

   Record for each scan the side, the clearance L/R, the sweep results L/R with any fail reason, and the measured and coverage percentages. Log `logs/<date>_S5-C08_v07_screen250_<case>.txt`. If S1 picks NONE, note whether the gap test or the footprint failed.

   **Report two results separately.** Never merge them into one "pass rate":
   - **Safe rejection** (S2, S3, S4): the blocked side was never chosen, and both-blocked stayed stopped.
   - **Successful open-side selection** (S1, plus the open side in S2/S3): a side was actually chosen when one was free. Record how often, and which side.
5. **If S1–S3 pass and S4 stays stopped:** one slow supervised bypass and line reacquisition, using the measured geometry. Then confirm with the 100 mm obstacle and repeat the successful runs.
   - **Target firmware (n1sh0, 21:36):** the teammate's working Buddy 1–4 micro T-Kernel integration. Reuse its existing drive, odometry, stop controls and analog line sensing, and integrate Buddy 5's tested profiling and navigation requests through that interface. No other motor implementation.
   - **If that integration is not ready:** finish the stationary evidence and record the powered bypass and reacquisition as **pending**. The port is on remote branch `codex/micro-tkernel-migration` (0f253db, not merged to `main` as of 2026-10-31 21:36).
6. **Compare 150 mm only if 250 mm exposes a specific problem.**

Background (empty-scene) scans may be recorded for later analysis. Background subtraction is **not** a dependency of the first bypass.

### Results — S5-C08 step 1, ping classification (2026-10-31 22:00–22:10, v0.7 SDK bench, USB power, no wheels)

| Log | Setup | Result |
|---|---|---|
| `logs/2026-10-31_S5-C08_v07_ping_open.txt` | "open" direction, 10 pings at 100 ms and 10 at 250 ms | 20/20 ECHO at 1050–1069 mm: the direction was not actually open (wall or furniture at ~1.06 m). No no-return exercised. |
| same | flat box face at 245 mm (tape), 5 pings | 5/5 ECHO, 249 mm (width 1453–1454 µs): about +4 mm, spread 0 |
| `logs/2026-10-31_S5-C08_v07_ping_noreturn.txt` | wall about 1 m away at roughly 45° (echo deflected away), 10 pings at 100 ms and 10 at 250 ms | 20/20 NO_RETURN, pulse width 58 994–59 003 µs, identical at both waits |

All 45 pings had ECHO low before TRIG (wait 0–3 µs), a rise 198–199 µs after TRIG, no noise pulses and no faults.

**Conclusions:**
- This HC-SR04 gives a completed ~59.0 ms pulse when nothing returns. That ends inside `B5_NORETURN_WAIT_MS` = 100 ms, so the three-way classification works on this sensor and no wait change is needed.
- The pre-v0.7 tools stopped waiting at 40 ms (SDK bench) or 50 ms (RTOS driver), before this pulse ended. That is why their "no target" results could not separate a no-return from a fault.
- **Caveat for weak clearance:** the no-return here came from a wall only ~1 m away, angled so the echo missed the sensor. A no-return therefore means "no echo heard", not "nothing there". This is why no-return clearance stays flagged as weak evidence and does not raise measured confidence.
- The healthy 198 µs rise latency contrasts with the RTOS fault's 4 µs rise: further evidence that fault is in the RTOS transaction, not the sensor.

### Results — S5-C08 step 2, servo direction and centre (2026-10-31 22:13–22:20)

- **First attempt** (`logs/2026-10-31_S5-C08_v07_servo_check.txt`, 3 scans): unusable. No near object was seen except one 314 mm ping. Likely causes: the object was below the beam, and/or it moved before the sweep reached its side. Kept as evidence.
- **Second attempt** (`logs/2026-10-31_S5-C08_v07_servo_check2.txt`, scans 4–7): object about 315 mm from the sensor. Side offsets were eyeballed, not measured. The fall wait was still 250 ms; classification is unaffected (no-return pulse 59 ms).

| Scan | Placement | Near-return arc (edges) | Arc midpoint | Closest | Result |
|---|---|---|---|---|---|
| 4 | front-left | −39.1° … −3.4° | −21.2° | 345 mm at −22.0° | sign correct |
| 5 | front-right | +22.7° … +35.1° | +28.9° | 374 mm at +22.7° | sign correct |
| 6 | centre | −10.3° … +27.5° | +8.6° | 332 mm at −8.2° | flat minimum −8° … +17° |
| 7 | centre | −15.8° … +26.2° | +5.2° | 335 mm at −5.5° | flat minimum −5.5° … +11° |

**Conclusions:**
- **Direction:** verified. Left reads negative and right reads positive.
- **Centre:** the centred object appears about **+3° to +9°** right of servo zero, depending on the estimator (arc midpoint +5° / +9°, middle of the flat minimum about +3° / +4°). The single closest angle is not a reliable centre for a wide beam. This cannot be split into servo offset and placement error: an eyeballed centre at 315 mm is uncertain by roughly ±5° (±28 mm sideways). The result is consistent with the earlier observation that the sweep reaches further right.
- **Status:** apparent offset ≤ ~9°, unresolved. Not trimmed. The model's 4° servo allowance may be optimistic.
- **Ranges:** the object read 332–345 mm against about 315 mm stated (measuring points not recorded). The flat box at 245 mm read 249 mm in step 1.

**Log note:** the raw `logs/2026-10-31_S5-C08_v07_servo_check2.txt` is 0 bytes. It was renamed while screen was still logging and then overwritten by a later `mv`. The scan lines analysed above are preserved verbatim in `logs/2026-10-31_S5-C08_v07_servo_check2_EXCERPT_reconstructed.txt`; the per-ping lines of scans 4–7 are lost. From now on, the log is renamed only after screen has exited.

**Third check** (`logs/2026-10-31_S5-C08_v07_servo_check3.txt`, 22:25–22:28; fall wait back to 100 ms):
- **Scan 8** (trim 1400 µs, nothing placed in front): wall at 1053–1089 mm only, no near object. As expected.
- **Trim returned to 1450 µs.** The user observed the sweep going further right; `.` would have turned it further right, and the echo data did not agree with the visual impression.
- **Scan 9** (1450 µs, bottle centred about 315 mm ahead on the tile junction):
  - near arc −21.3° … +19.3°, so arc midpoint **−1.0°**;
  - flat minimum 327 mm from −5.5° to 0° (middle about −2.8°);
  - this scan's side choice was RIGHT, with measured clearance 623/634 mm (gap test only, not a bypass test).

**Servo conclusion so far:**
- Direction verified.
- The centred-object estimate varies from **−1° to +9°** across three scans (scans 6, 7 and 9: arc midpoints +8.6, +5.2, −1.0). There is no consistent offset to trim. The spread is larger than the model's 4° servo allowance and includes placement and heading repeatability.
- The visual "sweeps further right" impression is recorded but not quantified; it may concern the end positions rather than the centre.
- **No trim applied.**

### Servo end-stop investigation (2026-10-31 22:30–22:48)

- **Frame contact** (bench keys, centre trim varied, observed by eye; screenshot only, no log):
  - At trim 1505 µs the ±55° ends (924 / 2085–2090 µs) both just touch the frame.
  - Relative to the 1450 µs centre, contact is at 526 µs to the right (≈ +49.8° on the linear map) and 637 µs to the left (≈ −60.4°).
  - At 1450 µs a +55° command (869 µs) therefore pushes about 5° into the frame. This is the "sweeps too far right" observation.
  - At 1505 µs the sensor visibly looks left, so straight ahead stays at about 1450 µs.
- **Right end at 970 µs** (trim 1550 µs while holding `]`): the right gap was a little larger than the left gap at 2030 µs.
- **Code check:** the angle-to-pulse map is linear and symmetric (+55° 869 µs, 0° 1450 µs, −55° 2031 µs) in both `drv_servo.c` and the bench.
- **Physical cause found by n1sh0:** a component on the right edge of the HC-SR04 board sits right at the edge (the left side's R16 does not), so the right side meets the frame first.
- **Tile symmetry test** (`logs/2026-10-31_S5-C08_v07_servo_tiles.txt`, 1450 µs). Car on the taped line; bottle (~75 mm diameter) at the left end, then the right end, of the taped cross arm (mirror positions, ~350–360 mm range):

  | Scan | Bottle | Near arc | Arc midpoint | Arc width | Closest |
  |---|---|---|---|---|---|
  | 1 | left | −35.7° … −11.6° | −23.7° | 24.1° | 359 mm |
  | 2 | right | +17.2° … +33.7° | +25.5° | 16.5° | 351 mm |

  - The mirrored arc midpoints agree within 1.8°: centre about +0.9°, and no large scale difference near ±24°.
  - The right arc is narrower (16.5° vs 24.1°). That could be the servo moving further per degree on the right, or an asymmetric beam or board. It is not resolved.
  - Taken together, the angle map is adequate within about ±2–4° around ±24°. The end-stop asymmetry is at least partly mechanical (the board component), so a whole-side right rescale (960 µs) is **not** supported by this data.

**Decision (n1sh0, 22:51): option 1, an asymmetric sweep from −55° to +44°.**
- `B5_SCAN_END_DD` 550 → 440 in `b5_config.h`, with the reason in the comment. The servo map is unchanged (centre 1450 µs).
- Pulses: +44° = 986 µs, about 61 µs (≈ 6°) short of the right contact at 924 µs. −55° = 2031 µs, about 56 µs (≈ 5°) short of the left contact at about 2087 µs.
- Known effect: with both sides open, the right lane is seen less completely, so selection may lean left. Unscanned space is never counted as clear.
- Host suite passes unchanged (`logs/2026-10-31_sweep44_host_suite.txt`).
- New bench `bench_tools/buddy5/b5_profile_bench_v0_7_1.uf2` (sha256 `cfad8637…fdd4`; source `b5_profile_bench_v0_7_1.c`): v0.7 plus this sweep. The `[` / `]` keys now hold at the actual sweep ends.
- To check after flashing: at `]` there must be a visible gap at the right; at `[` a gap at the left.

**Line offset noted (n1sh0):** while line following, the car runs slightly left of the line, because the left IR sensor is on the car centreline and the right IR sensor sits to its right. An obstacle on the line therefore sits slightly right of the car's centreline. The offset is to be measured with the geometry. Screening boxes are placed centred on the **line**, as in the demo, with the car where it sits on the line.

**v0.7.1 symmetry re-check** (`logs/2026-10-31_S5-C08_v071_servo_tiles.txt`, `..._servo_tiles2.tx` (the file name was truncated when saved), scans 4–15 in `..._v071_scans04-15_EXCERPT_reconstructed.txt` (raw log not saved); bottle at the cross-arm ends; the swing was visually correct per n1sh0):
- **Detected:**
  - left, −30.2° … −5.5°, midpoint −17.9°;
  - left (patchy), −27.5° … −11.0°;
  - right, +5.5° … +30.3°, midpoint **+17.9°** (357–369 mm).
  - The mirrored midpoints agree (−17.9° / +17.9°). They differ from the v0.7 test (±24°), consistent with a different bottle or car placement.
- **Not detected:** the bottle was missed entirely in 4 of 7 side scans (`found 0`; only the wall at ~1.02 m). A centred bottle was detected on every `p` ping (331–335 mm).
- **Reading:** a ~75 mm round bottle off to the side at ~350 mm is a marginal target for this sensor. Detection of small or curved side objects is intermittent; this is a limitation to report.
- **Consequence for screening:** blockers must present a **flat face turned toward the sensor** (perpendicular to the line from the sensor), because flat faces only echo near their normal. A blocker that is not detected would read as no-return, i.e. weak clearance. Record each blocker's position **and orientation**.

**Blocker detectability check** (`logs/2026-10-31_S5-C08_v071_blocker_check.txt`, 23:11–23:13; 100 mm box):
- **Box square to the car** (scans 12–15, left ×2 / right ×2, in the reconstructed excerpt): **0 of 4 detected**. Only the wall at ~1.02 m was seen.
- **Box straight ahead, `c` + `p`** ×9: 326–334 mm, every ping. The beam reaches the box, so this is not a beam-height problem.
- **Box angled toward the sensor** (scans 16–19): **4 of 4 detected**.

  | Scan | Side | Near arc | Midpoint | Range |
  |---|---|---|---|---|
  | 16 | left | −26.8° … 0° | −13.4° | 394–400 mm |
  | 17 | left | −33.0° … −2.7° | −17.9° | 385–409 mm |
  | 18 | right | +5.5° … +39.9° | +22.7° | 360–399 mm |
  | 19 | right | +6.2° … +41.3° | +23.8° | 360–399 mm |

  Placements were not measured. The left/right midpoint difference (~7°) is within placement uncertainty.
- **Conclusion:**
  - This confirms the 26 Oct finding: a flat side object echoes only when its face points roughly at the sensor; a face square to the car at ~20–45° off-axis returns nothing.
  - **Screening blockers must face the sensor.** The report must state this as a limitation: side obstacles angled away are invisible and read as no-return, which is weak clearance only, never measured clearance.

### Results — offset decision test, 150 vs 250 mm (`logs/2026-10-31_S5-C08_v071_offset.txt`, 23:20–23:27, v0.7.1, 180 mm box offset ~60 mm from the line, 3 repeats each; scans 5–16)

"Gap choice" = the clearance comparison alone (each side ≥ 255 mm; within 50 mm → preferred RIGHT; otherwise the larger side). "Final" = the v0.7.1 decision including the swept-footprint check.

| Stop | Box offset | Correct side | Scans | Clearance L/R (mm) | Gap choice | Final |
|---|---|---|---|---|---|---|
| ~250 (closest 266–282) | left | RIGHT | 5, 6, 7 | 376/310, 0/321, 543/350 | LEFT ✗, RIGHT ✓, LEFT ✗ (1/3) | RIGHT, RIGHT, RIGHT (3/3; left footprint failed on the envelope) |
| ~250 | right | LEFT | 8, 9, 10 | 361/346, 336/277, 601/344 | RIGHT ✗ (tie), LEFT ✓, LEFT ✓ (2/3) | NONE, NONE, LEFT (1/3, no wrong side) |
| ~150 (closest 175–186) | left | RIGHT | 11, 12, 13 | 0/323, 0/303, 30/311 | RIGHT ×3 (3/3) | NONE ×3 (footprint fails at 150, as modelled) |
| ~150 | right | LEFT | 14, 15, 16 | 515/245, 530/230, 399/243 | LEFT ×3 (3/3) | NONE ×3 (same) |

**Observations:**
- **At ~150 mm** the side evidence pointed the correct way in 6/6 scans, for both offsets.
- **At ~250 mm** it pointed the correct way in 3/6.
- **No final decision chose the wrong side** (0/12).
- **The asymmetric sweep biases 250 mm toward LEFT.** At 250 mm the left clearance was larger than the right in every scan whatever the offset (left 336–601 vs right 277–350). The right sweep ends at +44°, which caps how much right-side space can be measured. This is the known cost of the −55/+44 sweep, and it caused both wrong gap choices for the left-offset box.
- **Arc edges at 250 mm:**
  - Left offset: the near-return arc reached −28° to −30°, close to the box's true far edge (~−31°); its right end, +19° to +21°, is beam spread.
  - Right offset: the arc stopped at +16° to +19°; the true far edge at ~+31° was not seen. This is again weaker returns on the right at oblique angles; cause not resolved.
- **150 mm right-offset scans** read 327–354 mm at most angles, with only one 182–186 mm sample at +5.5°. The box placement relative to the beam is uncertain for those three.
- **n = 3 per cell.** Indicative, not proof.

**Decision update (23:30):** the stop distance stays at **~150 mm** (provisional), based on the offset test above. Recorded with evidence and open points in `STOP_DISTANCE_DECISION.md`; the earlier "250 mm next candidate" is superseded.
