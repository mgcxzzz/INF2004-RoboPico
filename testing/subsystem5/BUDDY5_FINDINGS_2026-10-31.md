# Buddy 5 — obstacle profiling findings, 31 Oct – 1 Nov 2026 (bench v0.7 → v0.7.4)

**Branch:** `buddy5-test`, from `buddy5` @ `1a0b559`.
**Scope:** stationary bench testing of the Buddy 5 obstacle scanner and planner on the Robo Pico car (servo GP12, HC-SR04+ TRIG GP16 / ECHO GP17). **No wheels were driven** during any test in this document.
**Companion documents:**
- `STOP_DISTANCE_DECISION.md`: the running decision record, with every intermediate result.
- `BENCH_PROFILE_TEST_2026-10-31.md`: test plans and the earlier S5-C07 results.
- `TEST_LOG.md`: one row per test.

---

## 1. Summary for the report

| Question | Answer, with evidence |
|---|---|
| Is the ultrasonic sensor working correctly? | **Yes.** Every ping has a 198–199 µs rise latency; range accuracy is +4 mm at 245 mm; a completed 59.0 ms "no-return" pulse when nothing echoes; 0 faults in several hundred pings. A spare HC-SR04+ gave the same results. |
| Can the sensor measure obstacle width? | **Only as an estimate.** At 150 mm a 100 mm box read 88–183 mm and a 185 mm box 204–308 mm (observed span), so the sizes were separable. At 250 mm they were not (159–204 mm for both). The planner therefore never trusts the width: it plans with max(observed span, **200 mm**) plus an allowance. |
| Does the robot pick the side the obstacle is shifted away from? | **Not reliably.** A flat-faced box shifted 40–60 mm looks almost the same as a centred one to this sensor (mirror-like reflection, ~±15–20° beam). When the two sides look equal, the planner takes the default side (RIGHT), which is wrong for a right-shifted box. Measured with the symmetric comparison at 150 mm (30 shifted-box scans, scene clear): 17 correct (4 of them only via the default), 7 stayed stopped, **6 wrong side**. The result depends on the box and its orientation. 110 mm: 6/12 correct, 6 wrong. 250 mm (earlier rules): 3/6. |
| Does it avoid a side that is actually blocked? | **Yes, when the blocker faces the sensor.** A box angled toward the sensor beside the path was detected 4/4. A box square to the car at 20–45° off-axis was detected 0/4, because a flat face only echoes near its normal. |
| Is a bypass from the stop point safe? | **Model only; no powered test yet.** Real-car simulation (measured geometry, 317 mm sideways move): from 150 mm the car fits on the correct side (21–63 mm spare), but **the wrong side makes contact** with a 185 mm box shifted 60 mm. From 250 mm even the wrong side clears by ≥ 48 mm. |
| Recommendation | Scan at ~150 mm (best data), then **back up to ~250 mm before the 45° turn-out**, so a wrong side guess is still safe. Optional: a second scan at 250 mm (the instructor's "scan from more than one position") to confirm the side. Needs a team decision and the powered test on the micro T-Kernel integration. |

---

## 2. Hardware facts established

| Item | Value | How |
|---|---|---|
| No-target behaviour | ECHO pulse of **58 991–59 015 µs**, then low | v0.7 `p` at a wall angled ~45° (`logs/2026-10-31_S5-C08_v07_ping_noreturn.txt`) |
| Range accuracy | flat box at 245 mm reads 249 mm (5/5 identical) | `..._v07_ping_open.txt` |
| Servo centre | **1450 µs** = straight ahead | bottle scans, tile mirror test; trimming to 1505 µs visibly pointed left |
| Servo direction | left = negative angle, right = positive | hand left/right; mirror positions read −17.9° / +17.9° |
| Mechanical limits | left about −60°; **right clear to +47.7°, contact at about +48.2°** | holding `]` and trimming in 5 µs steps; the contact is chip **U1** on the back of the sensor board against the battery mount (photo) |
| Car geometry (n1sh0, 2026-11-01) | front IR bar tip → drive-axle midpoint **130 mm**; axle → rear < 60 mm; widest **135 mm** (barcode IR to far wheel); sensor face **65 mm** behind the IR tip; sensor on the centreline; car rides < 10 mm left of the line; track 113 mm; sensor height 80 mm | tape measure |
| Box sizes | "180 mm" box is **185 mm** wide; 100 mm box | tape measure |

At a 150 mm stop (sensor face to obstacle) the IR bar tip is only **~85 mm** from the obstacle.

---

## 3. What was changed, and why

### 3.1 Rule B: three ping classes (v0.7)

**Why:** the old driver reported a still-high ECHO as "no target", so a stuck sensor looked like open space. An external review asked for measured echo, completed no-return and fault to be kept apart.

`b5_classify_ping()` (`app_program/subsystem5/obstacle_profile.c`) is shared by the RTOS driver, the SDK bench and the host tests:

```c
if (!b_pretrig_low)                       -> FAULT (stuck high; TRIG not sent)
else if (rise && fall && width >= MIN)    -> width <= 23 300 us ? ECHO(range) : NO_RETURN
else if (rise && !fall)                   -> FAULT (no fall within 100 ms)
else if (short pulses only)               -> FAULT (noise)
else                                      -> FAULT (no rise: silent / disconnected)
```

`b5_combine_pings()` turns the 3 pings at one angle into one sample:
- **any echo** → occupied at the **nearest** echo (stable only if all three agree within 20 mm);
- **echo + 1 fault** → blocks, and nearer space is weak evidence only;
- **echo + ≥ 2 faults** → blocks, otherwise unknown;
- **3 clean no-returns** → weak, flagged "clear" (never counted as measured);
- **anything else** → unknown.

`confidence_pct` counts measured echoes only. `coverage_pct` counts all usable samples and is the Planning gate (≥ 50 %).

**Result:** the sensor completes a 59 ms pulse, inside the 100 ms wait, so classification works on this hardware. The old 40/50 ms waits are why earlier logs could not tell a no-return from a fault.

### 3.2 Swept-footprint check of the real manoeuvre (v0.7)

**Why:** "327 mm + half car + margin" alone does not prove the whole manoeuvre is safe.

`b5_check_sweep()` replays the Navigation Coordinator's legs:
1. spin dir·45° on the spot;
2. drive the diagonal, lateral·√2;
3. spin back;
4. drive forward − lateral;
5. spin −dir·45°;
6. drive the rejoin leg, lateral·√2 + 100 mm search (cap 600).

At every 20 mm / 5° pose it tests the car rectangle grown by the 50 mm margin:
- **FAIL:** it overlaps the planning envelope, contains a measured return, reaches behind a side return, or crosses scanned-but-unknown space;
- **PASS_ASSUMED:** parts lie where the scan cannot see (behind the sensor, outside the sweep, behind the obstacle).

A side is chosen only if both the gap test and the footprint pass. At 150 mm this check **always** fails (worst-case 200 mm+ envelope plus margin), so the bench prints `side NONE` there. The side evidence in this document is therefore scored from the clearance comparison (3.4).

### 3.3 Sweep −55…+44° (v0.7.1), then symmetric ±46° (v0.7.4)

**Why:** at the old ±55° the right end pushed about 5° into the battery mount. Readings labelled +49.5/+55° were really taken at ~+50°, which overstates right-side room (the unsafe direction) and strains the servo.
- **v0.7.1** stopped at +44° (grid of 5.5°).
- **v0.7.4** uses a symmetric **±46°, 4.6° steps, 21 angles**, ~2° clear of the contact.
- A ±49.5° sweep was checked by eye and touches.

### 3.4 Side comparison over the same angles (v0.7.3)

**Why:** with the −55…+44° sweep the left side could always measure more room. As recorded, the comparison picked LEFT in **17 of 18** scans whatever the box position.

The left/right clearances used for the decision are now measured only over |angle| ≤ min(−start, end); the safety footprint still uses every sample (`B5_SIDE_COMPARE_SYMMETRIC 1`):

```c
sym_dd = -(int32_t)p_param->scan_min_dd;
if ((int32_t)p_param->scan_max_dd < sym_dd) { sym_dd = (int32_t)p_param->scan_max_dd; }
clear_l = nav_side_clearance(..., -1, sym_dd, &b_weak_l);
clear_r = nav_side_clearance(..., +1, sym_dd, &b_weak_r);
```

With the symmetric ±46° sweep (v0.7.4) the window simply equals the sweep.

### 3.5 Measured car geometry (v0.7.2)

`B5_PIVOT_TO_FRONT_MM` 150 (assumed) → **130**, `B5_SENSOR_BEHIND_FRONT_MM` 45 → **65**, `B5_CAR_WIDTH_MM` 155 → **135** (so `B5_LATERAL_MIN_MM` 327 → **317**). With this geometry the planner rejects more ~250 mm bypasses too.

---

## 4. The v0.7.4 decision logic, step by step

The SDK bench (`bench/b5_profile_bench/b5_profile_bench.c`) and the firmware use the same `obstacle_profile.c` and `b5_config.h`.

1. **Ping** (bench `ping()`): wait for ECHO low (≤ 100 ms), TRIG 12 µs, time the rise (≤ 30 ms) and fall (≤ 100 ms), then `b5_classify_ping()`.
2. **Angle sample:** 3 pings → `b5_combine_pings()`; servo settles 40 ms after each move.
3. **Coarse sweep:** −46° … +46° in 4.6° steps (21 angles).
4. **Edge refinement:** up to 12 extra angles bisecting where near (< ~550 mm) and far samples meet. These left/right moves after the sweep **are** used; the final return to centre takes no pings.
5. **Profile** (`b5_build_navigation_profile_ex()`):
   - near run → observed span;
   - stable-return span (stable samples only, reported separately);
   - planning envelope = max(span, 200 mm) + servo/range allowance;
   - coverage and measured percentages.
6. **Side clearance** within ±46°: free space beyond the envelope edge at the depth the car needs (closest + 200 mm assumed depth + car length). A side passes the gap test if ≥ **235 mm** (car 135 + 2 × 50).
7. **Footprint check** (3.2) for each side that passes the gap test.
8. **Decision:**
   - both pass and the difference is ≤ 50 mm → preferred side (RIGHT);
   - otherwise the larger side;
   - no side passes → `NONE` (stay stopped; in the firmware: Blocked → reverse → rescan).
   - Sideways move = envelope edge + 67 + 50, never < 317 mm.

**Key settings** (`b5_config.h`): `B5_D_STOP_MM 150`, `B5_NEAR_MM 550`, `B5_SCAN_START/END/STEP_DD −460/460/46`, `B5_SIDE_TIE_MM 50`, `B5_MARGIN_MM 50`, `B5_OBST_DEPTH_MM 200`, `B5_NORETURN_WAIT_MS 100`, `B5_PRETRIG_LOW_WAIT_MS 100`, `B5_SIDE_COMPARE_SYMMETRIC 1`.

---

## 5. Test results

### 5.1 Ping classification and sensor health — **worked**

- Open direction: 20/20 ECHO (~1.06 m wall).
- Angled wall: 20/20 NO_RETURN, 59.0 ms pulse, identical at 100 and 250 ms waits.
- Flat face at 245 mm: 249 mm.
- Rise latency 198–199 µs throughout (the broken RTOS image rose after 4 µs).

### 5.2 Servo — **worked after fixes**

- **Direction:** correct.
- **Centre 1450 µs:** the bottle centre estimates spread −1…+9° because of placement. A tile mirror test gave −23.7° / +25.5°; v0.7.1 gave −17.9° / +17.9°.
- **What did not work:**
  - trimming to the mechanically symmetric 1505 µs (it pointed left);
  - a right-side rescale to 960 µs (not supported by the mirror test; the asymmetry was the chip U1, not the servo).

### 5.3 Detecting side objects — **works only when they face the sensor**

100 mm box beside the path:
- square to the car: **0/4** detected;
- angled toward the sensor: **4/4** detected.

A ~75 mm round bottle at ±20–25° and ~350 mm was missed in 4 of 7 side scans (a marginal target).

### 5.4 Offset decision — **did not work reliably**

Side evidence = the clearance comparison (the footprint check blocks every 150 mm bypass, so it prints NONE). Offsets: 185 mm box 60 mm, 100 mm box 40 mm. "Tie" = difference ≤ 50 mm → RIGHT.

| Stop | Run | Box | Shift left → should be RIGHT | Shift right → should be LEFT |
|---|---|---|---|---|
| 250 mm | v0.6 rules, ±55° (10-31) | 180 | 1/3 | 2/3 |
| 150 mm | v0.6 rules, ±55° (10-31) | 180 | 3/3 | 3/3 (left echoes from other objects) |
| 150 mm | v0.7.1, −55/+44° | 185 | 1/3 | 3/3 (always LEFT: sweep bias) |
| 150 mm | v0.7.3, scene not clear | 185 | 0/3 | 1/3 |
| 150 mm | v0.7.3, clean | 185 | 2/3 + 1 stop | 3/3 |
| 150 mm | v0.7.3, spare sensor | 185 | 0/3 (3 stops) | 3/3 |
| 150 mm | v0.7.4, ±46° | 185 | 0/3 (3 stops) | 3/3 |
| 150 mm | v0.7.4, **box turned 180°** | 185 | 3/3 | **0/3 (tie → RIGHT)** |
| 150 mm | v0.7.4 | 100 | 3/3 (by default) | **0/3 (tie → RIGHT)** |
| 110 mm | v0.7.4 | 100 | 3/3 (by default) | 0/3 (tie → RIGHT) |
| 110 mm | v0.7.4 | 185 | 3/3 | 0/3 (tie → RIGHT) |

**Why it fails:**
1. **Physics.** A flat face square to the car echoes from the point straight ahead of the sensor whether the box is centred or shifted, as long as it still covers the centreline. The offset only shows in weak edge echoes, and at 150 mm the beam footprint (~±40–55 mm) is as big as the shift.
2. **Box shape.** In its first orientation the 185 mm box gave extra ~410–490 mm echoes (very likely its rear corners) on the side it was shifted toward. Those made the "right-shift" cases come out correct. Turning the box removed them, and the right shifts then failed.
3. **Logic.** The clearance comparison reduces the small real clue (the arc edge on the side away from the shift retracts, e.g. left edge −39° → −22…−25° for the 100 mm box shifted right at 110 mm) to a 20–32 mm difference. That is inside the 50 mm tie band, which exists because two open sides already differ by 10–45 mm by noise. Ambiguous cases then go to the default side (RIGHT).

**What did not help:** stop distance 110 / 200 / 250 mm, the symmetric ±46° sweep, or a different sensor.

**A possible improvement, not implemented:** decide from the arc edges directly. It would have got the 100 mm box right at 110 and 150 mm, but not the 185 mm box shifted right at 110 mm (its far edge, ~+54°, is outside the sweep). It needs proper testing, not tuning to 3-scan sets.

### 5.5 Width estimate — **estimate only**

| Stop | 100 mm box: observed (stable) | 185 mm box: observed (stable) |
|---|---|---|
| 110 mm | 135–183 (121–167) | 330–403 (111–398) |
| 150 mm | 88–183 (88–167) | 204–308 (148–247) |
| 250 mm | 163–204 | 159–202 |

Working hypothesis, not proven: a flat face echoes only within about ±20° of its normal, so the observed span ≈ 2·d·tan 20°. This explains most, not all, values.

### 5.6 Bypass footprint — **model only**

Real car against the real box (`logs/2026-11-01_real_footprint_sim.py`); minimum clearance with a 317 mm sideways move:

| Stop | 100 mm centred | 185 mm centred | 185 mm shifted 60: correct / wrong side | 100 mm shifted 40: wrong side |
|---|---|---|---|---|
| 150 mm | 49 | 21 | 63 / **0 (contact)** | 21 |
| 200 mm | 85 | 56 | 99 / 12 | 56 |
| 250 mm | 120 | 92 | 134 / **48** | 92 |

These assume an on-the-spot spin and exact distances; Buddy 4 measured distances 4–5 % short (~15 mm less room).

---

## 6. Things that went wrong in testing (kept for honesty)

- **Logs overwritten** (`screenlog.0` cleared or renamed while open). The scan lines analysed live are kept as `*_EXCERPT_reconstructed.txt`; their per-ping detail is lost.
- **An accidental scan 19** was removed from `2026-11-01_S5-C09_v071_offset150.txt` at the operator's request. The removed lines are in `..._REMOVED_accidental_tail.txt`.
- **The first v0.7.3 offset run** had a second object in view; it is reported but not used for conclusions.
- **"Never the wrong side (0/12)"** was claimed after the clean runs and later **withdrawn** by the box-turned test.
- **The RTOS 236 mm ultrasonic fault** is still open. A diagnostic image exists (`buddy5_rtos_ruleb_diag_v0_1`) but has not been run.

---

## 7. Open items / next steps

1. **Team decision:** scan at 150 mm, then back up to ~250 mm before turning out (optionally re-scan there), or stop at 250 mm.
2. **Integrate Buddy 5** into the teammate's micro T-Kernel Buddy 1–4 build (reuse its drive, odometry, stop and line sensing), then a slow supervised bypass and line reacquisition.
3. **Physical fix** for the U1 chip (a spacer behind the sensor board, or moving the battery mount) to get ±55° back.
4. **Measure the box depth.** Planning assumes ≤ 200 mm.
5. **If side choice is still wanted:** an edge-based rule, tested against fresh data.
6. **Run the RTOS diagnostic image** for the 236 mm fault.

---

## 8. Files in this branch

| Path | What |
|---|---|
| `app_program/subsystem5/obstacle_profile.[ch]` | classifier, combiner, profile, symmetric side comparison, swept-footprint check |
| `app_program/subsystem5/b5_config.h` | all settings above, with the reason for each change in comments |
| `app_program/subsystem5/drv_ultrasonic.[ch]` | RTOS driver with the pre-trigger check, three-way classification and `B5_US_DIAG` diagnostic prints |
| `app_program/subsystem5/task_obstacle.c` | RTOS task using `ping_once` / `b5_combine_pings`, scan dump with sample markers, coverage gate, sweep report |
| `testing/subsystem5/bench/b5_profile_bench/` | **SDK bench v0.7.4 source** + CMake (builds against this repo's `subsystem5`) |
| `testing/host/test_b5_ruleb.c` | rule-B, footprint and symmetric-comparison tests |
| `testing/host/test_b5_bench_replay.c`, `b5_bench_scans_2026-10-31.h` | replay of real 10-31 scans |
| `testing/host/test_b5_task.c` | statechart scenarios 1–13 (incl. stuck-high, unplugged, reverse-then-bypass) |
| `testing/subsystem5/logs/` | every bench log, host-suite logs, re-score outputs, footprint simulation |
| `testing/subsystem5/STOP_DISTANCE_DECISION.md` | full chronological decision record |

**Host tests:** `make -C testing/host` (all pass at this commit). Note that `make clean` cannot delete files on the shared drive.

**Bench builds** (UF2s are not committed; SHA-256 prefixes):

| Version | SHA-256 prefix |
|---|---|
| v0.7 | `806e1cc6e1ed659b` |
| v0.7.1 | `cfad86373aafa1ca` |
| v0.7.2 | `3e44c3239b463e68` |
| v0.7.3 | `2aab9207f4821175` |
| v0.7.4 | `fdd66af32f91e1d8` |

**Build the bench:** `cmake -S testing/subsystem5/bench/b5_profile_bench -B build_b5pb -G Ninja && ninja -C build_b5pb` (needs `PICO_SDK_PATH`).

**Bench keys:**

| Key | Action |
|---|---|
| `s` | scan |
| `p` | ping straight ahead |
| `c` | centre the servo |
| `[` / `]` | hold at the sweep ends |
| `,` / `.` | servo trim |
| `w` | 250 ms fall wait (diagnostic) |
| `i` | inject a fault |
| `a` | auto guard |
