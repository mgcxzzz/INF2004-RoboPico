# Buddy 5 — stop distance decision (provisional)

**Status:** provisional, 2026-10-31 23:30. Bench (stationary) evidence only; powered bypass not yet tested.
**Decision (n1sh0):** stop at **~150 mm** (sensor face to obstacle face) to scan and choose the bypass side. `B5_D_STOP_MM` is already 150 in `app_program/subsystem5/b5_config.h`.
**Supersedes:** "250 mm is the next candidate" in `BENCH_PROFILE_TEST_2026-10-31.md`. That proposal was dropped after the offset test below.

## Why 150 mm

### 1. Side choice (the demo decision): 150 mm was correct, 250 mm was not reliable

Offset test, 180 mm box ~60 mm off the line, 3 scans each (`logs/2026-10-31_S5-C08_v071_offset.txt`). Side evidence = the clearance comparison:

| Stop | Box shifted | Correct side | Side evidence correct |
|---|---|---|---|
| ~150 mm | left | RIGHT | **3/3** |
| ~150 mm | right | LEFT | **3/3** |
| ~250 mm | left | RIGHT | 1/3 |
| ~250 mm | right | LEFT | 2/3 |

- At 250 mm a flat face square to the car mostly echoes from straight ahead, so the offset barely shows.
- The −55°/+44° sweep also caps how much right-side room can be measured, so the left always looked bigger at 250 mm.
- Earlier 150 mm runs (`logs/2026-10-31_S5-C07_v03_150_110.txt`): a left shift was detected 3/3, a right shift 0/3, under an earlier rule and a ±55° sweep. Today's 150 mm right shift was 3/3.

### 2. Width estimate at 150 mm: an estimate only, reported with its uncertainty

Observed span (sensor estimate of the obstacle's width):

| Obstacle | At ~150 mm | Source |
|---|---|---|
| 100 mm box, centred | 98, 196, 100 mm | `S5-C07_v03_150_110`, scans 1–3 |
| 180 mm box, centred | 84–144 mm (reads short) | 2026-10-31 results notes |
| 180 mm box, offset left | 368, 398, 206 mm (side objects in view) | `S5-C08_v071_offset`, scans 11–13 |
| 180 mm box, offset right | 165, 197, 163 mm | `S5-C08_v071_offset`, scans 14–16 |
| (250 mm, for comparison) 100 / 180 mm centred | 163–204 / 159–202 mm | `S5-C07_case1`, `case2` |

- 150 mm is closer to the true width for a 100 mm box (2 of 3 within a few mm), but not reliable for 100–180 mm in general. It can read short, which is the unsafe direction, or long when side objects are in view.
- Neither distance tells 100 mm from 180 mm reliably. Working hypothesis, not proven: a flat face echoes only near straight-on (a cone of about ±20°).
- **The bypass therefore does not depend on the width estimate.** Planning uses `max(observed span, 200 mm)` plus the model allowance (the maximum expected width, as the instructor allows), and the sideways move is never less than 327 mm (200 + car/2 + 50 margin, a provisional minimum).
- **Report:** the estimate and its spread as above, not a single accurate width.

### 3. Bypass room from 150 mm: model says tight, physical test pending

- The planner's footprint model (pivot 150 mm behind the car front, **assumed**; 50 mm margin; 45° spin) says the spin/diagonal comes within the margin of the ≥200 mm planning envelope when stopped at ~130–170 mm. The v0.7.1 bench therefore prints NONE at 150 mm.
- n1sh0's judgement: 150 mm looks like enough physical space. **To be settled by measuring the car geometry and a slow supervised bypass**, not by the model alone.
- **If the physical test shows 150 mm is too tight:** the fallback is scan at 150 mm, then back off before the bypass. That needs a navigation change and team agreement, and it is not implemented.

## Known limitations at this point

- **Sweep −55°/+44°:** a component at the right edge of the sensor board meets the frame at about +50°. Servo angles are symmetric within ~2° (mirror test ±18°); centre 1450 µs.
- **Right-side echoes at wide angles are weaker** (far right edge not seen at 250 mm, small round objects missed). Cause not resolved.
- **Side objects only echo when they face the sensor** (box square to the car at 20–45° off-axis: 0/4 detected; angled toward the sensor: 4/4). An object angled away is unknown or weak clearance, never measured clearance.
- **No-return pulse:** a 59.0 ms completed pulse when nothing comes back, so echo, no-return and fault are told apart. A no-return means "no echo heard", not "nothing there".
- **Sample size:** 3 scans per case. Indicative, to be repeated.

## Next

1. Measure the car:
   - spin centre to front and to rear;
   - widest point;
   - sensor face to front;
   - servo shaft to sensor face;
   - sensor offset from the centreline;
   - car offset from the line.
2. Re-run the footprint model with the measured values (config only; bench rebuild under a new version).
3. Blocked-side screening at 150 mm, with blockers facing the sensor: left blocked, right blocked, both blocked, 3 each. Report "safe rejection" separately from "correct open-side choice".
4. A slow supervised powered bypass from 150 mm on the team's micro T-Kernel integration once Buddy 5 is integrated there; otherwise record as pending.
5. Repeat the offset scans for a larger sample, and with the 100 mm box.

## Update 2026-11-01 01:30 — measured car geometry

Measured by n1sh0. The front-most point is the IR bar tip, which sticks out 20 mm past the bumper.

| | Value |
|---|---|
| A, front IR bar tip → drive-axle midpoint (spin centre) | 130 mm |
| B, spin centre → rear | < 60 mm (length taken as 190) |
| C, widest, barcode IR (left) → far wheel | 135 mm (S5-C05 had 155) |
| D, sensor face → front IR bar tip | 65 mm (was 45 to the bumper) |
| E, servo shaft → sensor face | not measured (the model assumes the sensor turns about its face) |
| F, sensor offset from centreline | 0 |
| G, car offset from the line | < 10 mm, to the left |
| H, drive-wheel track (centre to centre) | 113 mm |
| I, sensor height | 80 mm |

**Consequences:**
- At a 150 mm stop (sensor to face), **the IR bar tip is only ~85 mm from the obstacle**. The spin centre sits 65 mm behind the sensor.
- In `b5_config.h`: `B5_PIVOT_TO_FRONT_MM` 150 → 130, `B5_SENSOR_BEHIND_FRONT_MM` 45 → 65, `B5_CAR_WIDTH_MM` 155 → 135 (so `B5_LATERAL_MIN_MM` 327 → 317).
- Host suite passes (`logs/2026-11-01_geometry_host_suite.txt`). Two navigation tests and three replay expectations changed: with this geometry the planner rejects more ~250 mm bypasses too.
- New bench `bench_tools/buddy5/b5_profile_bench_v0_7_2.uf2` (sha256 `3e44c323…eed6`).

**Two model views of a bypass from 150 mm:**

1. **The planner's conservative check** (envelope = max(span, 200 mm) + allowance, plus the 50 mm margin all round) **still rejects** every 150 mm scan from the offset test. It also rejects most ~250 mm scans: the grown car's inner side on the 45° diagonal meets the envelope corner.
2. **Real car against the real box** (`logs/2026-11-01_real_footprint_sim.py` / `.txt`). Measured rectangle, no margin; box 100 mm deep; on-the-spot 45° spin; 317 mm sideways move; exact replay of the coordinator legs. Minimum clearance:

   | Stop | 100 mm box centred | 180 mm box centred | 180 mm box offset 60 mm: correct side / wrong side |
   |---|---|---|---|
   | 150 mm | 49 mm | 21 mm | 63 mm / **0 (contact)** |
   | 200 mm | 85 mm | 56 mm | 99 mm / 14 mm |
   | 250 mm | 120 mm | 92 mm | 134 mm / 49 mm |

**Reading:**
- From 150 mm, the bypass physically fits for centred 100–180 mm boxes, but with only 21–49 mm to spare.
- For an offset box it fits only on the correct side. 150 mm is also where the side choice was correct 6/6.
- The real margins assume the spin is on the spot and the distances are driven exactly. Buddy 4 measured −4 to −5 % distance shortfall, which on the 317 mm sideways move is ~15 mm less room.
- **To run a physical bypass from 150 mm, the planner's conservative footprint check would have to be relaxed or made advisory for that test** (team decision).

## Update 2026-11-01 02:45 — offset test at 150 mm (S5-C09) and the side-comparison fix

**Test** (`logs/2026-11-01_S5-C09_v071_offset150.txt`; it ran on bench v0.7.1). Setup:
- 185 mm and 100 mm boxes, face 150 mm from the sensor (checked with `p`: 147–151 mm).
- Centred, shifted car-left and shifted car-right (185 mm box: 60 mm; 100 mm box: 40 mm), 3 scans each.
- An accidental 19th scan start was removed at n1sh0's request; the removed lines are kept in `..._REMOVED_accidental_tail.txt`.

**As recorded, the side comparison was biased.** It picked LEFT in 17 of 18 scans whatever the box position: the −55°/+44° sweep lets the left side measure more room (left clearance 364–481 mm against right 162–372 mm).

**Fix (`B5_SIDE_COMPARE_SYMMETRIC 1`, `obstacle_profile.c`):** the left/right clearances used for the comparison are measured only over |angle| ≤ 44°, the same window on both sides. The swept-footprint safety check still uses every sample; a host test checks that an object seen only at about −49° still blocks the left. Host suite passes (`logs/2026-11-01_symmetric_compare_host_suite.txt`). New bench `b5_profile_bench_v0_7_3.uf2` (sha256 `2aab9207…de43`).

**Recorded scans re-scored with the implemented fix** (`logs/2026-11-01_S5-C09_offset_rescore_implemented.txt`; side evidence = clearance comparison):

| Stop | Box | Shift | Correct | Evidence |
|---|---|---|---|---|
| 150 | 185 mm | 60 mm left | RIGHT | RIGHT ×3 |
| 150 | 185 mm | 60 mm right | LEFT | LEFT ×3 |
| 150 (10-31) | 180 mm | 60 mm left / right | RIGHT / LEFT | RIGHT ×3 / LEFT ×3 |
| 150 | 100 mm | 40 mm left / right | RIGHT / LEFT | tie → RIGHT ×3 / tie → RIGHT ×3 (difference only 10–32 mm, inside the 50 mm tie band) |
| 150 | 185 mm | centred | tie | **NONE ×3** (both clearances 168–213 mm, below the 235 mm gap test) |
| 150 | 100 mm | centred | tie | tie → RIGHT ×3 |
| 250 (10-31) | 180 mm | 60 mm left / right | RIGHT / LEFT | RIGHT ×3 (one via tie) / LEFT ×1 + tie → RIGHT ×2 |

**Reading:**
- At 150 mm, the 180–185 mm box shifted 60 mm is now classified correctly 12/12; at 250 mm, 4/6.
- A 100 mm box shifted 40 mm gives the right direction but too small a difference: it defaults to RIGHT. The tie band was **not** tuned to this data.
- **Cost of the ±44° window:** at 150 mm the sensor sees too little of the bypass lane within ±44° to certify 235 mm for a *centred* 185 mm box, so it stays stopped. That would then go Blocked → reverse → rescan.
- **Restoring a symmetric ±55° sweep** (physical clearance for the sensor-board component) would remove both the bias and this coverage loss. It is the preferred fix if feasible.
- **Width at 150 mm** (observed span; stable-return span):
  - 100 mm box: 88–144 mm; 88–123 mm.
  - 185 mm box: 204–308 mm; 148–247 mm.
  - The two sizes did not overlap. It is an estimate with roughly +20…+60 % spread for the observed span.

## Update 2026-11-01 03:10 — v0.7.3 live repeat: second echoes at ~410–490 mm

- **Runs:**
  - `logs/2026-11-01_S5-C09_v073_offset150.txt`: as labelled, 1/6 correct.
  - `logs/2026-11-01_S5-C09_v073_offset150_clean.txt` (nothing else within ~60 cm): also inconsistent.
- **In both runs, extra echoes at ~410–490 mm** appear at about ±22–30°, beside the box's own arc (~145–205 mm). Their side changes with the box placement, and the clearance comparison follows them, not the box offset.
- **An empty scan without the box is clean** (nothing nearer than ~1 m; `screenlog` of 03:04–03:05, to be saved as `..._v073_emptyscan.txt`). So the echoes need the box present.
- **Working hypothesis, not proven:** the box's rear corners or depth, heard at wide servo angles because of the wide beam. The box depth is still to be measured. A deep box would also exceed `B5_OBST_DEPTH_MM` (200).
- **The earlier v0.7.1 run at 150 mm also had ~405–455 mm echoes,** so its 12/12 re-score is not yet trusted.
- **Next session:** a shallow obstacle (thin face, small depth), 150 mm, centred / left / right ×3.
- **Sensor health checks so far all pass:**
  - rise latency 198–199 µs on every ping;
  - range +4 mm at 245 mm;
  - no-return pulse 59.0 ms;
  - zero faults in several hundred pings;
  - clean empty scan.

**Correction and spare-sensor run (03:15).**
- The clean run was mis-summarised above as "also inconsistent". Re-checked against the gap test (each side ≥ 235 mm, 50 mm tie band):
  - centred: NONE, RIGHT, NONE;
  - car-left: RIGHT, RIGHT, NONE;
  - car-right: LEFT ×3.
- **Spare HC-SR04** (n1sh0, same setup; to be saved as `logs/2026-11-01_S5-C09_v073_offset150_sparesensor.txt`):
  - centred: RIGHT ×3 (clearance 138–143 / 284–286);
  - car-left: NONE ×3 (0/211–214);
  - car-right: LEFT ×3 (259–263 / 100–127).
  - The same echo patterns as the first sensor (box arc plus ~410–437 mm returns), so **sensor fault ruled out** for this behaviour.

| v0.7.3, 150 mm, 185 mm box shifted 60 mm | Correct side | Stayed stopped | Wrong side |
|---|---|---|---|
| clean run (sensor 1) | 5/6 | 1/6 | 0/6 |
| spare sensor | 3/6 | 3/6 | 0/6 |
| **clean runs total** | **8/12** | **4/12** | **0/12** |
| first v0.7.3 run (scene not clear) | 1/6 | 0/6 | 5/6 (2 of them via tie → RIGHT) |

**Reading:**
- With a clear scene, the symmetric comparison never chose the wrong side (0/12). Stopping (4/12) is the safe outcome, from limited coverage within ±44° and the extra corner echoes.
- **The extra ~410–490 mm echoes appear on the side the box is shifted toward.** This is consistent with the box's far rear corner (box depth to be measured).
- The empty-scan lines are kept in `logs/2026-11-01_S5-C09_v073_emptyscan_EXCERPT_reconstructed.txt`.

**±50° clearance check (03:16).** Holding `]` (+44°, 986 µs) and trimming right in 5 µs steps:
- clear at trim 1410 µs (≈ 946 µs, **≈ +47.7°**);
- the next step (≈ +48.2°) touches.
- The contact point (photo): the receiver-side edge of the sensor board against the white frame standoff.

A ±49.5° sweep is therefore not possible without a physical change, and the sweep stays −55°…+44° with the symmetric comparison (v0.7.3).

**v0.7.4 (03:30):** symmetric sweep ±46° in 4.6° steps (21 angles), `B5_SCAN_*` in `b5_config.h`.
- About 2° clear of the right contact (+48.2°); the left loses −46…−55°.
- The comparison window becomes ±46°; at a 150 mm stop that is roughly +30 mm of measurable room per side (estimate).
- Host suite passes (`logs/2026-11-01_sweep46_host_suite.txt`). Three `test_b5_ruleb` checks were made grid-independent; the "object only beyond −44°" check is skipped while the sweep is symmetric.
- Bench `b5_profile_bench_v0_7_4.uf2`.
- Planned check: 185 mm box shifted 60 mm left ×3 / right ×3 at 150 mm, compared with v0.7.3.

## Update 2026-11-01 03:35 — v0.7.4 runs and the box-turned test: correction

- **v0.7.4, box as before** (`logs/2026-11-01_S5-C09_v074_offset150.txt`):
  - centred: RIGHT ×3;
  - car-left: stayed stopped ×3 (173–179 / 165–190);
  - car-right: LEFT ×3.
- **Same box turned 180°** (log to be saved as `logs/2026-11-01_S5-C09_v074_offset150_boxturned.txt`): the extra ~400 mm echoes are **gone**, and the box arc is wide on both sides (−34.5…+34.5°).
  - car-left: RIGHT, RIGHT, tie → RIGHT (3/3 correct);
  - **car-right: tie → RIGHT ×3** (304/310, 298/314, 301/311). **That is the wrong side, 3/3.**
- **Correction:**
  - The earlier "never the wrong side (0/12)" depended on extra echoes from that box in its first orientation, very likely its rear corners or depth.
  - With the turned box, a flat face shifted 60 mm is **not distinguishable** (left/right clearances within 6–16 mm). The tie rule then takes RIGHT, which is wrong for a right-shifted box.
  - **Side choice for an offset flat-faced obstacle is not reliable at 150 mm with this sensor and logic.**
- **Safety consequence (real-car simulation):** a wrong side from 150 mm makes contact with a box shifted 60 mm, because the 45° diagonal clips its near corner; from 250 mm the wrong side still clears by ~49 mm. The 317 mm sideways move itself is sufficient (box ≤ 200 mm). The problem is the turn-out distance.
- **Open decision (team):** make the bypass safe whichever side is picked:
  - (a) scan at ~150 mm, then back off to ~250 mm before the 45° exit; or
  - (b) stop at ~250 mm.
  - Either needs the powered test.

**100 mm box at 150 mm, v0.7.4** (03:36–03:38; log to be saved as `logs/2026-11-01_S5-C09_v074_offset150_100box.txt`; order centred / car-left / car-right ×3, offset as instructed, 40 mm):
- centred: 291–296 / 302–307 → tie → RIGHT;
- car-left: 295–299 / 307–311 → tie → RIGHT (correct only by default; no measurable shift);
- car-right: 318–319 / 289–291 → difference 28–30 mm in the correct direction, but inside the 50 mm tie band → **RIGHT, wrong side ×3**.
- Observed span 142–179 mm (true 100 mm).
- **Real-car wrong-side clearance** (simulation, 317 mm move), 100 mm box shifted 40 mm right: 21 mm from 150, 56 mm from 200, 92 mm from 250. For the 185 mm box shifted 60 mm: 0 / 12 / 48 mm.

**110 mm stop, v0.7.4** (03:44–03:48; log to be saved as `logs/2026-11-01_S5-C09_v074_offset110.txt`; closest 112–121 mm):

| Box | Placement | Clearance L/R | Choice | Arc edges L / R |
|---|---|---|---|---|
| 100 mm | centred | 270–278 / 282–288 | tie → RIGHT | −38.5…−39.1° / +34.5° |
| 100 mm | car-left | 268–273 / 287–292 | tie → RIGHT (✓ by default) | −37.3° / +29.9…+31.1° |
| 100 mm | car-right | 293–300 / 268–273 | tie → RIGHT ✗ | −22.4…−25.3° / +32.8…+34.5° |
| 185 mm | centred | 60–83 / 286–293 | RIGHT | −35.0…−37.3° / +24.8…+25.3° |
| 185 mm | car-left | 0 / 284–285 | RIGHT ✓ | −46° (arc limit) / +25.3° |
| 185 mm | car-right | 275–278 / 292–297 | tie → RIGHT ✗ | −34.5° / +25.3° |

- **Offset decision at 110 mm:** 6/12 correct. Every right-shifted case defaulted to RIGHT.
- **The arc edges carry a directional clue** for the 100 mm box: the edge on the side away from the shift retracts (left edge −39° → −22…−25° for a right shift; right edge +34.5° → +30…+31° for a left shift). The clearance comparison reduces this to 20–32 mm, inside the 50 mm tie band.
- For the 185 mm box shifted right, the far right edge (~+54°) is outside the ±46° sweep and not seen; the left shift reached the −46° limit.
- **Width at 110 mm is not better:** 100 mm box observed 135–183 mm; 185 mm box 330–403 mm.
- **Conclusion:** a shorter stop does not solve the side choice. The ambiguity resolves to the default side, so right-shifted obstacles are misjudged.
