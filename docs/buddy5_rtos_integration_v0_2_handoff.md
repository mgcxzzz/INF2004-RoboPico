# Handoff — Buddy 5 RTOS integration candidate v0.2 (APP=rtos_buddy1_5)

19 November 2026 (Asia/Singapore). Addresses `rtos_candidates/buddy5_rtos_integration_v0_1/INTEGRATION_REVIEW_2026-11-19.md`.

**Built and host-checked only.** Nothing was flashed, driven, committed, pushed or merged.

## What and where

- **Candidate folder:** `Embedded/rtos_candidates/buddy5_rtos_integration_v0_2/`. Paths below are relative to it.
  - The v0.1 folder, the frozen bench folders and the existing worktrees were not modified.
- **Firmware:** `firmware/robopico_buddy1_5_rtos_integration_v0_2_candidate.uf2`
  - SHA256 `01dda836a2621b21a89cfd73ab20e0e734d009ea625a3ba5c1955e0750c368a0`
  - ELF and map are alongside. `SHA256SUMS.txt` covers every file in the folder except `source/`.
- **Base:** `barr-c-cleanup` @ `1862e0d2ce0a5ed26687edc605a749140e1d3a4e` (v0.2.9).
- **Patch:** `patch/buddy5-rtos-integration-v0_2.patch` (73 files) against that commit. It contains all of v0.1 plus the v0.2 changes.
  - It was applied to a fresh worktree of 1862e0d and rebuilt: the UF2 is identical.
  - `source/` is that patched tree file for file (no `.git`, `firmware/migration` or old artifacts).
- **To make a real worktree:** same commands as v0.1, applying the v0.2 patch instead:
  ```
  git -C INF2004-RoboPico worktree add ../INF2004-RoboPico-buddy5-rtos-integration -b buddy5-rtos-integration 1862e0d2ce0a5ed26687edc605a749140e1d3a4e
  cd ../INF2004-RoboPico-buddy5-rtos-integration && git apply <candidate>/patch/buddy5-rtos-integration-v0_2.patch
  ```
- **Build** (from `build_make/`):
  ```
  make -j4 APP=rtos_buddy1_5 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0
  ```
  - Toolchain: xPack arm-none-eabi-gcc 15.2.1.
  - pico-sdk `98a542c1…`.
- **Buddy 1–4 image unchanged.**
  - A clean `APP=rtos_buddy1_4` build gives `d03284f9…c3`, the same value as before v0.1 and after it.
  - The shared coordinator additions are compiled only for rtos_buddy1_5.
- **Unchanged from v0.1** (the extensions folder is identical to the v0.1 snapshot): the width estimator, width probe, direction rules (edge_probe, 25 mm near allowance), bypass and every ported module.
- **Read next:**
  - `source/docs/buddy5_rtos_integration.md`
  - `source/app_program/obstacle_navigation/README.md`

## Review item 1 — failed width probes hold; a clean skip is separate

The mission now decides a **width policy** before it touches the probe. It uses a fresh stationary triplet taken after the stop: three clean echoes within 15 mm, two attempts.

| Situation | Policy (E5 byte 19) | What happens | Displayed width |
|---|---|---|---|
| Clean triplet inside 270–330 mm | probe attempted | frozen width v0.2 probe runs unchanged | measured or estimated on COMPLETE (`measured`, 1) |
| Attempted probe ends non-COMPLETE, **including REFERENCE** | `failed` (3) | **HOLD `WIDTH_ABORT`**, module reason = end code; nothing moves until Stop | the probe's own D3: 150 mm `PROJECT_RANGE_ASSUMPTION`, labelled ASSUMED |
| Clean triplet outside 270–330 mm | `skipped` (2) | the probe is not started and nothing moves for it; the mission continues to the approach | 150 mm assumption (basis 3, status INVALID), labelled "deliberately skipped" |
| Faulty or inconsistent triplet | `none` (0) | HOLD `RANGE` — never treated as a skip | none |

- **END_REFERENCE alone is never read as clean evidence.** A REF rejected by the probe's own rule after the policy accepted the range (for example a fault at REF) is a failure, and it holds.
- E5 bytes 57–58 carry the range that decided the policy.

## Review item 2 — bounded line steering during the 300→200 mm approach

- **Ownership:** Buddy 5 keeps its single lease. Nothing is handed back to Buddy 3, even briefly.
- **Coordinator extension `nav_steer_drive(owner, lease, degrees)`.** It is compiled only with `MOTION_STEERED_DRIVE`, which is set only for rtos_buddy1_5.
  - It travels through the nav codec as kind `NAV_STEER`.
  - It is accepted only from the lease holder, and only while its own straight drive is executing.
  - It moves the heading setpoint of the executor's existing heading hold, within ±8°.
  - The setpoint lapses to straight 100 ms after the last call and is cleared on every new move.
  - Goal, timeout, settle and the 12° heading fault are unchanged.
- **Adapter in the mission.** At each fresh observation during an approach leg, the mission runs Buddy 3's own `analog_demand()` edge law on the levels the line provider already normalised.
  - The law is the left sensor held on the strip's right edge at 500, with a ±60 deadband and the right-sensor guard.
  - Its wheel differential, scaled from 45 to 20 mm/s (same turn per distance), is integrated into the setpoint.
  - Both-black and the follower's stop decisions add no turn. While the line reads white, the follower's seek-left differential is kept.
- **Unchanged:** the sonar stop (clean echo ≤ 205 mm, then a fresh range, at most 3 legs), the 40 mm line-loss hold, every other hold and Stop/E-stop.
  - If the coordinator refuses the setpoint for any reason other than end-of-drive, the mission stops and holds `MOVE`.
- **E5 status v2** (59 bytes) adds the width policy, the deciding range, the last setpoint, the largest setpoint, and the number of frames at the ±8° bound.
  - The dashboard and the offline decoder still read the 54-byte v1 layout.

## Checks run (software only; logs in `checks/`)

- **Review item 1:**
  - **REF rejected (one fault at REF), then clean sonar:** HOLD `WIDTH_ABORT` for 30 s simulated, robot stopped. D3 shows 150 mm ASSUMED, policy `failed`, no approach or direction.
  - **Clean 180 mm stop:** policy `skipped`, no D1–D4 frames, mission completes the bypass with no contact.
  - **Faulted or noisy policy range:** HOLD `RANGE`, policy `none`.
- **Review item 2:**
  - **Three pushes after an approach leg starts** (odometry cannot see them): +3°/−2 mm, −1°, +1.5°/+1 mm.

    | Push | Edge error at the direction probe, steered | Same, steering off | Heading, steered | Heading, steering off |
    |---|---|---|---|---|
    | +3°/−2 mm | 0.6 mm | 24.9 mm | 1.44° | 6.0° |
    | −1° | 0.5 mm | 7.7 mm | 0.19° | 2.0° |
    | +1.5°/+1 mm | 0.1 mm | 16.1 mm | 0.01° | 4.5° |

  - The sonar still stops at 203 mm. The setpoint stays ≤ 8° (it saturated in the +3° case). No heading fault.
  - **Coordinator steering test:**
    - Codec round trip from −8.0 to +8.0°; requests above 8° are refused.
    - Refused in AUTO, in PAUSED, during a turn, after the drive ends, from another owner, with a stale lease, and for NaN.
    - A 6° setpoint gives 5.8° of odometric yaw. After the setpoint lapses, yaw returns to 0.2°.
    - A new drive starts straight. Stop during a steered drive stops the car.
- **Bounded behaviour:**
  - Line lost during the approach still holds `NO_LINE_APPROACH`, with the setpoint bounded.
  - Stop in each of eight stages: stopped, session ended, no hand-back.
- **Unchanged parts:**
  - Width: frozen width v0.2 suites 337 + 200 pass. Frozen decoder parity on the four RTOS-simulated runs is OK. The saved v0.2 captures give four OK and the 117 mm MISMATCH, which is retained.
  - Direction: frozen edge_probe suite 1298 + 1298. The near-25 replay changes no decisions. The 117 mm missing-record audit is unchanged.
  - Buddy 1–4: the rtos_buddy1_4 UF2 is byte-identical.
- **Build and static:**
  - 0 compiler warnings.
  - Coding-standard mechanical and clang-format checks PASS (142 files); the modified coordinator files are format-clean.
  - RAM: 57,216 B left (v0.1: 57,288 B).
  - Motion-task stack chain still about 3.0–3.2 KB of 4 KB. The steering path adds about 0.5 KB to the shallow approach branch only.
- **Dashboard:**
  - `test_b5_rtos.js`: 8/8 pass.
  - New headless panel smoke `smoke_b5_panel.js`: PASS.
  - Existing suites: the same 4 failures as the unmodified base (external fixture paths).
- **Re-run everything:** `sh testing/rtos_migration/buddy5/run_buddy5_checks.sh <bench_tools/buddy5_sdk>`.
- **A fix made during this work:** the first v0.2 build changed the Buddy 1–4 image. A rewritten but equivalent `#else` condition in `motion_api.c` compiled differently. The original text was restored, the per-object disassembly was compared, and the image was rebuilt byte-identical.

## Assumptions and unresolved issues

- **No hardware evidence for v0.2 or v0.1.**
- **Steering is not tuned on hardware.** The law and gain are Buddy 3's, scaled by speed.
  - The host plant is ideal: wheels follow commands, and the analog line model uses a 10 mm spot.
  - On the car, line noise, the ADC rate and turning at 20 mm/s may behave differently.
  - The approach still assumes a straight line segment at the obstacle.
- **Ported trade-offs:**
  - **Ignored stop signals.** The follower's both-black and white stop timers are ignored during the approach; the mission's own 40 mm line-loss hold applies instead.
  - **Seek-left while white.** While the line reads white, the setpoint seeks left (up to 8°), exactly as Buddy 3 would. That can add up to about 3 mm of lateral motion before the 40 mm hold.
- **Skip below 270 mm can start the direction probe closer than 200 mm.** For example, in the simulation the robot stopped at 180 mm and the probe started there. The approach has no reverse leg; this behaviour is the same as v0.1.
- **Changed shared files.** The coordinator sources (`motion_api`, `nav_codec`, `nav_coordinator`, `motion_executor`, `motion_api.h`, `nav_api.h`) now contain `#ifdef MOTION_STEERED_DRIVE` blocks.
  - The rtos_buddy1_4 binary does not change.
  - Text fingerprints of those files in historical `package_release.py` manifests will differ.
- **Remaining v0.1 limits still apply:** stop distance not measured, box size/clearance assumptions, width bias, 25 mm near allowance only centred on hardware, stack/heap estimates, and release toolchain differences.

## Minimal physical test sequence (one at a time; you flash and drive)

Before every run: keep a hand on Stop, use GP20 as the E-stop, and run the passive capture.

1. **Static check (as in v0.1).**
   - The panel shows idle, the start-ready flag, and servo 0°.
   - "Width policy" reads `none`; "Approach line steering" reads 0.0°.
   - Moving a hand in front changes the front range.
   - Buddy 1–4 status is normal.
2. **Guard with no obstacle.**
   - Expect normal line following and no trigger. Stop leaves the car cancelled with the motors off.
3. **Left bypass.** A 185 mm box straddling the line, offset 40 mm right, face at least 60 cm ahead on a straight segment, clear floor on the left.
   - Expected sequence: width probe at about 290 mm, alignment, approach, direction LEFT (evidence), bypass, hand-back.
   - Expected values: policy `measured`, gate 270–330 mm. Steering largest value a few degrees, and the car stays on the line to about 200 mm.
   - Note: the width and its basis, the steering largest value and bound count, and any contact, stall or HOLD.
4. **Right bypass** (the mirror of test 3).
5. **Only if 1–4 are clean, one REF-failure check.** Run test 3 with a hand briefly blocking the sonar while the width probe starts.
   - Expect HOLD "width probe aborted", policy `failed`, and 150 mm ASSUMED displayed.
   - The car must stay still until Stop.
6. **Send back:**
   - captures (`*_mqtt.csv`);
   - a panel screenshot for each run;
   - notes, including any HOLD reason.
