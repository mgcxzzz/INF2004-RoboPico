# Subsystem 4: IMU-Based Motion and Terrain Monitoring (Buddy 4)

Task **IMU Terrain** (Week 6 Section 3: periodic 20 ms, priority Medium).
Student ID 2503753. GY-511 (LSM303 accelerometer + magnetometer, **no
gyroscope**) on I2C1: SDA GP2, SCL GP3 (Robo Pico Grove 2 / Maker port).

It detects humps, estimates each hump's peak height and keeps the highest
of the run, reports motion events, and reports shocks within 50 ms. It
never commands motion and is the only user of I2C1.

Testing: `testing/subsystem4/HOW_TO_TEST.md`. Host tests:
`make -C testing/subsystem4/host`.

## 1. Files

| File | What |
|---|---|
| `imu_iface.h` | **The one shared header** (Tribunal 1, R3): `imu_event_t`, motion classes, event types, reject reasons, fault codes, the IMU telemetry frame (source 4) and the functions other buddies provide or use |
| `b4_config.h` | every pin, period, threshold; `TUNE` values are set with the bench tests |
| `imu_proc.c/.h` | statechart 6.4 and all processing, **pure C** (no kernel): calibration, filters, pitch, hump profile, shock, motion class, data health. Host-tested |
| `task_imu.c` | the RTOS task: 20 ms cyclic handler, reads the sensor and Buddy 2's estimate, runs one statechart step, reports to Safety, publishes telemetry |
| `drv_lsm303.c/.h` | I2C1 driven directly through the RP2040 registers, polled with a 2 ms timeout on every wait; LSM303DLHC and LSM303AGR recognised |
| `b4_pack.c` | telemetry pack / unpack (masks and shifts, clamped), names |
| `b4_stubs.c` | weak defaults for what other buddies provide (motion estimate, MQTT publish, event hook) |
| `b4_branch_glue.c` | only so the `buddy4` branch builds on its own (see section 8); compiles to nothing once merged |
| `subsystem4.c/.h` | `subsystem4_init/start`; keeps the two template IF-03 objects that subsystem 3's template consumer still uses |
| `subsystem4_test.c` | on-target tests (`make TEST=1`) |

## 2. Requirements and where they are met

| Requirement | How | Evidence |
|---|---|---|
| FR-IMU-01 detect humps | nose-up pitch > 4.0 deg for 60 ms ("sustained change") starts Hump Processing; `IMU_EVT_HUMP_START` | host R-03, U-01..U-12; bench `h`; car test |
| FR-IMU-02 peak height, highest of the run | front-contact height rebuilt from pitch and Buddy 2's distance (section 4); `IMU_EVT_HUMP_PEAK`; run maximum reset on a new run (not on resume), published in `IMU_EVT_RUN_SUMMARY` on Completed | host U-*, R-08, R-09 |
| FR-IMU-03 motion events | 8 classes in 3 bits: stationary, steady, accelerating, turning (from Buddy 2), climbing, descending, impact (from the IMU), unknown; 100 ms hold against flicker; `IMU_EVT_MOTION` | host M-01..M-12 |
| NFR-IMU-01 20 ms, no missed frames | cyclic handler (no drift); the sensor's 400 Hz FIFO is emptied each period, so a late period loses no data; late periods and FIFO overruns counted and published | S4-U02, bench `f` (S4-U07), host F-07 |
| NFR-IMU-02 height error | **clarified** (Tribunal 1): error <= max(5 mm, 10 % of the reference); the bounds meet at 50 mm | host U-12: 200 random humps 10-80 mm, 100-300 mm/s, noise up to 20 mg, all inside, worst 4 mm. Real humps: S4-U09/U10 |
| NFR-IMU-03 shock within 50 ms | every FIFO sample (2.5 ms apart) is checked, impact BEFORE terrain; Safety is told in the same period; the detect time is measured and published | host R-04/R-05, S4-U05, bench `k` (S4-U08) |

## 3. Statechart 6.4 as implemented

```
Initialising --sensor answers--> Calibrating --2.0 s still, |g| ok--> Idle
     |                               |  deadline 10 s / bad baseline
     +--no answer--+                 +-----------------+
                   v                                   v
            Fault / Recovery <--- I2C fails 3x, no data 3x, data frozen 1 s
                   |              (from any sampling state)
                   +--authorised reset (Nav back to Ready)--> Initialising

Sampling (every 20 ms, impact checked first):
  Idle --start / resume--> Motion Tracking --nose up 60 ms--> Hump Processing
  Motion Tracking / Hump --stop, pause, e-stop, complete--> Idle
  Hump --level a wheelbase + 30 mm after the descent--> publish peak / reject --> Tracking
  Hump --8 s or 1000 mm--> reject (timeout or no descent) --> Tracking
  any sampling state --shock [new]--> Impact / Shock (partial hump rejected)
  Impact --below 250 mg for 200 ms--> Tracking (mission) or Idle
```

- **Mission events** come from the Navigation Coordinator through
  `imu_notify_state()` (already called by `nav_coordinator.c`), queued in a
  message buffer so a quick Reset + Start is never lost. Running after
  Ready = new run; Running after Paused = resume; Completed = run summary;
  Ready = an authorised reset.
- **Calibration** needs a 2.0 s window in which every axis stays within
  40 mg peak-to-peak and Buddy 2 (if present) reports no motion. It checks
  |g| (850..1150 mg) and that gravity is on the configured UP axis (>= 90 %):
  a wrong mounting setting is a calibration fault, not a silent error. It
  is always redone after a fault (never reuses a baseline).
- **Fault / Recovery** sets `SYS_FAULT_IMU` in the shared fault mask,
  reports to Safety (`SAFETY_SRC_IMU`) and marks the data invalid. It never
  recovers by itself. The bit is cleared only after a reset AND a working
  sensor. Critical or warning is the team's choice: `B4_FAULT_CRITICAL`
  (default 0 = warning, because the IMU never moves the car).
- **Shocks** are always warnings (`IMU_WARN_SHOCK`): Safety decides.

## 4. Hump height (FR-IMU-02, NFR-IMU-02)

The GY-511 has no gyroscope (Week 6 correction of the AI's GY-521 design),
and Tribunal 1 flagged `h = v^2 / 2g` as drifting. So the height comes from
geometry, using distance rather than time:

- **Pitch** is the angle of the (forward, up) acceleration vector against
  the same vector at rest, with Buddy 2's longitudinal acceleration taken
  out first. Using the vector's direction, not its size, cancels the
  vertical jolts over the hump's edges.
- The rear contact follows the front one a **wheelbase L** behind, so the
  front contact's height is `h(s) = h(s - L) + L sin(pitch)`, rebuilt every
  4 mm of Buddy 2's distance `s` in a 64-entry ring (256 mm of history).
- The ground the car stands on is height 0 (after calibration, and again
  every time it has been level for a wheelbase in Motion Tracking).
- **Peak** = highest 5-bin (20 mm) average, so noise does not bias it up.
- **Validation before publishing** (a false peak is never kept): odometry
  fresh all through; 5..120 mm; the height back to within max(5 mm, 30 %)
  of 0 once the car is past (mean over the last wheelbase); no shock since
  the ground reference was last set. Otherwise `IMU_EVT_HUMP_REJECTED` with
  the reason (`imu_reject_t`).
- Without Buddy 2's distance the hump is still **detected** (FR-IMU-01)
  but its height is rejected as "no odometry", never guessed.
  `B4_BENCH_ASSUME_MM_S` can assume a speed for bench pushes only.

`B4_WHEELBASE_MM` (drive axle to caster contact) must be measured (S4-C05).

## 5. Tribunal 1 / Week 8 decisions and feedback, and what the code does

| Point (IS28_ImmTri1) | Done here |
|---|---|
| Minimum bit width for every IMU field | `imu_event_t` is 40 bits of data (3+3+4+8+11+1+1 + 16-bit time); the telemetry payload is 16 bytes with each field sized to its range; values clamped before packing (R4/R5); host P-01..P-08 test min, max, negative, clamping and wrapping |
| Buddy 4 single source of `imu_event_t`, all six states; climbing / descending / impact from the IMU, stationary / accelerating / turning from Buddy 2's estimate, packed into the same motion field (IF-17/IF-18 in the Tribunal) | `classify()` in `imu_proc.c`; Buddy 2's estimate read through `imu_motion_get()` |
| R3: one shared header | `imu_iface.h` |
| Freshness: reuse the existing rule of the shared motion estimate | `B4_MOTION_FRESH_MS` = `NAV_LINE_STALE_MS` (50 ms, IF-19), enforced with `#error` in `task_imu.c`; older or missing -> IMU-only fallback (stationary / unknown; turning never guessed) |
| h = v^2/2g drifts; validate against a reference | not used; pitch x distance with a closing check (section 4); reference tests S4-U09/U10 |
| Clarify NFR-IMU-02 | max(5 mm, 10 %), threshold 50 mm (`b4_nfr02_ok()`) |
| Magnetometer heading only a stretch goal | encoder heading is primary; the driver reads the magnetometer, bench tool `m` measures motor interference first; not used in any decision |
| Measure I2C-read-to-publish latency against 50 ms | the detect-to-notify time of every shock is measured and published (`shock_ms`, 6 bits); bench `k` prints it |
| A4 / feedback: log 50 Hz stationary, straight, turning, hump; missed samples; height vs reference; impact <= 50 ms | bench tool `r`, `f`, `h`, `k`; on-target S4-U02; `HOW_TO_TEST.md` |
| Integration: IMU event time within +-100 ms of Motion Control's position log | every event carries `t_ms` from the shared 1 us clock (`sys_time_us()`), the team's timestamp base |

## 6. Interfaces (to add to TEAM.md section 5 when the branches are merged)

| ID | From -> To | Mechanism | Name | Data |
|---|---|---|---|---|
| IF-20 | Nav -> S4 | function call -> S4 message buffer | `imu_notify_state()` | operating state (already in `nav_iface.h`) |
| IF-21 | S2 -> S4 | function call | `imu_motion_get()` | net distance, heading (int16, wrap) + age; until S2 provides it, the weak default reads `b5_motion_get_estimate()` (IF-16) |
| IF-22 | S4 -> S1 | function call | `imu_telemetry_publish()` | 19-byte frame, source 4, layout in `imu_iface.h` |
| IF-23 | S4 -> Safety | `safety_report_fault(SAFETY_SRC_IMU, ...)`, `safety_kick()`, `sys_fault_set(SYS_FAULT_IMU)` | fault / shock codes in `imu_iface.h`; health check every 200 ms |
| IF-24 | S4 -> anyone | function call | `imu_event_notify()` | `imu_event_t` as it happens (optional) |

Task and objects (TEAM.md sections 4 and 6): `imu_task` priority 18, any
core; 1 task, 1 cyclic handler, 1 message buffer (+ the 2 template IF-03
objects until the template consumer goes).

## 7. Tuning (b4_config.h)

| Value | Start | Set by |
|---|---|---|
| `B4_AXIS_FWD/UP`, `B4_SIGN_*` | X fwd, Z up | S4-C01 (bench `o` prints the lines to paste) |
| `B4_WHEELBASE_MM` | 120 | S4-C05 (measure, then the block test) |
| `B4_CAL_STABLE_MG` | 40 | S4-C02 |
| `B4_TILT_ON_DD` / `OFF` | 4.0 / 2.0 deg | S4-C03 noise with motors on; smallest hump seen is about L x sin(4 deg) = 8 mm |
| `B4_SHOCK_MG` | 1200 | S4-C04: above driving jolts, below a bump into a wall |
| `B4_ACCEL_MM_S2`, `B4_TURN_DD_S` | 200, 20 deg/s | car runs with Buddy 2 |
| `B4_LP_SHIFT` | 0 | only if the motors' vibration needs it (adds lag = height error) |

## 8. The buddy4 branch and merging

`buddy4` was made from `main` so it carries no Buddy 3 or Buddy 5 code:

- `app_program/system/` is an **unchanged copy** of the buddy5 branch at
  1a0b559 (Navigation Coordinator, Safety Supervisor, HAL, fault mask,
  telemetry schema), **without `sys_bench.c`** (it needs Buddy 3 and 5).
- `subsystem5/b5_iface.h`, `b5_pack.h`, `obstacle_profile.h`, `b5_stubs.c`
  are unchanged copies too (the coordinator includes them).
- `app.h` and `app_main.c` have exactly the buddy5 changes
  (`system_init/start`).
- `b4_branch_glue.c` stands in for `sys_bench.c` (GP20 / GP21 buttons,
  console status) and for Buddy 5's `b5_post_event()` / `b5_get_state()`.
  Each part is compiled only while the real file is absent
  (`__has_include`), so after a merge it compiles to nothing.
- TEAM.md: only the GP2 / GP3 rows are claimed here. The subsystem 4 row,
  the task row and the interface rows above go in at merge time (those
  table rows were changed on buddy5 too and would conflict).

Checked: merging `buddy4` into a branch holding buddy5 at 1a0b559 gives no
conflict, and the merged tree builds (normal and `TEST=1`) with no
"multiple definition". If buddy5 changes a file in `system/` before the
merge, git will report that file as a conflict: take buddy5's version
(buddy4 never edits those files). Merge buddy5 (which contains buddy3)
first, then buddy4.

Not done yet (needs the real car or the team): the wheelbase and mounting
values, the shock threshold, Buddy 2's real `imu_motion_get()`, and the
IMU telemetry section moving into `system/sys_tlm.h`'s "Sources 1, 2 and 4"
slot (the layout stays as it is in `imu_iface.h`).
