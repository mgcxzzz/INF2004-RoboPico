# Subsystem 5 — Adaptive Ultrasonic Scanning and Obstacle Profiling

Group IS28 · Buddy 5 · RTOS task **Obstacle Scan and Recovery** · priority
band 20–21 (TEAM.md)

An HC-SR04+ on an SG90 servo watches the road ahead. When something is in
the way, the subsystem:

- profiles it;
- plans a way round it;
- finds the line again afterwards.

The task owns the ultrasonic sensor and the servo **and nothing else**. It
never drives the motors: every movement is a request to the Navigation
Coordinator in [`../system/`](../system/README.md), as in the Section 4
task interaction diagram.

---

## Files

| File | What it is |
|---|---|
| `subsystem5.c` / `.h` | template entry points: `subsystem5_init()` (drivers, interrupt, event flag) and `subsystem5_start()` (the task) |
| `task_obstacle.c` | the task: statechart 6.5, one function per state |
| `obstacle_profile.c` / `.h` | obstacle width, beam-spread correction, side choice (pure C, also tested on a PC) |
| `drv_ultrasonic.c` / `.h` | HC-SR04+ driver: TRIG GP16, ECHO GP17 timed by the shared GPIO interrupt |
| `drv_servo.c` / `.h` | SG90 driver: GP12, 50 Hz PWM (slice 6A) |
| `b5_config.h` | pins, timing and every tuning value |
| `b5_iface.h` | events in, requests out: the contract with the other subsystems |
| `b5_pack.h` / `.c` | minimum-bit representation: field widths, clamping, masks and shifts, the echo result word, and packing of the binary telemetry frame (its layout is in the shared schema `../system/sys_tlm.h`) |
| `b5_stubs.c` | weak placeholders for the functions other subsystems provide |
| `subsystem5_test.c` | on-target tests (`make TEST=1`) |

The 1 µs clock and the GPIO interrupt dispatcher are shared, so they live
in `../system/sys_hal.c`.

---

## Hardware

| Signal | Pico W pin | Robo Pico connector | Notes |
|---|---|---|---|
| US_TRIG | GP16 | Grove 4 | 12 µs pulse |
| US_ECHO | GP17 | Grove 4 | both-edge interrupt; 3.3 V because the HC-SR04+ runs from 3V3 (A5) |
| SERVO_PWM | GP12 | Servo port 1 | PWM slice 6A, 50 Hz; servo + is VIN, never 3V3 (A8) |

Do not connect a 5 V HC-SR04: its echo is 5 V and would damage GP17 (A5).

Angles in the code are signed tenths of a degree, with the same sign rule
as Motion Control's heading: **−900 = left, 0 = straight ahead, +900 =
right** (Tribunal 1 decision; it was 0 = right … 1800 = left). If your
servo is mounted the other way round, set
`B5_SERVO_REVERSED` to 1 in `b5_config.h`.

---

## How the code follows statechart 6.5

| State | Function | Entry / do | Leaves on |
|---|---|---|---|
| Idle | `st_idle` | servo 90°, ranging off | mission start → Guard |
| Guard | `st_guard` | park at 90°, ping every 60 ms; ask Nav to slow below 500 mm | range < D_stop → Confirm |
| Confirm | `st_confirm` | count consecutive readings < D_stop | 3 in a row → Halting; reading ≥ D_stop → Guard |
| Halting | `st_halting` | request stop, swing servo to −55° (left) straight away | stop accepted → Coarse Sweep |
| Coarse Sweep | `st_coarse_sweep` | −55° to +55° in 27.5° steps (5 angles), median of 3 pings each. 55° is the car's mechanical limit: at 60° the sensor hits the casing | near points → Edge Refinement; none → Planning |
| Edge Refinement | `st_edge_refine` | bisect each near/far edge (self-transition) | bracket closed → Planning |
| Planning | `st_planning` + `b5_build_profile` (obstacle_profile.c) | profile, beam-spread width correction, score both sides | nothing / confidence < 50 % → Guard; a side passable → Bypassing; none → Blocked |
| Bypassing | `st_bypassing` | record pose, send plan, keep pinging ahead | new obstacle confirmed → Halting; passed → Recovering |
| Recovering | `st_recovering` | steer back to the stored line, poll line sensor | line found → Guard; 3 failed legs → Fault (LINE_LOST) |
| Blocked | `st_blocked` | publish blocked route | reverse safe and < 2 tries → reverse → Halting; tries used up → Fault |
| Fault | `st_fault` | release motion, report fault, stop ranging | fault reset → Idle |

---

## Interfaces with the other buddies

Everything Buddy 5 exchanges with the rest of the car is in
[`b5_iface.h`](b5_iface.h). `b5_stubs.c` gives **weak** default
versions, so this subsystem builds and runs on its own. When another buddy
writes a normal function with the same name, it replaces the stub
automatically at link time. Nobody has to edit Buddy 5's files.

The stubs fail safe. Without a real Navigation Coordinator, a stop request
is never accepted, so Halting times out into Fault rather than driving on.

### Events into Buddy 5

Call `b5_post_event(mask)` from any task or interrupt handler.

| Event | Sent by | Meaning |
|---|---|---|
| `B5_EV_MISSION_START` | Navigation Coordinator | system entered Running |
| `B5_EV_MISSION_STOP` | Navigation Coordinator | stop, pause, safe stop: go to Idle |
| `B5_EV_STOP_ACCEPTED` | Navigation Coordinator | reply to `b5_nav_request_stop` once the car is stopping |
| `B5_EV_PLAN_ACCEPTED` / `_REJECTED` | Navigation Coordinator | reply to `b5_nav_send_plan` |
| `B5_EV_MOTION_DONE` / `_FAILED` | Navigation Coordinator | the requested bypass, rejoin leg or reverse has finished |
| `B5_EV_FAULT_RESET` | Safety Supervisor | fault acknowledged, mission reset |

### Functions other buddies provide

The Navigation Coordinator functions (`b5_nav_*`) are already written, in
[`system/nav_b5.c`](../system/nav_b5.c). `b5_report_fault()` is written
in [`system/safety_glue.c`](../system/safety_glue.c), which also
decides which Buddy 5 faults stop the car.

| Function | Owner | Contract |
|---|---|---|
| `b5_nav_request_slow(bool)` | Navigation Coordinator | FR-USO-02 slow-down request |
| `b5_nav_request_stop()` | Navigation Coordinator | stop; answer with `STOP_ACCEPTED` |
| `b5_nav_send_plan(plan)` | Navigation Coordinator | drive round the obstacle on `plan.side`: shift `lateral_mm`, travel `forward_mm`; answer with `MOTION_DONE` |
| `b5_nav_request_rejoin(req)` | Navigation Coordinator | steer back to the line (`lateral_mm` > 0 means the car is right of it), sweeping `search_mm`; answer with `MOTION_DONE` |
| `b5_nav_request_reverse(mm)` | Navigation Coordinator | back up; answer with `MOTION_DONE` |
| `b5_nav_blocked_route(profile)` | Navigation Coordinator | FR-USO-07 blocked-route report |
| `b5_nav_release_motion()` | Navigation Coordinator | Buddy 5 no longer needs motion; line following may resume |
| `b5_motion_get_estimate(est)` | Motion Control (S2) | cumulative `distance_mm` and `heading_dd` (FR-MOT-03); **heading positive = turned right** |
| `b5_line_detected()` | Line and Barcode (S3) | true while either front sensor sees the line |
| `b5_telemetry_publish(frame, len)` | WiFi and MQTT (S1) | queue the 19-byte binary frame (team schema `../system/sys_tlm.h`) for MQTT; do not block |
| `b5_report_fault(code)` | Safety Supervisor | Buddy 5 entered Fault |

---

## Build, flash and try it

Use the template's normal build from `build_make/` (see the top-level
README):

```sh
cd build_make
make -j8
```

Copy the `.uf2` file to the Pico. The console is on the USB cable.

The shared bench helpers are switched on in `../system/sys_config.h`:

- a simulated car follows a line;
- a run starts by itself after 2 s;
- the Robo Pico buttons work: **GP20** = emergency stop, **GP21** = reset
  and start.

With only the sensor and the servo connected, hold something closer than
300 mm in front of the sensor. The console shows:

- Guard → Confirm → Halting;
- the sweep;
- the measured profile and the chosen side;
- the coordinator driving the bypass;
- Recovering, then back to Guard.

## Tests

| Where | How | What |
|---|---|---|
| `testing/host/` | `make -C testing/host` on a PC | packing (sizes, clamping, sign extension, 16-bit wrap, telemetry frame); width accuracy over the NFR-USO-02 grid; side choice; the statechart in 4 scenarios against a simulated car, including the shared fault-mask bit and decoding every telemetry frame |
| `subsystem5_test.c` | `make TEST=1`, flash, read the console | S5-U01 … S5-U06: maths, a recorded 100 mm obstacle, task alive, sensor answering, 60 ms ping spacing; S5-U13 … S5-U15: telemetry frame round trip, scan sample in one word with clamping, 16-bit wrap |
| `testing/subsystem5/TEST_LOG.md` | by hand | the manual tests (servo direction, bypass on the bench, stop timing) and every result |

The host tests use an ideal beam. They check the logic, not the sensor.
Assumptions A5–A8 still need the bench measurements.

---

## Minimum-bit data (Tribunal 1 requirement change)

The requirement: every application value uses only the bits its valid
range needs, faults go into a shared bitmask, MQTT payloads are compact
binary, and packed data shared with another task or an interrupt is
accessed safely. What changed in Buddy 5:

| Data | Before | Now |
|---|---|---|
| State, fault, side, counters | `enum` / `uint8_t` / `uint32_t` | bit-fields: 4, 3, 2 and 2 bits |
| Servo / edge angles | `int16_t`, 0 … 1800 | 11-bit signed, −900 … +900, + = right |
| Ranges, widths, plan distances | `int32_t` | 12 bits (0 … 4095 mm); bypass plan 20 → 8 bytes, rejoin request 16 → 8 bytes |
| Scan sample | 12 bytes | one 32-bit word (angle 11 + range 12 + valid 1) |
| Obstacle profile | 56 bytes | 28 bytes |
| Status record | 84 bytes | 40 bytes |
| Ping counters | 32-bit | 16-bit, wrapping (differences only) |
| Motion estimate from Buddy 2 | `int32_t` × 2 | `int16_t` × 2, signed, net distance, allowed to wrap |
| Echo result from the interrupt | width + a phase variable written by both the task and the ISR | one packed 32-bit word: width 16 + ping sequence 8 + flags, written only by the ISR |
| Faults | own variable | also one bit in the shared fault mask (`system/sys_fault.h`) |
| Telemetry | the status struct | a 19-byte little-endian binary frame in the team schema: 3-byte common header (version, source 5, length, sequence) + 16-byte payload |

**How it stays safe**

- **Bit-fields only for data that one task owns** (scan, context, status).
  A bit-field store is a read-modify-write of the whole word, which is only
  safe when nothing else writes that word.
- **Other tasks never touch the live record.** `b5_get_status()` copies a
  snapshot that is replaced under a mutex (`TA_INHERIT`), and
  `b5_get_state()` is one aligned word load.
- **The interrupt has its own words.** The task writes only the ping
  sequence number; the ISR writes only the result and "rise seen" words.
  The result is packed and stored with one 32-bit write (atomic on the
  M0+), then a barrier, then the event flag. An echo tagged with an old
  sequence number is ignored, which also removes the old two-writer
  `phase` variable.
- **Masks and shifts, not bit-fields, wherever the layout matters**: the
  echo word (`b5_pack.h`) and the MQTT frame (the team schema,
  `../system/sys_tlm.h`), so the PC decodes exactly what the Pico packed,
  whatever the compiler.
- **Clamp before packing.** 4396 mm in 12 bits would wrap to 300 mm, the
  stop distance; every value is clamped first, so it reads 4095 instead.
- **Wrap only where it is harmless.** Buddy 2's 16-bit distance and heading
  may wrap during a long run. Buddy 5 only uses the change over one bypass
  (about 1–2 m), computed with `b5_wrap_diff16()`, which stays correct
  across a wrap because 2 m is far below half the ±32 m range.

**What stayed full width, and why**

- Raw µs timestamps and `last_pub_us`: the hardware timer is 32 bits.
- Q15 trigonometry and intermediate maths: computation precision, not
  stored data; each result is clamped when it is stored.

**Telemetry frame for Buddy 1** (19 bytes, little-endian; defined in the
team's one shared schema header `../system/sys_tlm.h`, source 5): the
3-byte common header every subsystem uses (schema version, source, payload
length, sequence number), then state, fault, side, retries, servo angle,
obstacle found, guard range, closest range, confidence, width, sideways
shift, ping and no-echo counters. Decode with `b5_tlm_unpack()`; the bench
console already does.

**Cost**: a few extra shift and mask instructions per access (the M0+ has
no bit-field instructions), well under 1 % CPU. Static RAM went down by
about 190 bytes. The real gains are the compact MQTT frame and the safer
interrupt hand-over.

---

## Tuning (`b5_config.h`)

| Value | Default | Where it comes from |
|---|---|---|
| `B5_D_STOP_MM` | 300 | stop distance D_stop; tune on the track |
| `B5_D_SLOW_MM` | 500 | slow-down request (FR-USO-02) |
| `B5_PING_PERIOD_MS` | 60 | NFR-USO-06 |
| `B5_CONFIRM_COUNT` | 3 | Confirm state |
| `B5_SAMPLES_PER_ANGLE` | 3 | median of three (A6) |
| `B5_ALPHA_DD` | 0 | beam-spread correction. Measured (S5-C04, 1° scans): a 230 mm box read its true width uncorrected, a 100 mm box about 3× too wide, because a flat face echoes from wherever it is within the beam (±25–35°). No single correction fits both; 0 never makes an obstacle look narrower than it is |
| `B5_REFINE_STOP_DD` | 10 (1°) | see below |
| `B5_SERVO_MIN_US` / `MAX_US` | 500 / 2400 | 0 (1450 µs) faces straight ahead on our car (S5-C01). MIN = full right, MAX = full left |
| `B5_SERVO_SPEED_DD_S` | 6000 (600°/s, the SG90's own speed) | how fast the servo turns; 600 (60°/s) if the bracket hits the wiring (it did before the cables were re-routed); slower costs about 3 s of scanning |
| `B5_SERVO_SETTLE_MS` | 40 | rest after each move before the next ping (A8, S5-C02) |
| `B5_CAR_WIDTH_MM` / `LENGTH_MM` | 150 / 200 | **placeholders: measure the PicoCar** |
| `B5_MARGIN_MM` | 50 | clearance margin |
| `B5_MIN_CONFIDENCE_PCT` | 50 | Planning |
| `B5_BLOCKED_RETRIES` | 2 | FR-USO-07 |
| `B5_RECOVER_ATTEMPTS` | 3 | LINE_LOST |

---

## Where the code differs from the Week 6 text

The report says the design can still change after Week 6, so these
differences are recorded here rather than hidden:

1. **Edge Refinement stops below 1°, not at the beam half-angle.** The host
   test compares the two. With an ideal beam, stopping at α leaves the
   width up to **42 %** off. That fails NFR-USO-02 for small or distant
   obstacles. Stopping below 1° keeps the whole grid within **6 %**
   (±55° sweep; it was 48 % and 8 % with the earlier ±60° sweep).
   Refinement samples use one ping each (`B5_REFINE_PINGS`) to save time.
   If the A6 bench test shows the real beam edge is too soft for this to
   help, set `B5_REFINE_STOP_DD` back to `B5_ALPHA_DD`.
2. **Guard can also go to Fault.** In Guard, three pings in a row with no
   echo raise ECHO_TIMEOUT, so the car never keeps driving without forward
   sensing. The chart only shows the timeout from the Scanning substates.
3. **Small additions the chart leaves implicit:**
   - mission stop from any state returns to Idle;
   - no stop acceptance within 1 s goes to Fault;
   - a rejected plan goes to Blocked;
   - a failed or timed-out motion goes to Fault;
   - in Blocked, if reversing is not safe, the car stays stopped with the
     route reported (FR-USO-07).
4. **"Servo timeout" is a register check.** The SG90 has no position
   feedback, so the driver can only detect that the PWM is not running or
   not holding the commanded pulse.

### Timing budget (NFR-USO-05: confirmation → line reacquired < 3 s)

| Step | Estimate |
|---|---|
| Confirm (3 × 60 ms) | 0.18 s |
| Halting (servo to −55° at 60°/s) | 0.9 s |
| Coarse sweep: 4 × 27.5° of travel + 5 × (3 pings + settle) | 1.8 + 1.1 s |
| Edge refinement: ≤ 10 × (1 ping + settle) + travel | ≤ 1 + 1.8 s |
| **Scanning total** | **about 4–7 s** |

The servo speed limit (60°/s, added because the full-speed swing snapped
the sensor bracket into the wiring) costs about 3 s of scanning, so 3 s in
total is out of reach (it was already tight at 2 s of scanning). If the
mounting allows it, raising `B5_SERVO_SPEED_DD_S` is the biggest saving;
one ping per coarse-sweep angle saves another 0.6 s.

---

## Things to agree with the team

1. **One GPIO interrupt for everyone.** The RP2040 has a single GPIO
   interrupt. Encoders (S2) and any other edge-driven input must register
   with `sys_gpio_irq_attach()` in `../system/sys_hal.c`, not with their
   own `tk_def_int(13, …)`.
2. **Priority bands.** The template gives S5 band 20–21, below S1–S4.
   Section 3 of the report puts Obstacle Scan above IMU Terrain and WiFi.
   It works either way, because the 60 ms ping cycle is long, but the team
   may want to re-order the bands in TEAM.md section 4.
3. **Car size.** `B5_CAR_WIDTH_MM` and `B5_CAR_LENGTH_MM` are placeholders
   (150 × 200 mm); measure the PicoCar.
