# Shared system code

Group IS28 · shared by the whole team · `app_program/system/`

This folder has the code that belongs to no single buddy:

| Part | Files | What it does |
|---|---|---|
| **Navigation Coordinator** | `nav_*` | Holds the car's operating state (report Section 5) and decides who may move the car. The only task that commands Motion Control. |
| **Safety Supervisor** | `safety_*` | Emergency stop and unrecoverable faults: cuts drive within 100 ms and keeps it cut until a reset. Watches that tasks are still alive. |
| **Shared HAL** | `sys_hal.*` | Microsecond timer, busy-wait, and the one GPIO interrupt shared between subsystems. |
| **Bench helpers** | `sys_bench.c`, `sys_config.h` | Stand-ins for subsystems that are not written yet, so the car can be tried today. Switched off one by one. |
| **Start-up** | `system.c`, `system.h` | `system_init()` and `system_start()`, called by `app_main.c` before the subsystems. |

```
WiFi and MQTT (S1) ──commands──▶┐
Line and Barcode (S3) ─speeds, events, barcodes─▶┤
Obstacle Scan (S5) ─requests─▶┤  Navigation   ──commands──▶ Motion Control
Safety Supervisor ─e-stop / fault─▶┤  Coordinator  ◀──results───  (S2)
                                   └──▶ operating state to every task
```

Subsystem numbers follow the report: S1 WiFi and MQTT, S2 Motion Control,
S3 Line and Barcode, S4 IMU Terrain, S5 Obstacle Scan and Recovery.

---

## 1. Navigation Coordinator

Task `nav_task`, priority 10, woken by messages in its inbox (a message
buffer) and at least every 100 ms.

### Operating state (Section 5)

| From | Event | To |
|---|---|---|
| Ready | start command | Running |
| Running | pause | Paused (motors stopped) |
| Paused | resume | Running |
| Running, Paused | stop | Safe Stop |
| Running | course finished | Completed |
| Safe Stop, Completed | reset | Ready |
| any | emergency stop (from Safety Supervisor) | Emergency Stop |
| any | unrecoverable fault (from Safety Supervisor) | Fault |
| Emergency Stop, Fault | cleared + reset | Ready (Fault calls `nav_reinitialise()` first) |

Every time the car leaves Running, the coordinator stops the motors, takes
line control away from S3, and tells S5 the mission stopped.

### Who drives while Running (FR-SYS-02)

Exactly one behaviour owns motion at a time. A higher one takes over from
a lower one:

| Priority | Owner | Gets control when | Gives it back when |
|---|---|---|---|
| 1 (highest) | **Obstacle** (S5) | S5 asks for a stop | S5 releases motion |
| 2 | **Barcode turn** | a left / right / U-turn barcode is decoded | the turn finishes |
| 3 | **Line search** | the line is lost during normal following | the line is found again |
| 4 | **Line following** | default while Running | — |

While another behaviour owns motion, S3's wheel speeds are not forwarded.
A barcode that arrives while S5 is handling an obstacle is dropped, so
there is still one event per barcode (FR-LINE-05).

### Bounded moves

Bypass legs, barcode turns and line-search sweeps are sent to Motion
Control as short lists of commands. Each command carries an **id** and
Motion Control answers with that id. A late answer for a move that has
been replaced is ignored. A command that takes longer than 8 s counts as
failed.

| Request | Commands sent to Motion Control |
|---|---|
| S5 stop | `mc_stop` → reply `STOP_ACCEPTED` |
| S5 bypass plan (side, shift L, forward F) | turn 45° to that side → drive L × 1.414 → turn back 45° → drive F − L → reply `MOTION_DONE` |
| S5 rejoin (offset, heading, search width) | turn to face the line at 45° → drive offset × 1.414 + search width (max 600 mm) → reply `MOTION_DONE` |
| S5 reverse | drive backwards → reply `MOTION_DONE` |
| Barcode left / right / U-turn | turn −90° / +90° / 180° |
| Line lost | stop → turn +15° → −30° → +45° → −60°. Still lost: Safe Stop |

If S5 releases motion **from its Fault state**, the coordinator does not
resume line following (that would drive with no forward sensing). It
stops in Safe Stop and reports the fault.

---

## 2. Safety Supervisor

Task `safety_task`, priority 6 (highest of all application tasks), woken
by its event flag and every 20 ms for the health check.

- **Emergency stop** (FR-WIFI-02, NFR-SYS-04): motor drive is cut within
  100 ms and stays cut until a reset.
- **Unrecoverable faults** (FR-SYS-03): any task can report one. The car
  stops and stays stopped until reset.
- **Task health:** while the car is Running, a task that stops checking
  in is treated as an unrecoverable fault.
- **Stop timing:** it measures each stop, from accepting it to Motion
  Control confirming the motors are off (the evidence for NFR-SYS-04).

```
NORMAL ──emergency stop──▶ ESTOP ──reset request──▶ NORMAL
NORMAL ──critical fault──▶ FAULT ──reset request──▶ NORMAL
(an emergency stop during FAULT keeps FAULT)
```

On entering ESTOP or FAULT it calls `mc_inhibit(true)`, tells the
coordinator, and starts the stopwatch. The stopwatch stops when Motion
Control calls `safety_ack_drive_off()`. A stop of 100 ms or more is
counted and published as a warning.

It never drives the motor pins itself, because only Motion Control writes
them.

Which faults stop the car is decided in `safety_glue.c`:

| Reported | Critical? | Why |
|---|---|---|
| S5 ECHO_TIMEOUT, SERVO | yes | no forward sensing |
| S5 STOP_TIMEOUT, MOTION | yes | the car did not stop or move as asked |
| S5 LINE_LOST, BLOCKED | warning | the car is already stopped safely |
| Coordinator MOTION | yes | Motion Control failed a command |
| Coordinator LINE_LOST, OBSTACLE | warning | the car is already in Safe Stop |

### Health monitoring

```c
/* once, in subsystemN_init() */
safety_watch(SAFETY_SRC_MOTION, 100);    /* expect a check-in every 100 ms */

/* in the task's own loop */
safety_kick(SAFETY_SRC_MOTION);
```

Check-ins are only needed while the coordinator is **Running**, so a task
can block while the car is idle. A task that never calls `safety_watch()`
is not monitored, so monitoring can be switched on one task at a time.

---

## 3. What each buddy needs to do

The interfaces are [`nav_iface.h`](nav_iface.h) and
[`safety_iface.h`](safety_iface.h). Units are mm, mm/s and tenths of a
degree. **Positive angles are to the right.**

- `nav_post_*()` never blocks. Call it from a task, not an interrupt.
- `safety_*()` never blocks and is safe from a task **or** an interrupt.

### S1 — WiFi and MQTT

- `nav_post_command(NAV_CMD_START / PAUSE / RESUME / STOP / RESET)` for
  each permitted supervisory command.
- Emergency stop: call `safety_post_estop()` (not the coordinator).
- Reset: call `safety_request_reset()` **and**
  `nav_post_command(NAV_CMD_RESET)`.
- Write `nav_telemetry_publish()`, `safety_telemetry_publish()` and
  `b5_telemetry_publish()`: copy the status into the telemetry queue that
  the MQTT code reads (TEAM.md section 3, "WiFi and MQTT are special").
  Then set `SYS_CONSOLE_STATUS` to 0.
- Optional: `wifi_notify_state(nav_state_t)`;
  `safety_watch(SAFETY_SRC_WIFI, 3000)` + `safety_kick()` per heartbeat.

### S2 — Motion Control

- Write:
  - `mc_set_wheel_speeds(left, right)`: line following (FR-MOT-05);
  - `mc_drive_distance(mm, speed, id)`: negative = reverse (FR-MOT-01);
  - `mc_turn(angle_dd, speed, id)`: positive = right;
  - `mc_stop(id)`: controlled stop to standstill;
  - `mc_inhibit(bool)`: **true** = motor outputs to zero at once, answer
    the current command with `NAV_MOTION_CANCELLED`, refuse new commands;
    **false** = accept commands again;
  - `b5_motion_get_estimate()` for S5 (see `../subsystem5/README.md`).
- Call `nav_post_motion_result(id, DONE / CANCELLED / FAILED)` when each
  command with an id finishes (FR-MOT-04). A new command replaces the
  current one; answer the replaced one with `CANCELLED`.
- Call `safety_ack_drive_off()` as soon as the motors are actually off.
- Also check `safety_drive_inhibited()` at the top of every control cycle.
- Recommended: `safety_watch(SAFETY_SRC_MOTION, 100)` and
  `safety_kick()` every cycle.

### S3 — Line and Barcode

- Call `nav_post_line_speeds(left, right)` every control cycle.
- Call `nav_post_line_event(NAV_LINE_LOST / FOUND / JUNCTION)`.
- Call `nav_post_barcode(NAV_BARCODE_LEFT / RIGHT / STRAIGHT / UTURN)`
  once per barcode, **at the moment the turn should start**.
- Write `line_set_control(bool)`: only propose speeds while it is true.
- Write `b5_line_detected()` for S5.
- A broken sensor: `safety_report_fault(SAFETY_SRC_LINE, code, true)`.

Once S2 and S3 are in, set `SYS_SIM_CAR` to 0.

### S4 — IMU Terrain

- Optional: `imu_notify_state(nav_state_t)` to start and stop sampling
  with the mission.
- Shock warning: `safety_report_fault(SAFETY_SRC_IMU, code, false)`. It is
  published but does not stop the car.
- Recommended: `safety_watch()` + `safety_kick()` every 20 ms.

### S5 — Obstacle Scan and Recovery

Nothing to do. `nav_b5.c` turns S5's requests into coordinator messages
and `safety_glue.c` connects its faults.

### Still a placeholder

`nav_reinitialise()`: after a Fault the car goes back to Ready without
re-running calibration until the team's start-up code provides it.

---

## 4. Placeholders and the switches in `sys_config.h`

Two kinds of stand-in let the code build before everyone's part exists.

**Weak placeholders** (`nav_stubs.c`, `safety_stubs.c`). A function marked
`__attribute__((weak))` is used only if nobody writes a real one with the
same name. When you write the real function in your own folder, it
replaces the placeholder automatically. The placeholders fail safe:
Motion Control never answers, so a move times out and the car goes to
Safe Stop.

**Bench helpers** (`sys_bench.c`), switched in `sys_config.h`:

| Switch | When 1 | Set to 0 when |
|---|---|---|
| `SYS_SIM_CAR` | Simulates S2 and S3: a virtual car follows a line at 150 mm/s | S2 and S3 provide `mc_*()`, `line_set_control()`, `b5_motion_get_estimate()`, `b5_line_detected()` |
| `SYS_CONSOLE_STATUS` | Prints coordinator, supervisor and S5 status on the console | S1 provides the `*_telemetry_publish()` functions |
| `SYS_BENCH_BUTTONS` | Robo Pico buttons: **GP20** = emergency stop, **GP21** = reset and start | the supervisor PC is connected over MQTT |
| `SYS_SIM_AUTOSTART_MS` | Starts a run by itself this long after power-up (0 = wait for GP21) | — |

If the build fails with **"multiple definition of ..."**, someone has
written the real version of a helper. Set the matching switch to 0.

---

## 5. Shared HAL (`sys_hal.*`)

The RP2040 has **one** GPIO interrupt for all pins. Only one handler can
own it, so it lives here and passes each edge on to whoever registered
that pin:

```c
#include "../system/sys_hal.h"

static void on_edge (uint32_t gpio, uint32_t edges, uint32_t t_us) { ... }

sys_gpio_irq_attach(17U, SYS_GPIO_EDGE_RISE | SYS_GPIO_EDGE_FALL, on_edge);
```

Use it for wheel encoders, the ultrasonic echo, buttons, anything with a
pin interrupt. The callback runs in the interrupt: keep it short and wake
your task with an event flag.

Also: `sys_time_us()` (free-running microsecond timer) and
`sys_busy_wait_us()` for short pulses.

---

## 6. Try it on the bench

```sh
cd build_make
make -j8                 # normal image
make TEST=1 -j8          # test image (runs every subsystemN_test.c)
```

Flash the `.uf2` and open the USB console. With the default switches, the
Safety Supervisor, the coordinator and S5 run for real, and the car is
simulated. Hold something closer than 300 mm in front of the ultrasonic
sensor to watch:

1. S5 stops the car through the coordinator;
2. S5 scans;
3. the coordinator drives the bypass legs;
4. the car rejoins the line and line following takes over.

Press GP20 for an emergency stop; the console shows how many ms the drive
took to switch off. GP21 resets and starts again.

## 7. Host tests

No Pico needed (see [`../../testing/host/README.md`](../../testing/host/README.md)):

```sh
make -C testing/host
```

| Test | Checks | Result |
|---|---|---|
| `test_nav` | start and line following, a full bypass, late results ignored, barcode turns, line search, pause/resume/reset, e-stop latching, S5 fault, motion timeout, inhibit not treated as a motion fault | 48 passed |
| `test_safety` | e-stop and timing, reset order, slow stop flagged, e-stop during a fault, warnings, silent task caught only while Running | 25 passed |

## 8. To agree as a team

- **Priorities.** The template bands put S1–S4 above S5, while the report
  (Section 3) has obstacle handling above line following. See
  `../subsystem5/README.md`.
- **Barcode timing:** the turn starts as soon as the barcode is posted.
- **Speeds and angles** in `nav_config.h` are starting values to tune on
  the real car.
- **Hardware watchdog:** a later addition could kick the RP2040 watchdog
  only while every watched task is healthy.
