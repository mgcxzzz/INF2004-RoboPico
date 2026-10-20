# Shared system code

Group IS28 · shared by the whole team · `app_program/system/`

This folder has the code that belongs to no single buddy:

| Part | Files | What it does |
|---|---|---|
| **Navigation Coordinator** | `nav_*` | Holds the car's operating state (report Section 5) and decides who may move the car. The only task that commands Motion Control. |
| **Safety Supervisor** | `safety_*` | Emergency stop and unrecoverable faults: cuts drive within 100 ms and keeps it cut until a reset. Watches that tasks are still alive. |
| **Telemetry schema** | `sys_tlm.*` | The team's one shared binary telemetry format (common header, every subsystem's payload layout) and the bit helpers (Tribunal 1, minimum bits). |
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

Fault codes are **8 bits**. Each source numbers its own codes; the source
is recorded separately, so codes from different buddies cannot clash.

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
Timeouts are stored in 16 bits, so the longest is 65535 ms.

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
- Write `nav_telemetry_publish(frame, len)`,
  `safety_telemetry_publish(frame, len)` and
  `b5_telemetry_publish(frame, len)`. All three hand over a packed binary
  frame in the **team schema** [`sys_tlm.h`](sys_tlm.h) (summary in
  section 6): a 3-byte common header, then the payload. Copy the bytes
  into the telemetry queue that the MQTT code reads (TEAM.md section 3,
  "WiFi and MQTT are special") and publish them unchanged. The PC reads
  the header to know which subsystem sent it.
- Your own frames (heartbeat, link state) follow the same schema: add
  your payload section to `sys_tlm.h` under source 1.
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

## 5. Shared fault bitmask (`sys_fault.*`)

Tribunal 1 decision: fault conditions are single bits in one shared mask,
not separate integer variables. **Owner: the Safety Supervisor**; each
buddy owns its own bits. The mask is a µT-Kernel **event flag**, so
the kernel sets and clears bits as one operation: safe from any task or
interrupt on either core. A plain `faults |= bit` is not, because the
Cortex-M0+ has no atomic read-modify-write instruction.

| Bit | Name | Owner |
| --- | --- | --- |
| 0 | `SYS_FAULT_MOTOR_STALL` | S2 |
| 1 | `SYS_FAULT_ENCODER` | S2 |
| 2 | `SYS_FAULT_IMU` | S4 |
| 3 | `SYS_FAULT_NO_ECHO` | S5 (done) |
| 4 | `SYS_FAULT_WIFI_LOSS` | S1 |
| 5 | `SYS_FAULT_SERVO` | S5 (done) |
| 6 | `SYS_FAULT_LINE_SENSOR` | S3 |
| 7 | `SYS_FAULT_OBSTACLE` | S5 (done): stop timeout, motion failure, line lost, blocked |

```c
#include "../system/sys_fault.h"

sys_fault_set(SYS_FAULT_MOTOR_STALL);     /* only your own bits */
sys_fault_clear(SYS_FAULT_MOTOR_STALL);
uint32_t active = sys_fault_get();        /* e.g. for telemetry */
```

**Set, latch and clear rules** (the Tribunal 1 follow-up):

| Rule | What it means | Kernel call |
|---|---|---|
| Set | only the bit's owner, from a task or an interrupt, as soon as the fault is detected | `sys_fault_set()` → `tk_set_flg` (one kernel operation) |
| Latch | a bit stays set until it is cleared on purpose, so a fault that comes and goes is still visible until a reset. Nobody waits on this flag with `TWF_CLR` / `TWF_BITCLR`, which would wipe other buddies' bits; the module offers no wait call at all | `sys_fault_get()` → `tk_ref_flg` (read only, never blocks) |
| Clear | only the bit's owner, and only when **both** the fault condition is gone **and** a reset has been accepted (Buddy 5 clears its bits on `B5_EV_FAULT_RESET`) | `sys_fault_clear()` → `tk_clr_flg` |

Severity (what stops the car) is still decided by the Safety Supervisor
through `safety_report_fault()`; the mask is the compact shared record.
The Safety Supervisor publishes the whole mask in its telemetry frame and
never sets or clears another buddy's bit.

## 6. Minimum-bit data in the coordinator and the supervisor

The same Tribunal 1 rules as Buddy 5 (`../subsystem5/README.md`):

- every value is stored in only the bits its range needs;
- values are **clamped or saturated** before they are stored, never
  wrapped;
- C bit-fields only for records a single task owns;
- masks and shifts in a fixed little-endian layout for anything that
  leaves the chip.

**Who writes what** (the part that matters for races):

| Data | Bits | Written by | Why it is safe |
|---|---|---|---|
| coordinator state + motion owner | 3 + 3, one byte | coordinator only | one byte store, so a reader always sees a matching pair |
| supervisor state + drive inhibited | 2 + 1, one byte | supervisor only | same |
| coordinator records (manoeuvre, context) | bit-fields | coordinator only | single writer |
| inbox message | 8 bytes (was 16): type 4, a 16, b 16, c 12 | sender, then the coordinator | copied into and out of the kernel message buffer, never shared |
| dropped-message count | 0..255, saturating | **any** task whose post failed | a kernel **semaphore** used as a counter: `tk_sig_sem()` adds 1 as one kernel operation and refuses beyond 255. A plain `count++` from several tasks would lose counts |
| each source's last fault / warning code | 8 bits, one byte per source | that source | one byte each on purpose. Packing several sources into one word would make two reporters read-modify-write the same word |
| supervisor record (codes, counters, stop times) | bit-fields | supervisor only | single writer |
| timestamps | 32 bits | | kept 32-bit (team decision), wrap-safe differences |

**Ranges chosen**

| Value | Before | Now |
|---|---|---|
| Motion Control command id | 32 bits | 15 bits, 1..32767 then back to 1. An id above that was never issued, so its result is ignored. |
| step value (turn dd, drive mm) | 32 bits | 13 bits signed, ±4095 (clamped) |
| step speed | 16 bits | 8 bits, 0..255 mm/s |
| commands sent | 32 bits | 16 bits, wrapping |
| fault / warning code | 32 bits | 8 bits, each source's own numbering |
| e-stop, fault, warning, slow-stop counters | 32 bits | 8 bits, wrapping: use differences |
| stop time, worst stop time | 32 bits | 10 bits, ms, saturating at 1023 (= far too slow) |
| health timeout | 32 bits | 16 bits, ms |

**One telemetry schema for the whole team** ([`sys_tlm.h`](sys_tlm.h)).
This is Buddy 1's Tribunal 1 decision ("one standard packet format shared
by all devices, with consistent bit layout and scaling rules") and R3
("one shared header used by all buddies"). Every frame is:

| Part | Bytes | Contents |
|---|---|---|
| Common header | 3 | byte 0: schema version (bits 0–3) and source (bits 4–7: 1 WiFi, 2 Motion, 3 Line, 4 IMU, 5 Obstacle, 6 Nav, 7 Safety); byte 1: payload length; byte 2: sequence number per source, wrapping (the PC counts gaps for the ≥ 95 % delivery test) |
| Payload | per source | fixed field positions, widths, units and signs, all listed in `sys_tlm.h` |

The schema also fixes the rules: little-endian, two's complement with
sign extension, clamp before packing, reserved bits 0, and when the
version number changes. A frame whose header is wrong (version, source or
length) is rejected, not decoded.

| Source | Frame bytes | Payload |
|---|---|---|
| 6 Navigation Coordinator | 9 | state, owner, slow, search step, dropped (sat. 255), commands sent (16-bit) |
| 7 Safety Supervisor | 15 | state, inhibited, fault and warning source and code, 4 event counters, last and worst stop time, **shared fault mask** |
| 5 Obstacle Scan (Buddy 5) | 19 | state, fault, side, retries, servo angle (signed), ranges, confidence, width, shift, ping counters |
| 1–4 | — | to be added by Buddies 1–4 in the same file |

**Effect** (default build): static RAM of these two modules went from
555 to 278 bytes. The inbox alone went from 320 to 192 bytes. Total
`.bss` went from 25 512 to 25 228 bytes. Code is about 1.9 KB larger
(packing, clamping, frame builders). One more kernel object: the
dropped-message semaphore.

## 7. Shared HAL (`sys_hal.*`)

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

## 8. Try it on the bench

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

## 9. Host tests

No Pico needed (see [`../../testing/host/README.md`](../../testing/host/README.md)):

```sh
make -C testing/host
```

| Test | Checks | Result |
|---|---|---|
| `test_nav` | start and line following, a full bypass, late results ignored, barcode turns, line search, pause/resume/reset, e-stop latching, S5 fault, motion timeout, inhibit not treated as a motion fault; minimum bits: 8-byte inbox message, clamped step values, ids above 15 bits ignored, dropped count saturating at 255, every telemetry frame decoded, header sequence never skips (incl. across its wrap) | 63 passed |
| `test_safety` | e-stop and timing, reset order, slow stop flagged, e-stop during a fault, warnings, silent task caught only while Running; minimum bits: stop time saturating at 1023 ms, 8-bit counters wrapping, fault mask in the frame, every telemetry frame decoded, header sequence never skips | 36 passed |
| `test_b5_pack` | minimum-bit packing: record sizes, clamping, sign extension, 16-bit wrap, echo word, telemetry frame with the common header (wrong version, source or length rejected) | 43 passed |

## 10. To agree as a team

- **Priorities.** The template bands put S1–S4 above S5, while the report
  (Section 3) has obstacle handling above line following. See
  `../subsystem5/README.md`.
- **Barcode timing:** the turn starts as soon as the barcode is posted.
- **Speeds and angles** in `nav_config.h` are starting values to tune on
  the real car.
- **Hardware watchdog:** a later addition could kick the RP2040 watchdog
  only while every watched task is healthy.
