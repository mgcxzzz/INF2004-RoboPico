# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi and MQTT: supervisory commands, heartbeat and telemetry to the dashboard (WiFi and MQTT) | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | *example:* Producer: makes a sample every 300 ms | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following and barcode navigation (Line and Barcode). The template Consumer stays until S2/S4 drop their template tasks | 2501910 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | *example:* Monitor: prints reports, pauses the LED | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Adaptive ultrasonic scanning and obstacle profiling (Obstacle Scan and Recovery) | 2500339 | | `app_program/subsystem5/` | `testing/subsystem5/` |
| – | Shared: Navigation Coordinator, Safety Supervisor, shared HAL, bench helpers | whole team | | `app_program/system/` | `testing/host/`, `testing/integration/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | | |
| GP3 | | |
| GP4 | | |
| GP5 | | |
| GP6 | S3 | LINE_LEFT front IR D0 (Grove 5; GP26 on that cable left unconnected) |
| GP7 | S3 | LINE_RIGHT front IR D0 (Grove 7; GP28 on that cable left unconnected) |
| GP8 | | |
| GP9 | | |
| GP10 | | |
| GP11 | | |
| GP12 | S5 | SG90 scanning servo, PWM 6A (Robo Pico servo port 1) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S5 | HC-SR04+ TRIG (Grove 4) |
| GP17 | S5 | HC-SR04+ ECHO, edge interrupt via `sys_gpio_irq_attach()` (Grove 4) |
| GP18 | | |
| GP19 | S3 | BARCODE_IR side-offset IR D0 (header, physical pin 25), edge interrupt via `sys_gpio_irq_attach()` |
| GP20 | system (bench) | Robo Pico button: emergency stop. Only while `SYS_BENCH_BUTTONS` = 1 |
| GP21 | system (bench) | Robo Pico button: reset and start. Only while `SYS_BENCH_BUTTONS` = 1 |
| GP22 | | |
| GP26 | | (ADC0 capable) |
| GP27 | | (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | | GP26–GP28 |
| PWM slices (list) | S5 | `obstacle_task` | slice 6 (GP12) |
| GPIO interrupt (IO_IRQ_BANK0) | system | `sys_hal.c` dispatcher; each owner registers its own pins with `sys_gpio_irq_attach()` (S2 encoders, S3 GP19, S5 GP17) | any |
| TIMER (µs counter) | system | read-only for everyone via `sys_time_us()` | – |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).
- **Done this way by S1** (`buddy1` branch, IF-25): `mqtt_utk.c` is now
  S1's network side and `app_program/subsystem1/task_wifi.c` is its task
  side. The two meet only in `b1_net.h` (two message buffers, all calls
  `TMO_POL`). `lwipopts.h` raises `MQTT_OUTPUT_RINGBUF_SIZE` to 1024.
  *To agree with the team and the instructor.*

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | wifi_task – 12 – any; led_idle – 13 – any (does nothing; only while the template Monitor still uses `tid_led`, IF-05) |
| 2 | 14–15 | *example:* producer_task – 14 – core 1 |
| 3 | 16–17 | line_task – 16 – any; consumer_task – 17 – core 0 (template demo) |
| 4 | 18–19 | *example:* monitor_task – 18 – any |
| 5 | 20–21 | obstacle_task – 20 – any |
| system | 5–11 | safety_task – 6 – any; nav_task – 10 – any; bench_task – 11 – any (bench only) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *example* |
| IF-06 | S5 → Nav | function calls → Nav inbox (message buffer, owned by Nav) | `b5_nav_request_*()`, `b5_nav_release_motion()` | stop, bypass plan, rejoin, reverse requests (`b5_iface.h`) | S5 + team |
| IF-07 | Nav → S5 | event flag (owned by S5) | `b5_post_event()` | `MISSION_START/STOP`, `STOP_ACCEPTED`, `PLAN_ACCEPTED/REJECTED`, `MOTION_DONE/FAILED`, `FAULT_RESET` (`b5_iface.h`) | S5 + team |
| IF-08 | Nav → S2 | function calls | `mc_set_wheel_speeds`, `mc_drive_distance`, `mc_turn`, `mc_stop` | commands with an id; mm, mm/s, 0.1° (+ = right) (`nav_iface.h`). `mc_turn` must spin on the spot (barcode turns pivot on the junction, up to 360°, stopped early by a new command) | to agree with S2 |
| IF-09 | S2 → Nav | Nav inbox | `nav_post_motion_result()` | id + DONE / CANCELLED / FAILED | to agree with S2 |
| IF-10 | S3 → Nav (→ S2) | direct call while granted; Nav inbox for events | `nav_line_drive()`; `nav_post_line_event()`, `nav_post_barcode()` | wheel speeds every 10 ms go straight to Motion Control while S3 holds line control (no inbox in the steering loop, Week 6 feedback); LOST / FOUND / JUNCTION; one barcode event per barcode. Barcode turns assume no angle: LEFT / RIGHT posted with the axle over the next junction, U-turn right after the barcode; Nav stops and spins slowly; S3 posts FOUND when the leading front sensor reaches a line; Nav hands control back (`nav_barcode_t` in `nav_iface.h`) | S3 + team |
| IF-11 | Nav → S3 | function calls (single-writer bytes + S3 event flag) | `line_set_control(bool)`, `line_notify_state()` | S3 steers only while true; operating state for statechart 6.3 | S3 + team |
| IF-12 | S1 → Nav, Safety | Nav inbox / safety event flag | `nav_post_command()`, `safety_post_estop()`, `safety_request_reset()` | supervisory commands (`nav_iface.h`, `safety_iface.h`). S1 sends only START, PAUSE, RESUME, STOP, RESET (= `safety_request_reset()` + `NAV_CMD_RESET`) and ESTOP (straight to Safety); steering and drive commands are rejected (FR-WIFI-03) | S1; to confirm with team |
| IF-13 | Nav, Safety, S3, S4, S5 → S1 | function calls into S1's telemetry queue (never block; a full queue drops the frame and counts it) | `nav_telemetry_publish(frame, len)`, `safety_telemetry_publish(frame, len)`, `line_telemetry_publish(frame, len)`, `imu_telemetry_publish(frame, len)`, `b5_telemetry_publish(frame, len)`, or any source with `b1_telemetry_publish(frame, len)` (`subsystem1.h`) | binary frames in the **one team schema** `system/sys_tlm.h`: 3-byte common header (version, source, length, sequence) + payload. Nav 9 bytes, Safety 15 (incl. the shared fault mask), S3 15, S5 19. S1 queues the bytes unchanged (≤ 40 bytes) and publishes them on `is28/tlm`; its own heartbeat (source 1, 12 bytes) goes on `is28/hb`. Buddies 2 and 4 add their payloads to the same file | S1; to confirm with team |
| IF-14 | Safety → S2 | function call + polled flag | `mc_inhibit(bool)`, `safety_drive_inhibited()`; S2 answers `safety_ack_drive_off()` | cut drive now (< 100 ms, NFR-SYS-04) | to agree with S2 |
| IF-15 | any → Safety | safety event flag | `safety_report_fault()`, `safety_kick()` | fault source + 8-bit code (each source numbers its own); health check-ins (timeout ≤ 65535 ms) | team |
| IF-16 | S2, S3 → S5 | function calls | `b5_motion_get_estimate()`, `b5_line_detected()` | distance + heading as `int16_t` (net signed distance in mm, goes negative on reverse, may wrap; S5 only uses differences); line seen | to agree with S2, S3 |
| IF-17 | any → all | shared fault event flag | `sys_fault_set()`, `sys_fault_clear()`, `sys_fault_get()` (`system/sys_fault.h`) | one bit per fault (motor stall, encoder, IMU, no echo, WiFi loss, servo, line sensor, obstacle); set/clear are atomic. Only a bit's owner sets or clears it; bits latch until the owner clears them after the fault is gone and a reset was accepted; nobody waits on the flag with a clear option | Safety Supervisor (mask); each buddy (own bits) |
| IF-18 | S2 → S3 | function call | `s3_motion_get_distance()` | net signed distance, `int16_t` mm, may wrap (same rule as IF-16); read every 10 ms to place barcode edges by distance. Until S2 provides it, S3 reuses `b5_motion_get_estimate()` | to agree with S2 |
| IF-19 | S2 (Motion Control) | rule | `NAV_LINE_STALE_MS` (50 ms) | a wheel-speed target not renewed within 50 ms is stale: stop (Week 6 statechart 6.2) | to agree with S2 |
| IF-25 | S1 task ↔ S1 network side (`mqtt_utk.c` in `cyw43_utk_task`) | two message buffers owned by S1, `TMO_POL` on the network side | `b1_net_take()`, `b1_net_command()`, `b1_net_heartbeat()`, `b1_net_set_state()`, `b1_net_count()` (`subsystem1/b1_net.h`) | outgoing: kind byte + frame (telemetry or command answer); incoming: raw `is28/cmd` payload. Topics `is28/tlm`, `is28/hb`, `is28/ack`, `is28/cmd`, last will `is28/lwt`; command format in `system/sys_tlm.h` | S1; needs team + instructor (shared library file) |
| IF-26 | PC ↔ S1; later S1 → S3 | MQTT configuration; coherent snapshot under S1-owned mutex | `b1_barcode_mapping_get()`, pure `b1_mapping_lookup()` | four distinct A–Z assignments for Forward/U-turn/Left/Right, atomic whole-map update; 5-byte SET, 4-byte GET, source-8 readback (schema and `tools/dashboard/README.md`); RAM-only, no default letters; getter is task-only and never waits | S1 implemented; real MQTT tests and S3 junction integration pending |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | system |
|---|---|---|---|---|---|---|---|
| Task | 32 | 2 (wifi, led_idle stand-in) | 1 | 2 (line, template consumer) | 1 | 1 | 3 (safety, nav, bench) |
| Semaphore | 16 | 1 (dropped-frame counter) | 1 | | | | 2 (bench, nav dropped-message counter) |
| Event flag | 16 | | | 1 | 1 | 2 | 2 (safety, fault mask) |
| Mailbox | 12 | | | 1 | | | |
| Mutex | 12 | 1 (barcode mapping snapshot) | | 1 (template) | | 1 | 1 (nav mode lock for `nav_line_drive`) |
| Message buffer | 12 | 2 (outgoing, commands) | | | 1 | | 1 (nav inbox) |
| Fixed memory pool | 12 | | 1 | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | | | | | | |
| Alarm handler | 12 | | | | 1 | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
