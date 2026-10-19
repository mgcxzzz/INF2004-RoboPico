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
| 1 | *example:* LED heartbeat on GP19 | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | *example:* Producer: makes a sample every 300 ms | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | *example:* Consumer: collects samples into statistics | | | `app_program/subsystem3/` | `testing/subsystem3/` |
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
| GP6 | | |
| GP7 | | |
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
| GP19 | *example:* S1 | external LED (LED + 330 Ω to GND) |
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
| GPIO interrupt (IO_IRQ_BANK0) | system | `sys_hal.c` dispatcher; each owner registers its own pins with `sys_gpio_irq_attach()` | any |
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

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | *example:* led_task – 12 – any |
| 2 | 14–15 | *example:* producer_task – 14 – core 1 |
| 3 | 16–17 | *example:* consumer_task – 16 – core 0 |
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
| IF-08 | Nav → S2 | function calls | `mc_set_wheel_speeds`, `mc_drive_distance`, `mc_turn`, `mc_stop` | commands with an id; mm, mm/s, 0.1° (+ = right) (`nav_iface.h`) | to agree with S2 |
| IF-09 | S2 → Nav | Nav inbox | `nav_post_motion_result()` | id + DONE / CANCELLED / FAILED | to agree with S2 |
| IF-10 | S3 → Nav | Nav inbox | `nav_post_line_speeds/_event()`, `nav_post_barcode()` | wheel speeds each cycle; LOST/FOUND/JUNCTION; one barcode at turn point | to agree with S3 |
| IF-11 | Nav → S3 | function call | `line_set_control(bool)` | S3 proposes speeds only while true | to agree with S3 |
| IF-12 | S1 → Nav, Safety | Nav inbox / safety event flag | `nav_post_command()`, `safety_post_estop()`, `safety_request_reset()` | supervisory commands (`nav_iface.h`, `safety_iface.h`) | to agree with S1 |
| IF-13 | Nav, Safety, S5 → S1 | function calls into S1's telemetry queue | `*_telemetry_publish()` | Nav, Safety: status structs. S5: 16-byte packed binary frame `b5_telemetry_publish(frame, len)` (bit map in `subsystem5/b5_pack.h`) | to agree with S1 |
| IF-14 | Safety → S2 | function call + polled flag | `mc_inhibit(bool)`, `safety_drive_inhibited()`; S2 answers `safety_ack_drive_off()` | cut drive now (< 100 ms, NFR-SYS-04) | to agree with S2 |
| IF-15 | any → Safety | safety event flag | `safety_report_fault()`, `safety_kick()` | fault source + code; health check-ins | team |
| IF-16 | S2, S3 → S5 | function calls | `b5_motion_get_estimate()`, `b5_line_detected()` | distance + heading as `int16_t` (net signed distance in mm, goes negative on reverse, may wrap; S5 only uses differences); line seen | to agree with S2, S3 |
| IF-17 | any → all | shared fault event flag | `sys_fault_set()`, `sys_fault_clear()`, `sys_fault_get()` (`system/sys_fault.h`) | one bit per fault (motor stall, encoder, IMU, no echo, WiFi loss, servo, line sensor, obstacle); set/clear are atomic | team |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | system |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | 1 | 3 (safety, nav, bench) |
| Semaphore | 16 | | 1 | | | | 1 (bench) |
| Event flag | 16 | | | | 1 | 2 | 2 (safety, fault mask) |
| Mailbox | 12 | | | 1 | | | |
| Mutex | 12 | | | 1 | | 1 | |
| Message buffer | 12 | | | | 1 | | 1 (nav inbox) |
| Fixed memory pool | 12 | | 1 | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | 1 | | | | | |
| Alarm handler | 12 | | | | 1 | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
