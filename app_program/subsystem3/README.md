# Subsystem 3: Line Following and Barcode Navigation

Group IS28 · Buddy 3 (2501910) · RTOS task **Line and Barcode** · priority 16

Keeps the car on the black line with the two front IR sensors, reads the
course's Code 39 barcodes with the side-offset sensor, and turns each
barcode into **exactly one** left / right / straight / U-turn event for
the Navigation Coordinator. It never drives the motors itself.

Requirements: FR-LINE-01 to FR-LINE-05, NFR-LINE-01 (on the line for 30 s
in 3 lighting conditions), NFR-LINE-02 (19 of 20 barcodes, no command for
an unreadable one). Design: Week 6 report, section 6.3. Updated for the
Week 6 feedback and the Tribunal 1 minimum-bit change (below).

## Files

| File | What it does |
|---|---|
| `task_line.c` | the task: statechart 6.3, one cycle every 10 ms (`s3_step()`) |
| `s3_line.c/.h` | line position, steering proposal, line lost / found, junction, stuck sensor |
| `s3_reader.c/.h` | Read Barcode: GP19 edges → bar and space widths by **distance** |
| `s3_barcode.c/.h` | Code 39 decoding (and encoding, for tests) |
| `s3_ir.c` | the three sensors; GP19 edge interrupt into a single-writer ring |
| `s3_pack.c` | telemetry frame (layout in the team schema `../system/sys_tlm.h`, source 3) |
| `s3_config.h` | pins, polarity, timing, speeds, thresholds, **command table** |
| `s3_iface.h` | interface with the other subsystems |
| `s3_stubs.c` | weak defaults for what Buddies 1 and 2 provide |
| `subsystem3_test.c` | on-target tests (`make TEST=1`) |
| `task_consumer.c` | template demo, kept until S2/S4 drop their template tasks |

## The course (Barcode_Sample.pdf and the course photo)

- Line: black, **18 mm** wide.
- Commands (Project Write-up, "Barcode Navigation"): **A** turn left,
  **B** turn right, **C** go straight, **D** U-turn. The write-up calls the
  table an example, so check the barcodes on the final course.
- Barcodes: **Code 39**, `*A*` and `*Z*` in the sample, about **141 mm**
  long, printed **20 mm beside the line**. Narrow element **3.0 mm**, wide
  **9.0 mm** (3 : 1). The half-size print in the same file (1.5 mm) also
  decodes.
- Mount the side sensor so it passes over the bars: at least
  9 + 20 = 29 mm from the line's centre, better about 40–50 mm.

## How it works (statechart 6.3)

| State | Does | Leaves when |
|---|---|---|
| Initialising | clears the line tracker and the barcode reader; pins and GP19 interrupt were set up in `s3_init()` | set-up OK → Idle (failed → Fault) |
| Idle / Ready | front sensing only | coordinator Running → Line Following |
| Line Following | steering proposal every 10 ms via `nav_line_drive()` (used only while line control is granted) | first bar [armed] → Read Barcode; line lost → Await |
| Read Barcode | keeps steering; the reader collects edges | quiet zone → Decode and Request |
| Decode and Request | decode once; valid + in the command table → **one** `nav_post_barcode()` at the turn point | valid → Await; invalid, backwards, timeout → Line Following, no event |
| Await Navigation | no proposals; after LINE_LOST, reports LINE_FOUND once when a front sensor sees the line | line control granted again → Line Following |
| Fault / Inhibited | no proposals | coordinator Ready again (authorised reset) → Initialising |

Stop, pause or mission complete → Idle. Emergency stop or coordinator
Fault → Fault / Inhibited.

**One event per barcode (FR-LINE-05).** After any read the reader is
disarmed and re-arms only after the barcode sensor has seen white for
40 mm of travel. Turning in place does not count as travel, so the side
sensor sweeping back over the barcode during a turn can never start a
second read.

**Bad reads are not faults (NFR-LINE-02).** Wrong length, unclear
wide/narrow ratio, an unknown character, missing `*`, a **backwards** read,
a timeout or lost edges: no event, the run continues, and the reason is
counted in the telemetry. The Week 6 drawing routes an invalid read through
Await Navigation ("request hold"); since no event is sent, the coordinator
would never hand control back, so the code goes straight back to Line
Following, which is what the Week 6 text ("the run continues") says.

## Week 6 feedback, and what the code does about it

| Feedback | Answer in the code |
|---|---|
| "How many encoder-distance samples does a narrow bar get? If few, use edge interrupts on GP19 with encoder timestamps." | 3 mm bar at 150 mm/s and a 10 ms task = **2 samples** (1.5 mm print: 1). Not enough, so **every GP19 edge is timestamped by interrupt** (µs) and the task places it on the distance axis by interpolating between the motion-estimate readings it takes every cycle (`s3_reader.h`). Bars read the same width at any speed, even if the car speeds up, slows down or stops in the middle of a barcode. |
| "Steering through the event-driven coordinator adds latency and jitter. Let the coordinator grant line control so proposals flow directly; define a stale-target timeout." | `nav_line_drive()` goes **straight to Motion Control** while line control is granted, with no inbox. A mutex shared with the coordinator's owner changes means a stale proposal can never land after the coordinator's own stop or turn. **Stale-target timeout:** Motion Control stops if no target arrives for `NAV_LINE_STALE_MS` = 50 ms (5 cycles); the bench simulation already does. |
| "Two digital sensors give bang-bang steering, and junction detection is undefined: state how a junction differs from a curve." | Sensors straddle the 18 mm line. **Curve:** only ever one sensor on the line at a time. **Junction:** both sensors black at once for ≥ 12 mm of travel. Steering is a hard correction while a sensor sees the line, then a gentle lean back towards where it was last seen, which also catches a sharp curve. Validate at target speed (S3-M04). |
| "NFR-LINE-01/02 lack conditions; 30 s is weak." | Conditions to record in the test log: nominal speed `S3_BASE_MM_S` = 150 mm/s, the three lighting set-ups, the print (full size), the sensor height. |

## Minimum-bit data (Tribunal 1)

| Data | Bits | Who writes it | Why it is safe |
|---|---|---|---|
| task record (state, counters, flags, distances) | bit-fields: state 3, side 2, char 6, result 3, command 3, distances 12 (saturating), counters 8 (wrapping) | the task only | single writer |
| line tracker | one 32-bit word | the task only | single writer |
| barcode edge | **one 32-bit word**: time in bits 31..1, colour in bit 0 | the GP19 interrupt only | word written, then a barrier, then the 8-bit head index; the task only moves the tail |
| edge positions in a read | 16 bits (mm × 16, saturating at 4095 mm) | the task only | single writer |
| coordinator's grant and state | one byte each | the coordinator only | single writer, plus an event-flag bit to wake the task |
| "line detected" for Buddy 5 | one byte | the task only | single writer |
| shared fault bit `SYS_FAULT_LINE_SENSOR` | 1 bit | this task | event flag; set on a stuck sensor, cleared only after the reset (latch rule) |
| timestamps | 32 bits | | team decision |

Telemetry: a 15-byte frame in the team schema (`../system/sys_tlm.h`,
source 3): state, sensors, grant, side, last character, last result and
command, and counters for good, bad and backwards reads, junctions, line
lost and lost edges.

## To agree with the team

1. **Command table** (`S3_BC_COMMANDS` in `s3_config.h`): A → LEFT,
   B → RIGHT, C → STRAIGHT, D → U-turn, from the Project Write-up. The
   write-up calls it an example, and the sample sheet has `*Z*`, which is
   not in it (read and logged, no command). If the final course uses other
   letters, change the table; nothing else changes.
2. **IR polarity** (`S3_IR_BLACK_LEVEL`): HIGH over black on most boards.
   Check with S3-M01 first.
3. **Distance** (IF-18): Buddy 2 provides `s3_motion_get_distance()`, a
   wrapping `int16_t` mm like Buddy 5's. Until then S3 reuses
   `b5_motion_get_estimate()`. **Finer than 1 mm would help the half-size
   print**: with 1 mm steps it decodes at steady speed, but at strongly
   varying speed it may be refused (never misread).
4. **Turn point** (`S3_BC_TURN_DELAY_MM`, 0 now): how far after the
   barcode the coordinator should start the turn.
5. **Stale-target timeout** (IF-19): Motion Control stops when no wheel
   speed arrives for 50 ms.
6. **Template LED**: subsystem 1's template example still blinks an LED
   on GP19, this sensor's pin. Its task makes GP19 an output after
   `s3_init()` has made it an input, so on the car the barcode reader sees
   nothing and the pin drives against the sensor's D0. Subsystem 1's owner
   should remove or move the example before the barcode sensor is fitted.

## Bench use

`SYS_SIM_LINE` in `../system/sys_config.h`:

- **0 (default):** the real Line and Barcode task runs on the real sensors.
  While Motion Control is still simulated (`SYS_SIM_CAR` 1) the wheels do
  not turn: keep the front sensors over the line (or black tape) so the
  line is not "lost", and pass a printed barcode under the side sensor by
  hand at a steady speed. The console prints each state change and every
  barcode result.
- **1:** the bench simulates Line and Barcode instead (the Buddy 5 demo
  with no line under the car). The real task is not started, so S3-U06
  fails in that build.

## Tests

| ID | What | Where |
|---|---|---|
| S3-U01…U07 | Code 39 table, the course `*A*` print, backwards read, half-size `*Z*`, line logic, task running, telemetry | `make TEST=1`, `subsystem3_test.c` |
| S3-U08 | host: 58 checks, incl. all four course barcodes from the PDF, 2000 noisy reads never misread, the reader at varying speed and with a stop on a space mid-barcode | `make -C testing/host` (`test_s3`) |
| S3-U09 | host: the statechart on a simulated track, 55 checks: one event per barcode, all four commands A/B/C/D, sample Z gives no command, stray marks, bad and backwards reads, no distance, ignored events, junction, lost / found, pause, e-stop, stuck sensor and reset, lost edges, every telemetry frame | `make -C testing/host` (`test_s3_task`) |
| S3-M01…M06 | on the car: polarity, NFR-LINE-01, NFR-LINE-02, junction vs curve, steering latency and stale stop, side-sensor position | `testing/subsystem3/TEST_LOG.md` |
