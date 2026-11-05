# RTOS line-navigation boundary (Step 2)

Main release APP=rtos_buddy1_4 enables the tested Buddy1–4 adapters plus
network-supervised powerbank operation and stopped wireless exports. See
[main setup](../../docs/setup.md) and [architecture](../../docs/architecture.md).
The checkpoint history below describes the port and USB-supervised test profiles.

This folder ports the SDK integration's profile 1 execution boundary. It does
not replace the reference tracker, Continuous V2 steering, PI controllers,
turn geometry or line-search algorithms. The reference feature source,
interfaces and calibration configuration retain the main baseline
`7843f45fbe45d5a774bc18c93be9bb11084d1b64`, with two explicitly reviewed opt-in
departure-guard edits in junction_logic.h/uturn_logic.h. Packaging checks every
other shared source byte and exactly reconstructs these two permitted changes.

Current Step 2 version is **v0.1.2**, the angle-only junction experiment. See
`firmware/migration/micro_tkernel/step_02_line_following/UPDATE_V0_1_2.md`.
APP=rtos_line and the Step3B APP=rtos_line_wifi define JUNCTION_ANGLE_GATE_ONLY=1; default0 retains the old
white prerequisite for reference builds. Original steering/calibration, branch
threshold/dwell, angle limits, acquisition and stops remain. White is separately
observed, never used to manufacture a sensor reading. The ADC correction from
v0.1.1 is retained.

Step3B reuses this owner without retuning. It adds a locked copied live-motion
snapshot, qualified Start acceptance and ordinary tracking reacquisition after
a coordinator-approved Pause/Resume. The full-rate stored log remains USB d.
See [the connected checkpoint](../../firmware/migration/micro_tkernel/step_03_connected_line/README.md).

## Ownership and update order

| Owner | Core / priority | Work |
|---|---|---|
| Kernel USB | 0 / 3 | TinyUSB; copies incoming bytes into a bounded ring. |
| Flash parking worker | 0 / 5 | Sleeps; acknowledges a stopped flash transaction from RAM. |
| Console command task | 0 / 12 | Copies timestamped input to the existing command message buffer. |
| Diagnostics | 0 / 18 | Copied one-second status; exports a stopped frozen RAM log. |
| Safety supervisor | 1 / 6 | 1 ms stop checks; can only reduce/disable already committed outputs. |
| Navigation command task | 1 / 10 | Command age/acceptance, stops, route API requests; queues sensor/calibration actions. |
| Control task | 1 / 14 | 10 ms encoder/IR/line/navigation/power sequence; sole normal PWM writer. |

Lower priority numbers run first. The actual micro T-Kernel scheduler is used;
this is not an SDK loop labelled as RTOS. Sensor and controller operations stay
together on core 1 for this stage, preserving their order without stale
inter-task sensor snapshots. Future task splitting requires a separate timing
and data-age check, not just moving function calls into new tasks.

The controller drains encoder deltas, updates speed, checks timing, reads each
front ADC (discard one conversion, average 32 with 20 us spacing), updates the
0.35 filter, runs the original tracker/junction state machine and V2 steering,
submits Buddy 3 targets through `motion_submit_navigation`, calls `motion_update`,
selects the original power path and commits by coordinator generation.

Side ADC uses discard-one/average-four reads on a nominal 2 ms schedule in the
same owner. Between encoder drains, side-edge wheel positions are interpolated
using the last measured velocity. This is a port difference from SDK GPIO
encoder peeks; side sampling gaps/edge placement require a physical cross test.
No catch-up burst of PWM is issued after a delayed control cycle.

The portable `adc_reader.h` rechecks READY at the original 100 us deadline;
preemption cannot turn a completed conversion into a false timeout. Genuine
timeout/ERR failures retain first/last channel, stage, sample, CS, elapsed and
clock/reset snapshots. Only the control owner touches the ADC; the diagnostic
task sees copies under the existing snapshot lock. A failed front pair is not
partially published. A runtime failure calls the shared stop path with reason11
and an emergency latch. Stopped-only reset/retry is bounded to 100 ms spacing;
three consecutive complete all-channel frames are required before ADC readiness
returns. Recovery does not clear the emergency latch or restart motion.

`line_model.h` is hardware-free. Its profile 1 wrapper omits allocation of the
63 inactive barcode banks. The parity test feeds the same front samples,
velocity, scan activity and elapsed time through it and the unmodified full
`continuous_follow_step`, comparing targets, decisions, levels, phases and
steering state. Bare-cross qualification is separate from decoder validation:
no physical decode can be claimed in Step 2.

## Motor and safety boundary

Ordinary drive uses the unchanged ContinuousDrive 420 physical-PWM cap and
single bounded startup aid. Capture/load correction, rolling turn, U-turn and
active recovery use the original respective controllers and output envelopes.
The output callback converts signed/native demands into mutually exclusive
H-bridge A/B channel levels. Signed reverse is allowed only by the reference
U-turn path. Board writes reject two simultaneously powered channels.

A committed immutable envelope is protected by the same motion lock as the
generation check. The 1 ms supervisor can expire a peak without accessing the
control task's mutable PI state. GP20 IRQ latches emergency stop and writes OFF;
every subsequent commit checks the latch. GP21, `s`, USB loss, invalid encoder
transitions, sustained wrong direction and timing faults stop motion. This
stage retains Step 1's stricter 50 ms active scheduling-gap guard; the reference
controller tuning itself is unchanged. No-progress is logged through encoder
and speed fields; Step 1's fixed-pulse stall timer is not imposed on line motion.

USB STOP uses an urgent copied flag as well as the queue, so filling a command
buffer cannot remove the supervisor's stop request. A separate cancellation
generation invalidates the hands-free countdown, and timestamped queued starts
from before a stop are rejected. Export also cancels any pending countdown.
Idle timing gaps invalidate
clock readiness and require three new good windows; active gaps latch OFF.

## Calibration and flash

`rl_boot_cal` validates: own RTOS page, SDK record 28, original fallback, in that
priority order. SDK CRC/ABI validation uses the same record headers with
`BUDDY_WIFI_LOGGING=1`; no Wi-Fi code is linked by that layout definition.
Calibration actions run in the control owner and are rejected during motion.
Only it can start a line session, so a save cannot race a new motor start.

`k` writes one 256-byte versioned little-endian CRC page in its own 4 KiB sector
at `0x1a9000`. Core 0 parks in RAM with interrupts masked; core 1 uses RP2040 ROM
flash operations and a RAM copy of boot2, restoring XIP before releasing core 0.
The writer also masks its interrupts while XIP is unavailable. Handshake timeout
fails before erase. `.rtos_ramfunc` is explicitly linked into copied `.data`;
packaging verifies the linked RAM addresses and disassembly call boundaries.
No task sleeps or formats text during the flash lockout. Writes require motors
stopped, released buttons and no IRQ emergency latch. Read-back must match.

## Diagnostic representation and sharing

The bounded 1,536 x 56-byte RAM ring stores the final ~15.36 seconds. It is
frozen on stop with a terminal sample and overwritten only by the next accepted
start. Packed words use explicit masks/shifts, and all cross-core snapshots use
short spinlocks. The producer owns planner/PI state; the diagnostic task never
reads those structures directly. USB export applies backpressure and never
blocks motion while a run is active.

| Field | Representation |
|---|---|
| Three raw ADC and two filtered ADC readings | 12 bits each in packed words. |
| Two normalized optical levels | 10 bits each, 0–1000. |
| Steering gain/damping quantized to 0–1000 | 10 bits each. |
| Junction white-now/white-qualified/gate/accepted/without-white/event | One bit each in spare aux bits; copied under the same snapshot lock. |
| Encoder turn angle x10 | Signed13 bits, saturating -4096..4095; remaining spare bit unused. |
| Navigation phase | 4 bits, values 1–10. |
| Optical decision/reason | 6 bits each, preserving existing code ranges. |
| Side black, busy, white, active-search | One bit each. |
| Pending/executing route | 2 bits each: none/A/F/Z; rendered as the reference ASCII value on export. |
| Supervisor flags / reason | 7 / 4 bits. |
| ADC flash calibration | Four packed 12-bit references; page is padded to flash programming granularity. |
| Timestamp, sequence, cumulative encoder counts | Fixed 32-bit fields; counts use signed saturation. |
| Wheel speed x10, targets, physical signed PWM | Signed 16-bit fields; preserve current controller units. |
| Control interval/cost | Saturating unsigned 16-bit microseconds. |

This is not a claim that the entire legacy application has completed the
course's minimum-bit audit. That wider audit must also cover IMU, obstacle,
kernel message buffers and eventual MQTT payloads as those stages migrate.

Known hardware acceptance questions: current calibration contrast on the tilted
front sensors, exact 100 Hz cadence/cost on the car, sharpest bend tracking,
side-cross qualification and outgoing-line reacquisition after each turn.
