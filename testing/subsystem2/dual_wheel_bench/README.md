# V5: separate power sources and persistent logs

Use `firmware/buddy2_dual_wheel_v5.uf2` for the current build. V4 UF2 is retained
as a historical artifact; the adjacent source now builds V5.

## First V5 test (wheels raised)

1. With the power bank disconnected, flash V5 through the computer USB.
2. Open PuTTY to check the `bench v5` banner, then close PuTTY.
3. Disconnect computer USB. Connect the power bank as the only power source.
4. Wait two seconds. Hold BOTH GP20 and GP21 for THREE seconds.
5. Release BOTH buttons. Wait one second, then briefly press and release GP21.
6. A two-second countdown precedes a single 100 mm encoder-target forward run.
   GP20 or another GP21 press stops it. The three-second run limit still applies.
7. Once wheels stop, wait THREE seconds for settling and flash programming.
8. Disconnect the power bank, then connect computer USB normally (no BOOTSEL).
9. Open PuTTY and press `p`. Expect `RESTORED: CRC-verified log...` and the run CSV.

No automatic movement occurs on power-up, plugging or unplugging. The two-button
arm can be used once per boot. Start another standalone run by restarting with
one power source. The standalone button sequence always selects 100 mm forward;
USB-selected Q/E profiles do not survive power loss. USB hold-to-run f/b tests
and USB D/Q/E/a arming remain available as described below.

Latest completed run is saved automatically after one second of settling,
including USB runs. Each save replaces the previous saved log. No continuous
flash streaming: one sector erase/program per completed run. Avoid unnecessary
repeated saves; flash has finite write endurance. Power interruption while
saving can lose the saved log; CRC and format checks reject damaged records.
Wait three seconds after stopping before removing power. No erase/flash writes
occur while driving. Flash routines run with interrupts masked, core 1 unused,
outputs off; this flash approach must be redesigned before RTOS/multicore use.

One 4 KiB sector at offset 0x1ff000 is reserved on the 2 MiB Pico W. A runtime
firmware-end check refuses storage if the image overlaps it. UF2 contains only
the program image, not saved logs. Treat logs as bench diagnostics, not a general
filesystem or a final minimum-bit application data design.

Firmware and host checks passed; standalone buttons, flash persistence and
power-cycle recovery still need physical verification. Earlier V4 raised tests
confirmed forward/backward direction, timeout, GP20 and GP21-release stops.
They do not validate V5's new persistence paths or floor accuracy.

---
## Previous V4 reference (RAM-only power instructions superseded above)

# Buddy 2 dual-wheel feedback bench v4

**Initial physical validation is still required.** This is standalone Pico SDK
test firmware, not the final micro T-Kernel motion subsystem or minimum-bit
implementation. V3 remains available in `../left_motor_bench/`; this version has
its own `firmware/buddy2_dual_wheel_v4.uf2` to avoid overwriting that known baseline.

## Purpose / hardware

- Closed-loop wheel-speed control on both wheels, using initial PI gains and the
  user's measured direction, circumference and encoder counts.
- Normalised output: positive wheel speed/distance always means chassis-forward.
- M1 GP8/GP9, encoder GP0/GP1; M2 GP10/GP11, encoder GP4/GP5.
- GP20 stop; GP21 hold/start/stop according to the explicitly selected mode.
- 10 kHz PWM, hard cap **25%** on each wheel; 10 ms main-loop controller.
- Initial PI: Kp=0.6 permille/(mm/s), Ki=0.8 permille/(mm/s*s), integral bounded
  to +/-80 permille, anti-windup, 100 mm/s^2 demand ramp, 0.25 speed filter.
  These are initial development settings, NOT physically validated tuning.
- Calibration: L=2524.05, R=2528.30 x4 counts/rev; both circumferences=200 mm;
  track=110 mm. Source observations: `../MEASURED_CALIBRATION_2026-10-20.md`.

Connect with power off. Sensor GPIOs require 3.3 V compatible signals. Use the
intended motor supply/board power wiring; no new electrical rating or current
measurement has been established by software. Do not intentionally stall motors.
Keep fingers clear of wheels. Do not pause the CPU in a debugger during drive:
PWM can continue while software stop handling is paused. No IR/sonar/Wi-Fi or
other buddies' code runs here; this image cannot avoid obstacles.

## Load / initial test: USB connected and wheels raised

Flash this version's UF2 with BOOTSEL and confirm **`DUAL FEEDBACK bench v4`**.
Open Pico USB CDC serial at 115200 8N1, flow control None; enable PuTTY **All
session output** logging. UART stdio is disabled. Do not connect probe UART wires
to encoder GP0/GP1. The physical motors remain off on boot or USB connection.

1. Support both drive wheels off the surface.
2. Release GP20 and GP21. Type `1` for a 60 mm/s timed target.
3. Hold GP21 and send **`f`** once. BOTH wheels should turn chassis-forward.
4. Let it run to the three-second limit, or release GP21 / press GP20 / send `s`
   sooner. If either wheel runs the wrong way, fails to turn or behaves
   unexpectedly, stop and send the log; do not move to floor tests.
5. Release GP21 and wait until **LOG READY**, then send **`p`**. Save everything
   from `RUN:` through `END RUN LOG`, and describe actual wheel directions.
6. Review direction, measured speeds and stops before trying `b` (backward) or
   moving to offline operation. Each hold accepts at most one run; release GP21
   between runs and wait through the settling/rest period.

### Commands differ from v3

| Key | Meaning |
|---|---|
| `f` | BOTH wheels forward at selected target, for at most 3 s; hold GP21 |
| `b` | BOTH wheels backward; hold GP21 |
| `l` / `r` | Timed left/right pivot, each wheel 40 mm/s; hold GP21 |
| `1` / `2` / `3` | Timed straight speed target 60/80/100 mm/s (default 80), not PWM |
| `D` / `Q` / `E` | Select OFFLINE test: 100 mm forward / 45 deg left / 45 deg right |
| `a` | Explicitly arm the selected offline test; requires drive off and GP21 released |
| `s` | Stop drive and cancel arming |
| `p` | Print last buffered run, while stopped |
| `h` | Help |

**`r` now means a right pivot, NOT reverse as in v3.** No L/R single-wheel
selection or encoder-zero keyboard command is needed; each accepted run creates
its own encoder baseline. Position fields are relative to that run.

## Offline tests: no laptop following the robot

First validate wired forward/backward, manual stops, and an OFFLINE D test with
wheels still raised. Then place the robot in clear floor space and externally
measure travel. Keep access to GP20. These tests stop from encoder estimates;
wheel slip and calibration error mean the actual distance/angle can differ.

1. Keep the power bank on so BOTH Robo Pico and Pico remain powered without USB.
2. While stopped, GP21 released and USB connected, type `D` (100 mm forward),
   then `a`. Confirm **OFFLINE ARMED**. This alone does not move the robot.
3. Close PuTTY and unplug the Pico USB cable. Confirm the power bank keeps the
   board on. No movement is triggered by USB removal.
4. Ensure GP21 is released. Tap GP21 and release it. This requests one start
   after a **two-second countdown**, allowing hands to move clear.
5. During countdown, GP20 or another GP21 press cancels it. During a run, GP20
   or another GP21 press removes drive. Three seconds is the maximum run length.
6. Wait until the wheels have stopped and at least one further second for the
   settling log. Keep the power bank on. Reconnect USB WITHOUT BOOTSEL or reset.
7. Open PuTTY at the Pico's current COM port. It automatically prints the latest
   RAM log on connection; send `p` to print it again if needed.
8. Save the log plus ruler-measured chassis travel and any drift. Run Q/E angle
   tests only after D has been checked; use an external angle reference and send
   actual angle as well as encoder-estimated angle.

Arming is one-shot. A held button before USB removal must be released before a
fresh press/release. USB reconnection cancels pending offline starts and removes
drive if an offline run is still active. The countdown has no audible/LED signal;
it starts after the released tap is debounced. A failed start request does not
retry automatically. You must reconnect and explicitly arm another test.

Offline D uses per-wheel 100 mm targets at at most 60 mm/s. Q/E use opposing
wheel targets of about 43.2 mm, at at most 40 mm/s, using the provisional 110 mm
track. A proportional approach profile slows near the endpoint; reaching within
2 mm on each wheel removes drive. There is no automatic reverse correction.
`encoder target reached` means target crossing caused drive off, NOT a validated
physical completion/standstill result for the coordinator. Coast is logged.
If friction prevents reaching a target before 3 s, it times out for review.

## Logs and units

The latest accepted run is stored in RAM, up to 100 rows at about 50 ms spacing,
including one second of post-cutoff settling. A new accepted run overwrites it.
Reset, BOOTSEL, reflashing or power loss ERASES the buffer. Save each important
run before starting another. This is not persistent flash storage or MQTT.

`LIVE` lines are brief live status; `p` prints the complete record:

```text
ms,active,L_mm,R_mm,progress_mm,heading_deg,L_mm_s,R_mm_s,L_PWM_permille,R_PWM_permille
```

- `ms`: elapsed since run start; `active`: 1 drive/controller enabled, 0 stopped.
- L/R mm and speed: normalised signed wheel travel and filtered wheel speed.
- `progress_mm`: (left+right)/2; signed net run progress, not absolute path length.
- `heading_deg`: (left-right)/track in radians converted to degrees; right positive.
- PWM fields: signed command in permille, not percent: +150 = 15% forward,
  -150 = 15% backward on that wheel. Right polarity is inverted at the driver.
- STOP summary uses raw positions latched at drive cutoff, before later coast.
- `rows`/`dropped` report buffer usage. Dump after **LOG READY** for a complete
  settling record. Reconnecting early may print an incomplete settling log;
  wait and send `p` again.

Do not equate encoder estimates with measured floor travel. Logs show initial
controller behaviour; they do not certify accuracy, latency or obstacle safety.

## Safeguards / limits

The nominal 1 ms safety timer checks GP20, GP21 according to mode, 3 s deadline
and a 100 ms stale-control-loop limit independently of USB printing. Controller
also stops for sustained opposite encoder direction, eight invalid transitions
within a run, or no appreciable encoder movement for 500 ms while applying at
least 15% duty with demand ramp at least 30 mm/s. These heuristics need physical
validation; they cannot distinguish a stalled motor from missing feedback or
guarantee against every electrical/software fault. They are not final RTOS safety.
Do not intentionally provoke a stall to test them.

Encoder IRQs have higher priority than the timer. Floating-point controller work
and logging happen in main with interrupts enabled; short IRQ-disabled sections
protect snapshots and prevent a stale controller update re-enabling stopped
outputs. Core 1 is never launched. A CPU hang/disabled IRQs can defeat software
guards. Removing drive is not guaranteed active braking or instant mechanical stop.

## Build and verification

Pico SDK 2.2.0 + TinyUSB, Arm GNU toolchain, CMake/Ninja:

```text
cmake -S testing/subsystem2/dual_wheel_bench -B testing/subsystem2/dual_wheel_bench/build -G Ninja -DPICO_BOARD=pico_w
cmake --build testing/subsystem2/dual_wheel_bench/build
```

Set PICO_SDK_PATH and put compiler on PATH first. From this folder, native tests:

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic -O2 test_dual.c -o test_dual -lm
./test_dual
```

Windows: use `.exe`. Tests cover calibration, polarity, PWM caps, geometry,
hold/timeout guards and the actual offline arming state machine. An independent
first-order motor model checks PI bounds/initial simulated tracking; it does NOT
validate controller tuning, timing, physical wiring, power or floor behaviour.
Bench tests/trace storage use convenient numerical representations: final compact
application fields, reliable coordinator replies, safety latching, odometry sharing
and MQTT packing remain part of the later micro T-Kernel implementation.
