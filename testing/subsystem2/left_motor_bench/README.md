# Buddy 2: single-wheel hardware test v3

Test **one motor at a time**, using the same six-wire cable if needed. Firmware
starts with drive off, LEFT selected, and 25% PWM. Selecting a wheel never starts
movement. The unselected driver is held at zero PWM on both inputs.

The folder and UF2 retain the original `left_motor_bench` name so download links
remain usable. The current CMake source is **`single_wheel_v3.c`**, not `main.c`.
`main.c` and `VERSION2.md` preserve the previous left-only version for reference.
This is standalone Pico SDK firmware: flashing it replaces the Pico program. It
does not run the course micro T-Kernel application, PID, Wi-Fi or other tasks.

## Wiring and power

Disconnect **all power, including USB**, before moving wires or the motor plug.
Follow the professor's PH2.0-6P numbering rather than cable colours or an assumed
connector orientation. Never wire motor power leads directly to GPIO.

| Motor connector | Function | LEFT setup | RIGHT setup |
|---|---|---|---|
| 01 | Encoder ground | GND | GND |
| 02 | Encoder A | GP0 | GP4 |
| 03 | Encoder B | GP1 | GP5 |
| 04 | Encoder supply | Verified encoder supply | Same verified supply |
| 05 | Motor winding + | M1A motor terminal | M2A motor terminal |
| 06 | Motor winding - | M1B motor terminal | M2B motor terminal |

M1 uses GP8/GP9 internally; M2 uses GP10/GP11. GPIO numbers are not physical pin
numbers. Encoder outputs must be compatible with 3.3 V GPIO; never apply 5 V
directly to them. Weak internal pull-ups are enabled. Electrical supply/output
ratings remain to be established from the actual motor documentation. Successful
operation at a bench setting does not establish those ratings.

Keep both wheels raised for powered tests. Use the intended rated motor supply
and Robo Pico power input. The user's supply label is 5 V/2.1 A; that does not
measure motor startup/stall current. Do not improvise connections between USB,
VIN and 3V3. Do not intentionally stall the motor, or use the onboard M1/M2
hardware test buttons while this program controls the driver.

## Load and connect

1. Download `firmware/buddy2_left_motor_bench.uf2` (GitHub: Download raw file).
2. Close PuTTY. Disconnect motor power. Hold BOOTSEL while plugging a USB data
   cable into the **Pico's micro-USB port**. Release BOOTSEL and copy the UF2 to
   `RPI-RP2`. Its disappearance is the normal reboot.
3. Open the Pico's USB serial COM port, 115200, 8 data bits, 1 stop bit, parity
   None, flow control None. The user's laptop previously assigned COM13; check
   Device Manager if that changes. No debug probe is required. Do not connect
   the probe's U/UART port to encoder GP0/GP1.
4. Confirm banner **`Buddy 2 SINGLE WHEEL bench v3`**. Start with `L` then `2`.

## Commands (case sensitive; no Enter needed)

| Key | Action |
|---|---|
| `L` | Select LEFT: M1 and encoder GP0/GP1 |
| `R` | Select RIGHT: M2 and encoder GP4/GP5 |
| `1` | Select 15% PWM |
| `2` | Select 25% PWM (default) |
| `3` | Select 40% PWM; only use after wiring, ratings and supply checks |
| `f` / `r` | Opposite electrical directions; requires GP21 held and GP20 released |
| `s` | Remove drive |
| `z` | Reset selected encoder while stopped |
| `p` | Print a labelled snapshot of selected encoder counts |
| `h` | Show help |

Change wheel/duty only while stopped with GP21 released. Reboot always defaults
to LEFT, even if the cable is on the right: send `R` explicitly for right tests.
Release GP21 once after boot and after every pulse; wait at least one second
between runs. Keep GP21 held during a run; releasing it removes drive. GP20 and
serial `s` also remove drive. Every run has a **3-second maximum**, checked by a
nominal 1 ms interrupt timer. No automatic sequence or two-motor mode exists.
USB disconnection is checked in the main loop; it is not the hard timeout path.
These are software safeguards, not a certified emergency stop. A halted CPU or
disabled interrupts can defeat them. Never pause in a debugger during a run.

## Test order

### A. Finish LEFT measurements before moving the cable

1. Select `L`, `2`. Repeat one run each with `f` and `r` while the wheels are
   raised. Record which command would propel the chassis **forward**, and the
   encoder sign in that direction. Clockwise without a viewing direction is
   ambiguous. Reconfirm stop controls for the new firmware.
2. **Counts per wheel revolution:** leave drive off. Mark the wheel against a
   fixed reference. Send `z`. Turn gently for **exactly five full wheel turns**
   in one direction, stopping at the mark. Do not force the gearbox. Send `p`.
   Record `abs(net)/5` as x4 counts per wheel revolution. Repeat twice in each
   direction, resetting before each trial. Count physical wheel turns, not motor
   shaft turns. `p` itself never drives the motor.
3. **Repeatability:** at 25%, collect three full-timeout runs with `f` and three
   with `r`, releasing GP21 and waiting at least one second between runs.
   Keep the entire logs and annotate the direction. A fixed duty is not a fixed
   speed. Stop if it hums without moving, binds or the supply resets.
4. **Low drive:** select `1` (15%) and check whether each direction starts from
   rest. Return to `2`. If rating and supply checks permit, use `3` (40%) for
   further speed data. No automatic duty increase. Do not infer that a motor is
   faulty solely because it fails to start at low duty.
5. Check no-hold rejection, 3-second timeout, no retrigger on the same hold,
   GP21 release, GP20 stop, and serial `s`. A button stop should report a run
   clearly shorter than 3000 ms. Release GP21 to rearm after every stop.

### B. Move the cable and test RIGHT

Power everything off. Move the motor plug to the physical right motor and route
the board ends as in the RIGHT column above. Check the routing before reconnecting
power. Open PuTTY, send **`R`**, and confirm `CONFIG: wheel=R pwm=25 | drive=0`.
Repeat A1-A5. This checks M2 and GP4/GP5, not just the second motor on M1.

If you keep the board ends on M1/GP0/GP1 instead, keep the firmware in `L` and
label results **physical right motor tested on M1**. This alternative does not
verify the final M2/GP4/GP5 wiring path. Never select `R` and expect M1 to move.

### C. Geometry with a ruler

Record left/right rolling circumference in mm under normal chassis load, ideally
from a straight rollout over several marked wheel revolutions. Also measure the
lateral distance between the centres of the two tyre contact patches. Record
the measurement method and uncertainty. Photos with the ruler help. These
measurements convert ticks into distance and differential wheel travel into an
approximate chassis turn; tyre slip still needs physical calibration later.

Use `HARDWARE_HANDOVER.md` for result tables and previously supplied observations.
Do not call all tests passed until actual results have been recorded. Testing
both motors together under load, closed-loop tuning, distance/turn accuracy,
course behaviour and final RTOS integration remain for later robot access.

## Output

The first **12 CSV columns are unchanged**. Two labels are appended:

```text
ms,drive,held,stop,A,B,A_edges,B_edges,invalid,net,delta,cps,wheel,pwm
```

`wheel` is L or R. `pwm` is the selected duty, including when stopped; use `drive`
to determine whether drive is applied. A/B are latest logic levels; edge totals
count both rising and falling changes. `net` is signed x4 quadrature position;
`delta` is its change since the last line; `cps` is signed counts/s over the
actual interval (about 250 ms). Counts are not mm or RPM before calibration.
Counters saturate instead of wrapping. Invalid transitions can mean noise or
missed transitions; zero invalid transitions does not prove no lost pulses.

Stop text reports reason, elapsed drive time, selected wheel/duty, signed
`ticks_at_cutoff` and `invalid_delta` during the run. Cutoff ticks **exclude later
coasting**; a settled snapshot includes coast. Times are software timestamps,
not measured electrical/mechanical response latency. GP20 has precedence over
GP21 release, then timeout, if detected in the same timer check. Only the latest
unreported stop is retained if the main loop is delayed.

## Build / verification

Requires Pico SDK 2.2.0 plus its TinyUSB submodule, Arm GNU compiler, CMake and
Ninja. From repo root, with `PICO_SDK_PATH` set and toolchain on PATH:

```text
cmake -S testing/subsystem2/left_motor_bench -B testing/subsystem2/left_motor_bench/build -G Ninja -DPICO_BOARD=pico_w
cmake --build testing/subsystem2/left_motor_bench/build
```

UF2 is in `build/`. Rebuild/update the checked-in firmware when source changes.
Run native helper tests from this folder:

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic test_logic.c -o test_logic
./test_logic
```

On Windows use `.exe`. Tests cover actual stop/decoder helpers, not electronics
or PWM routing. IRQ-disabled sections protect main/ISR access; core 1 is not
launched. UART stdio is disabled; USB enabled. No wireless library is linked.
`firmware/BUILD.md` records the current binary and build checks. No production
minimum-bit compliance, PID accuracy or RTOS timing is claimed for this bench code.

Sources: [Cytron motor example](https://github.com/CytronTechnologies/Cytron-ROBO-PICO/blob/main/Getting_Started_Guide/CircuitPython/DC_Motor/code.py),
[Pico SDK hardware APIs](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html),
and the professor's supplied six-pin connector diagram.
