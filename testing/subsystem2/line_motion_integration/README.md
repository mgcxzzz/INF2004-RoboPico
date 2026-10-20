> **Current hardware blocker:** V2 now enumerates and prints initially, but the
> user reports the IR stream freezes later. Do not use frozen values for sensor
> adjustment or start motion yet. Use `sensor_probe/README.md` and the dedicated
> `firmware/buddy23_ir_sensor_probe_v1.uf2` for the next diagnostic. This probe
> has no motor commands and does not run the full integration.

# Buddy 2 + Navigation Coordinator + Buddy 3: isolated line-motion test v2

This folder is separate from `dual_wheel_bench`. Flash
`firmware/buddy23_line_motion_v2.uf2` for this experiment; flash the V7 dual
bench UF2 to return to the turning experiment. Only one program runs at a time.

## V2 startup fix / current status

V1 was reported not to enumerate a COM port, while V6 standalone firmware did.
Source inspection found that returning from usermain calls shutdown_system in
this kernel, which disables interrupts. V2 keeps the initial task asleep on
both success and init failure. A source regression gate in prepare.py checks
this lifecycle. Compile/link passed; USB enumeration on hardware awaits retest.
Do not use the retained V1 UF2 for testing. No sensor/steering changes in V2.

Current S3 code straddles the line: both white can mean centred OR off-track.
It reports LOST after 250 mm without black; the bench limit is only 150 mm.
This can also falsely report LOST on a long centred straight, so this is not
an adequate final line-end detector. Both-black-centred operation is a valid
alternative but requires changed sensor placement, steering and junction rules.
Moving tests remain pending sensor-polarity checks and agreement on that logic.

## What is real

```
GP6 / GP7 -> Buddy 3 real sensor driver + line task (10 ms)
         -> nav_line_drive() in the REAL Navigation Coordinator
         -> mc_set_wheel_speeds() in the new Buddy 2 hardware adapter
         -> PI speed control -> GP8/9 left, GP10/11 right motor driver
GP0/1 left + GP4/5 right encoders -> measured wheel speeds / S3 odometry
```

Real micro T-Kernel tasks and Safety Supervisor run on one RP2040 core.
This is not a mock coordinator, fake odometry or a simulated car.
Pinned buddy3 source: `bc950be50755d3a67cc8de82716c5b1582b56ff1`.
The S3 sources and nav_coordinator.c are compiled unchanged from that commit.
The S2 PI, calibration, output polarity and decoder come from Buddy 2 V7
`deac4940395b6cb9da23cfa29003e95f75a7f0fa`, adapted to the kernel/shared IRQ.
Your branch's top-level app_program and the teammate's branch are not edited.
`prepare.py` creates an ignored build tree from the pinned source and applies
explicit build-only config, USB input and flash-reservation changes.

## FIRST TEST: IR sensors only — NO movement

The user confirmed front sensors on GP6/GP7 but they have never been tested.
Check these before any moving test:

1. Disconnect power bank; flash this UF2 using Pico BOOTSEL and computer USB.
2. Keep both wheels raised. Open PuTTY Serial on the Pico's current COM port,
   115200, 8 data bits, 1 stop, no parity, flow None.
3. Do NOT press onboard buttons or send f. Boot has DRIVE OFF, no autostart.
4. Wait for `BUDDY 2 + NAV + BUDDY 3 LINE integration v2` and repeated IR lines.
5. Present white under both sensors at their normal working height. Record
   `rawL/rawR` and `blackL/blackR`.
6. Put black tape under LEFT only; then RIGHT only; then BOTH. Record each pair.
7. Confirm physical left changes rawL, physical right changes rawR, and return
   to white restores the original readings. Share these four cases first.

The inherited S3 setting assumes raw HIGH/1 means BLACK. Correct expected
raw pairs would be white 0/0, left black 1/0, right black 0/1, both black 1/1.
If your hardware is opposite, do not arm motion: report readings and change
the isolated S3 polarity before proceeding. Stable readings alone do not prove
sensor mounting height/spacing; use the actual floor/tape surface.

## Bounded moving test — only after sensor/direction/stop checks

S3's original proposals are scaled by 50% as a TEST LIMIT, preserving steering
ratios. Centred =75/75 mm/s; left-black=30/95; right-black=95/30;
remembered soft correction is 62/82 (or the reverse) after integer rounding.
This is genuine steering, so wheels need not have equal speeds all the time.
No automatic pivot/line search/barcode manoeuvre is allowed by this adapter.
A bounded command request removes drive and returns unsupported to nav.
Thus losing the line or encountering a barcode may stop this experiment;
this is not the full course-following system.

- PWM cap 25%, same measured circumference/CPR as V7.
- First moving run ends at estimated average travel 150 mm OR 5 s, whichever
  comes first. These are drive cutoff limits, not guaranteed real stopping
  distances. Coasting and floor slip still need external measurements.
- S3 must renew wheel targets within 50 ms. An independent 1 ms hardware alarm
  checks target age, motion-task age, button stops, USB change and deadline.
- Motion task uses PI, encoder direction/invalid checks and 500 ms no-movement
  check. Brief 18% drive pulses (up to 80 ms, 100 ms cooldown, release at
  20 mm/s) help low-speed forward steering wheels break away. Untuned on floor.
- Either button stops an OFFLINE run; USB hold test stops on GP21 release.
  s via USB stops. Safety inhibit removes outputs and rejects targets.
- No obstacle/ultrasonic guard, Wi-Fi, IMU or MQTT is started. Use clear space.
- Stop acknowledgement to nav is deferred until encoder speed is below 5 mm/s
  on both wheels and 100 ms has elapsed; this is estimated standstill.

### Raised USB steering/stop check (second stage)

Hold GP21, send lowercase f. Move black/white under the front sensors and check
that left-black slows the left wheel and right-black slows the right wheel.
Release GP21 or press GP20 to stop. One run per boot, max 5 s /150 mm estimated
wheel travel. This is a later stage, NOT the first sensor-only test.

### Offline floor procedure (later, after raised checks)

1. Computer USB disconnected. Power bank only; wait until startup completes.
2. Hold BOTH buttons 3 s, release BOTH, wait 1 s.
3. Tap/release GP21. A 2 s countdown precedes the one bounded line run.
4. Either button stops/cancels. No GP20 profile-selection taps in this program.
5. Wait 3 s after the wheels stop so the log settles and saves to flash.
6. Disconnect bank, connect computer normally (no BOOTSEL), open PuTTY, press p.

Never overlap power sources. One accepted run per boot. Each completed run
replaces the previous integration log in the same last 4 KiB flash sector used
by standalone bench firmware. Different magic/version rejects old-format logs.
Power loss during save can lose the record; wait before disconnecting. Finite
flash endurance: one erase/program per run, no continuous logging.

## Logs

IR lines while idle are live, read-only diagnostics. Buffered run CSV includes
raw and interpreted IR bits, nav state/owner, S3 state, wheel distance/speed,
S3 requests, scaled targets and actual PWM. `mm10` / `speed10` are tenths of mm
/ tenths of mm/s. Other targets/requests are mm/s; PWM is permille.
Nav Running=1, owner line=1; S3 Follow=2, Read=3, Await=4, Fault=5.
`accepted` counts forwarded motor-target calls, not distance or completed moves.
`rows/dropped` measure trace capacity; `usb_tx_dropped` measures console loss
during dump. Both should be zero losses. p is repeatable while stopped.

S3 receives a continuous wrapping signed int16 millimetre odometer from real
encoders, not a per-command reset counter or the nominal fallback. The adapter
protects shared target pairs and ISR snapshots with short interrupt-masked
sections on this single core. It is a test adapter, not the final minimum-bit
S2 status, all motion APIs, safety-certified controller or RTOS timing proof.
Physical sensor polarity, IR decisions, GPIO IRQ coexistence, motor response,
stale stops and flash recovery still require testing on the robot.

## Rebuild

Requires Git, Python3, GNU Make, Arm GCC, host C++, and SDK 2.2.0 TinyUSB.
The two pinned source commits must exist locally (fetch buddy3 and the Buddy 2
branch first if necessary). From repository root:

```sh
python testing/subsystem2/line_motion_integration/prepare.py
export PICO_SDK_PATH=/path/without/spaces/pico-sdk
make -C testing/subsystem2/line_motion_integration/build/build_make -j8 \
  SMP=0 CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 CRASH_REPORT=0
```

Do not change SMP=0: flash persistence and adapter critical sections depend on
one core. `CRASH_REPORT=0` keeps UART off GP0/GP1. USB RX is added only to the
copied build's existing USB service task; TinyUSB has a single owner. The flash
writer runs from SRAM using RP2040 ROM routines and an SRAM boot2 copy, with
interrupts masked and both motor outputs off. Linker reserves sector 0x1ff000.

Host checks: see firmware/BUILD.md. Compiled success is not hardware validation.
