# Step 1: RTOS motor, encoder and stop bring-up

**EXPERIMENTAL — hardware acceptance pending. This is not line-following firmware.**

Download `firmware/robopico_rtos_motor_bringup_v0_1.uf2` or the complete bundle in
the same folder. This checkpoint follows the motor-first migration sequence.
Wi-Fi/MQTT/dashboard start at Step3; line/barcode/IMU and Buddy5 tasks are not
running in this image. Existing saved calibration and network sectors are not
read or written. The working main-branch UF2 remains the rollback image.

## What actually runs

Real micro T-Kernel SMP tasks, on top of the repository's Pico BSP and SDK
TinyUSB sources, use a 1ms kernel tick. Hardware TIMER is established at 1MHz
before USB/tasks start, including on a warm BOOTSEL boot.
The template sample ADC/UART device drivers are disabled for this profile:
UART0 must not claim encoder GP0/GP1. USB CDC remains available.

| Task | Priority; core | Work |
|---|---|---|
| Kernel USB service | 3; core0 | Own TinyUSB; copy incoming bytes into a bounded ring and drain nonblocking output. |
| Safety Supervisor | 6; core1 | Poll stop/USB/deadline/control-liveness conditions; call the motion stop API. |
| Bench Navigation Coordinator | 10; core1 | Consume copied command messages; authorize bounded fixed wheel targets through the existing shared API. |
| USB command task | 12; core0 | Read copied input, submit a five-byte timestamped kernel message; urgent stop survives a full command queue. |
| Motion Control | 14; core1 | Own encoder IRQ/draining and PWM calculations. Run the unchanged ContinuousDrive at an absolute 10000us schedule, with measured dt. |
| Diagnostics | 18; core0 | Once per kernel second, report both clocks, control timing and motor/encoder status. |

`motion_api.c`, `nav_coordinator.c`, `nav_codec.c`, `motion_executor.c` and all
ContinuousDrive headers compile directly from the unchanged working integration.
The bench deliberately substitutes fixed Buddy3 targets; it does not claim to
be sensing a line. Requests still cross the compact API boundary, ownership,
generation checks and coordinator stale-target watchdog. Normal PWM is written
only by Motion Control. GP20 has an immediate OFF-only IRQ path; its latch is
checked inside the final PWM commit and cleared only explicitly while stopped.

Wiring and units match main: PWM GP8/9 left, GP10/11 right; 10kHz, wrap999,
420 permille ceiling; encoder GP0/1 left and GP4/5 right. Chassis-forward raw
encoder sign is negative left, positive right. Wheel circumference200mm,
left CPR2524.05 and right CPR2528.30. The speed filter and controller gains,
slew, one-shot start assistance and physical output limits are unchanged.

## Test in this order

1. Lift the driven wheels clear of the mat. Flash the new UF2 through BOOTSEL
   as usual. Reconnect normal USB and enable the robot's motor power as for
   earlier bench tests. Motors must remain stationary at boot.
2. Close PuTTY. Install pyserial if needed: `py -3 -m pip install pyserial`.
   Double-click **Run timing check.cmd** next to this README, or run
   `py -3 tools/rtos_migration/capture_bringup.py` from the bundle root.
   Enter the Pico COM port (`COM4` or its actual port). This measures40s after
   allowing2s for buffered startup output to drain, and sends
   **no robot commands**. It creates raw text, CSV and a JSON summary under
   `rtos_test_session`. Wait for completion and require `result: PASS` before
   trying a powered wheel test. Send all three files for review, even if it fails.
3. Open PuTTY on the same port,115200. Press `h`, then `p`. Three consecutive
   one-second clock comparisons are required for firmware `CLOCK ... status=PASS`.
   This internal PASS compares the two robot clocks; the laptop timing check is
   needed as well. Reports continue if validation fails, with driving inhibited.
4. While stopped, turn each wheel manually in the chassis-forward direction.
   Press `p`: each encoder's edge count should increase, invalid transitions
   should stay low, left raw ticks should decrease and right raw ticks increase.
5. Release both buttons. Press **1 once**: left wheel forward only, target45mm/s,
   automatic stop after3s. Wait at least1s. Press **2 once** for right only, then
   **3 once** for both. These are short bench pulses, not a circuit start.
6. During separate pulses, verify `s` stops, GP21 stops, and GP20 stops and
   latches. A latched stop must refuse another pulse until buttons are released
   and `x` is pressed; wait1s after reset. `e` is the USB latched emergency stop.
   Closing PuTTY/dropping USB DTR must also stop a running pulse; no automatic
   restart is allowed. Reopen the port and request a new pulse explicitly.
7. In PuTTY, enable Session→Logging→All session output **before** the motor
   tests. After testing press `p`, `n` and `z`, then send that log, the timing
   capture files and a short wheel-test video. We will review before Step2.

Do not run this image on the circuit: it has no optical navigation. Wi-Fi
configuration and the dashboard are unnecessary for this checkpoint. Reflashing
the reference UF2 restores the previous application, without changing its saved
calibration/settings. Do not use flash-nuke for this test.

## USB keys

| Key | Meaning |
|---|---|
| `1`, `2`, `3` | One left, right or both-wheel forward pulse;45mm/s, at most3s. Press once. |
| `s` | Normal stop. |
| `e` | Latched emergency stop. |
| `x` | Clear stop/fault latch only while stopped, clock valid and both buttons released. |
| `p`, `n` | Status and raw clock registers. |
| `z` | Up to12 queued coordinator events, with owner/requester/result. |
| `h`, `?` | Help. |

Calibration keys `w/l/r/k/c` have no calibration implementation in Step1.
The normal circuit startup sequence and old profile-selection keys do not apply.
This image does not preload or replace any front-sensor calibration.

## Reading failures

`CLOCK CHECK/PENDING` means the gate is not yet satisfied. Repeated CHECK,
stopped counters, or laptop capture FAIL must be investigated; do not adjust
controller gains or increase PWM to hide a timing problem. `flags` packs active,
E-stop, clock-valid, ready, stall, late-control and encoder-init state. `reason`:
0boot,1pulse complete,2USB command,3GP20,4GP21,5USB lost,6clock mismatch,
7control gap,8encoder progress stalled,9encoder,10API failure.

`systick_reload` is core-local: core0 is the kernel timekeeper and should show
124999 for125MHz/1ms; core1 SysTick is intentionally disabled. `CLOCK_REG core=0`
is reported automatically every5s. Watchdog tick must have cycles12, ENABLE
and RUNNING set (`tick & 0x7ff == 0x60c`); the high count bits vary normally.
Both clocks should advance about1000ms/1000000us per wall second, with roughly
100 controller updates per wall second. The host script allows3% endpoint
uncertainty and flags new control gaps over15ms or USB drops.

## Build and evidence

From Git Bash at the repository/bundle root:

```bash
bash tools/rtos_migration/build_motor_bringup.sh
```

Prepared tools are in the workspace's `.buddy2-tools`; use an equivalent
Arm GCC/Pico SDK environment and the documented make profile elsewhere:

```text
make -j8 APP=rtos_bringup SMP=1 CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 CRASH_REPORT=0 TEST=0
```

Run offline checks with `py -3 tools/rtos_migration/run_tests.py`.
See the firmware manifest, source hashes and `logs` for software verification.
Host tests do not establish hardware scheduling, physical stop latency or motion
quality; the tests above are the acceptance gate.
