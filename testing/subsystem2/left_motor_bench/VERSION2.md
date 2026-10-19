# Buddy 2: left motor and integrated encoder bench test

**Only the left motor needs wiring.** This standalone C firmware boots with motor
drive off, displays left encoder readings over USB, and offers short manual motor
pulses. The right motor is held off on GP10/GP11. It never runs automatically.

This is a separate Pico SDK project for the **RP2040 Pico W/WH on Robo Pico**.
It replaces the current firmware when flashed. It does not run the micro T-Kernel
examples, other buddies' tasks, Wi-Fi, PID, navigation or MQTT. Do not add this
`main.c` to `app_program/`; the normal course build is unchanged. After the bench
checks, integrate the verified pin mapping and encoder logic into the motion task
using the kernel's interrupt and synchronization APIs.

## 1. Wiring to check with ALL power disconnected

Use the professor's numbered PH2.0-6P connector diagram, not wire colours or a
guess about which end is pin 1. The loose cable connector can appear mirrored.

| Left motor connector | Function | Robo Pico connection |
|---|---|---|
| 01 | Encoder ground | GND |
| 02 | Encoder channel A | GP0 |
| 03 | Encoder channel B | GP1 |
| 04 | Encoder supply | Verified compatible encoder supply; planned 3V3, see below |
| 05 | Motor winding + | M1A **motor output terminal** |
| 06 | Motor winding - | M1B **motor output terminal** |

The encoder's supply range and output voltage must be confirmed for the actual
motor. This program uses 3.3 V GPIO inputs with weak pull-ups. **Never apply a
5 V encoder output directly to GP0/GP1.** If the encoder requires 5 V, establish
its output type and use appropriate level conversion where needed. A lack of
counts cannot establish whether its supply voltage is correct.

GP8/GP9 already connect internally to the M1 driver: do not wire the motor winding
to the GP8/GP9 header. GPIO numbers are not physical Pico pin numbers.

Keep the right motor disconnected for this test. Keep the robot supported with
**both drive wheels off the table**. Use the motor's rated supply and the Robo
Pico's specified power input. Your 5 V/2.1 A power bank is a supply limit, not proof
that it can meet the motor's startup/stall current. Use one intended board supply
arrangement; do not improvise connections between USB, VIN and 3V3. Avoid the
onboard M1 hardware test buttons while running this firmware; they are not the
GP20/GP21 software buttons.

## 2. Load the firmware

1. Use the supplied `firmware/buddy2_left_motor_bench.uf2` if present, or build below.
2. Disconnect external motor power. Connect a USB **data cable directly to the
   Pico W's micro-USB port**, holding BOOTSEL as you connect.
3. Release BOOTSEL when the `RPI-RP2` drive appears. Copy the UF2 onto that drive.
   The drive disappears and the Pico reboots. This overwrites its existing program.
4. Open the new USB serial COM port with PuTTY (Serial, 115200 baud) or your serial
   monitor. Only one program can open the COM port at a time. Enable terminal
   logging to save evidence. Send `h` to show help again.
5. Confirm the banner says `Buddy 2 LEFT M1 bench v2` and `RIGHT M2 disabled`.

No debug probe is needed. Do not connect the probe's **U/UART** wires to GP0/GP1;
those pins are the encoder inputs. USB serial uses the Pico's USB interface and
does not claim GP0/GP1. A probe's separate SWD debug connection is different.

## 3. Test encoder wiring first

With drive off and the encoder supplied correctly, turn the wheel gently by hand.
Do not force the gearbox. Observe several turns in each direction; one snapshot
of A/B alone is insufficient.

| Serial field | Meaning / expected observation |
|---|---|
| `drive` | `0` off, `1` A-side PWM, `2` B-side PWM |
| `held`, `stop` | 1 when GP21 / GP20 is pressed respectively |
| `A`, `B` | Last observed encoder logic levels, 0 or 1 |
| `A_edges`, `B_edges` | Both should increase when the wheel turns |
| `invalid` | Both channels changed between samples; normally 0 in a slow hand test |
| `net` | Signed quadrature count; opposite rotations should change it in opposite directions |
| `delta` | Change in net count since the previous log line |
| `cps` | Signed quadrature counts per second over the actual logging interval |

The decoder counts all four valid transitions of each A/B cycle (x4). Its positive
sequence is `00 -> 01 -> 11 -> 10 -> 00`; this does not establish chassis-forward.
Counts are not millimetres or wheel revolutions yet. Motor encoder pulses per
revolution, gearbox ratio and effective wheel diameter still need calibration.
Counters saturate instead of wrapping; send `z` while stopped to reset them.

Logs are CSV rows every approximately 250 ms, mixed with help/command messages.
`ms` is time since boot. A/B fields describe the encoder snapshot, not supply
voltage. Example format (illustrative, **not a measured result**):

```text
ms,drive,held,stop,A,B,A_edges,B_edges,invalid,net,delta,cps
2500,0,0,0,1,1,20,20,0,40,8,32
```

## 4. Test M1 with a short pulse

After the encoder check, apply the intended motor power and keep the wheels raised.

1. Release GP20 and GP21 once after boot.
2. Hold the onboard **GP21** button. While holding it, send lowercase `f` over
   USB serial (no Enter required). M1 receives **25% PWM at 10 kHz for 3000 ms**.
3. Releasing GP21, pressing **GP20**, or sending `s` removes drive. After any
   pulse, release GP21 and wait at least one second before another pulse.
4. Repeat with `r` to energise the opposite side. Record the actual wheel direction.
   `f`/`r` are electrical test directions, not a guarantee of chassis-forward/backward.

One button hold permits only one accepted pulse. There is no automatic sequence,
no continuous-run command and no instant direction reversal. A 1 ms interrupt
timer checks the stop/hold buttons and pulse deadline independently of USB logging.
This is a software bench safeguard, not a certified emergency stop: a wedged CPU
or disabled interrupts can defeat it. Disconnect motor power if necessary.
Removing drive does not guarantee the wheel stops rotating immediately.

USB disconnection removes drive when the main loop detects it; physical button
and pulse timeout checks continue in the interrupt timer. A reconnect is not a
start command. Do not press and hold a hardware motor-test button to substitute
for GP21.

## 5. Interpret the result

| Observation | What to check next |
|---|---|
| No USB output | Data-capable cable, correct COM port, correct UF2, serial port not open elsewhere; send `h` |
| Pulse rejected | GP21 held, GP20 released, release/repress GP21 between pulses, one-second rest elapsed |
| Encoder works but wheel does not run | M1 terminal wiring and motor power first; 25% may be below starting torque |
| Motor hums but does not turn | Send `s`/release GP21 and disconnect power; check mechanical binding and supply; do not hold it stalled |
| Wheel runs but both edge counters stay still | Encoder supply/GND, GP0/GP1 routing, connector numbering and voltage compatibility |
| Only one channel changes | Missing/wrong channel connection; do not trust signed position |
| High invalid count or stationary counts | Loose connections, noise, inadequate grounding; ISR overload is also possible |
| Reverse direction changes count sign | Expected; record which sign corresponds to chassis-forward |
| No invalid counts | Encouraging, but does not prove no missed pulses at higher speeds |

Low duty is deliberately a first check; failure to start at 25% does not alone mean
the motor is broken. Do not increase duty until wiring, motor rating and supply are
checked. If needed, change `DUTY_PERCENT` in `main.c` and rebuild deliberately.
This test cannot establish stall current, motor voltage, distance accuracy, or
production control performance. Do not intentionally stall the motor.

### Version 2: checking the stop buttons without rushing

Version 2 increases the maximum run from 0.5 to **3 seconds**, at the same 25%
duty. It also reports the first software stop cause and elapsed run time. Keep
both wheels raised and fingers clear of the wheel and gearbox.

- **GP21 release:** hold GP21, send `f`, then let go after about one second.
  Expect `STOP: GP21 released | run_ms=... | drive=0`.
- **GP20 stop:** release both buttons, wait at least one second, hold GP21 and
  send `f`. After about one second, press GP20 while keeping GP21 held.
  Expect `STOP: GP20 pressed | run_ms=... | drive=0`.
- **Timeout:** release both buttons, wait at least one second, hold GP21 and send
  `f`. Keep holding GP21 and leave GP20 released. Expect
  `STOP: 3000 ms timeout | run_ms=... | drive=0` at about 3000 ms.

For a button-stop check, `run_ms` should be clearly **less than 3000**. If it says
timeout, the pulse ended before the button action was detected. These are
software timestamps, not an external measurement of electrical or mechanical
stop latency. The wheel may coast after drive is removed. If more than one stop
condition occurs on the same timer check, GP20 takes precedence, then GP21, then
timeout; the elapsed time helps identify a boundary case.

After a stop, releasing **GP20 alone** does not rearm the test: also release GP21
before starting again. The existing CSV columns stay unchanged; `STOP:` messages
are additional text lines outside the CSV. Only the latest unreported stop is
retained if the main loop is delayed, rather than an unbounded event queue.

## 6. Build and logic tests

Requires CMake, Ninja, Arm GNU `arm-none-eabi` tools, and official Pico SDK **2.2.0**
with its `lib/tinyusb` submodule. No CYW43/Wi-Fi library is linked.

From the repository root (PowerShell):

```powershell
$env:PICO_SDK_PATH = 'C:/path/to/pico-sdk'
$env:Path = 'C:/path/to/arm-toolchain/bin;' + $env:Path
cmake -S testing/subsystem2/left_motor_bench -B testing/subsystem2/left_motor_bench/build -G Ninja -DPICO_BOARD=pico_w
cmake --build testing/subsystem2/left_motor_bench/build
```

Output: `build/buddy2_left_motor_bench.uf2`. USB stdio is explicitly enabled;
UART stdio is explicitly disabled. Core 1 is never launched. Main/ISR access to
shared control and encoder snapshots uses short interrupt-disabled sections;
`volatile` is not used as a substitute for synchronization.

Host test from this directory, using a native C compiler (not Arm GCC):

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic test_logic.c -o test_logic
./test_logic
```

On Windows use `-o test_logic.exe` and `./test_logic.exe`. These tests exercise
the actual shared control/decoder helpers, including stop, timeout, rearm,
direction reversal decoding and counter limits. They do not simulate the motor,
GPIO electronics, USB, IRQ latency or the PWM hardware.

## 7. Record real evidence

Fill `TEST_RESULTS.md` during testing. Keep the original serial log; distinguish
the host tests and firmware build from real robot results. This firmware is
diagnostic instrumentation, not the final minimum-bit application representation.

## Sources

- [Cytron official Robo Pico DC motor example](https://github.com/CytronTechnologies/Cytron-ROBO-PICO/blob/main/Getting_Started_Guide/CircuitPython/DC_Motor/code.py): M1 GP8/GP9, M2 GP10/GP11, 10 kHz PWM.
- [Raspberry Pi Pico SDK hardware APIs](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html): GPIO, PWM, timers and synchronization.
- Encoder connector numbering: professor's supplied PH2.0-6P wiring diagram; electrical supply/output ratings remain to be confirmed for the actual motors.
