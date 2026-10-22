# Buddy 4 isolated IMU USB test V2

Standalone Pico W SDK diagnostic firmware for the existing GY-511. Added under
`testing/subsystem4/imu_isolated/` on `codex/buddy2-left-motor-test`.
The user confirmed SDA GP2, SCL GP3, 3.3 V and common GND on 22 October 2026.

No production Buddy 4, shared system or working Buddy2/3 files are changed.
This is an isolated hardware check, not the final terrain algorithm or RTOS integration.

## Wiring

| GY-511 label | Robo Pico / Pico W connection |
|---|---|
| VIN | Grove 2 3V3 |
| GND | Common GND |
| SDA | GP2, I2C1 SDA |
| SCL | GP3, I2C1 SCL |

Use the existing wiring, with power off before changing connections. Do not
substitute GP0/GP1 (the encoder wiring). Pin numbers here are GPIO labels, not
physical header positions. Module interrupt pins are not used.
For the pictured GY-511 with separate VIN and 3.3V pins, use **VIN as the
power input**, and leave its 3.3V regulator-output pin unused. Do not connect
both power pins. Follow labels at each end, not assumed wire colours/order.
[Module pinout](https://electropeak.com/learn/interfacing-gy-511-lsm303-3d-compass-and-accelerometer-with-arduino/).

## First test and what to send

1. Hold BOOTSEL while connecting the Pico's own USB socket to the computer.
   Copy `firmware/buddy4_imu_isolated_v2.uf2` to RPI-RP2. It should reboot and
   expose a USB serial COM port. A BOOTSEL drive is not the serial port.
2. Open PuTTY on the new COM port: 115200, 8N1, flow control None. Use computer
   USB power for this test, with the separate power-bank input disconnected.
3. Press `h`. Confirm `BUDDY4 IMU ISOLATED v2`, `motors=OFF always`, and the
   `DEVICE` line. There is no motor start command or button sequence.
4. Press `i` once. Expected 7-bit addresses are `0x19` (acceleration) and `0x1e`
   (magnetic). Expected IDs are acceleration `0x33` and magnetic bytes
   `48,34,33` in hexadecimal, spelling `H43`. `acc_cfg=1 mag_cfg=1` means the
   corresponding identity and configuration-readback checks succeeded.
5. Leave the robot stationary on a flat table for 10 seconds. Then press `p`
   and `h`. Send the DEVICE/SCAN lines, several IMU/RATE lines, and the SUMMARY.
   Check that `h` still responds after at least a minute, as in the USB tests.

The first assessment is connection, fresh gravity readings, rates and I2C errors.
Send that log before drawing conclusions about a hump estimator.

## Expected readings

- `A_valid=1`, `M_valid=1`, advancing `A_n/M_n`, low ages, and no increasing I2C
  errors are the initial communication checks. They do not establish full calibration.
- At rest, `norm` (vector magnitude in g) should be approximately 1. A **rough
  diagnostic band** is 0.85–1.15 g; it is not the project's accuracy acceptance limit.
- If the PCB is flat with its sensor Z axis vertical, Z is approximately +1 or
  -1 g and X/Y approximately zero. A differently mounted module can distribute
  gravity across axes while the magnitude remains near 1. Do not subtract gravity.
- `A_Hz` should be roughly 50 and `M_Hz` roughly 30 after startup. Printing is
  only 5 Hz. Rates are measured successful data-ready reads per elapsed second,
  not an assertion that every hardware conversion was captured.
- `A_overruns` records reported acceleration overrun statuses, not an exact
  count of missed conversions. Diagnostic commands/USB output can introduce gaps;
  leave it untouched for a meaningful baseline.
- `ageA/ageM` are milliseconds since the last successful fresh-data read on the
  Pico, not hardware conversion timestamps. `-1` means no sample was read.
- `mag_uT` uses nominal gain and is uncalibrated. Magnets, motors and chassis
  metal can distort it even with drive off. There is no heading estimate in V1.
- Magnetic overflow reports invalid magnetic data (`M_valid=0`, `nan` values,
  rising `M_overflows`). This can be local magnetic interference or range
  saturation; it is not by itself proof of a dead module.
- Invalid, stale, saturated or absent readings display `NA`, with validity cleared
  and the corresponding last-read age. Raw previously received values are available
  through `p`; a part with no received samples displays `NO_SAMPLE`.

## Follow-up manual checks after the flat log

### Stationary reference and gentle tilt

With the robot resting still, press `c` and leave it untouched for about two
seconds. `CAL: OK` captures 100 fresh acceleration samples as a **RAM-only gravity
reference**. It rejects insufficient samples, I2C errors, clipping, large gaps,
non-gravity mean magnitude or per-axis standard deviation above 0.02 g. These
are preliminary stillness checks and cannot prove there was no motion.

Gently lift the front, lower it, then lift the right side and lower it. Hold
each position still before judging the values. `tilt_ref` should rise away from
the reference and return near zero afterward. `roll/pitch` follow the sensor's
axes, whose mapping to robot forward/right is not yet established. The sensor
measures gravity plus linear acceleration; angles during movement are not pure tilt.

This operation does not correct scale/bias, calibrate a compass, estimate height,
or create a six-face accelerometer calibration. A failed new capture retains any
previous valid reference. Reboot/reconfigure clears it; no flash is written.

### Gravity axis/sign check

If practical without stressing wiring or dragging the mounted sensors, orient
the module/robot so successive sensor axes point up and down. The corresponding
axis should approach +1/-1 g while the others approach zero and norm remains
near 1. Gentle tilt tests are sufficient first; the robot need not be inverted.

### Magnetic response

Turn the stationary robot by hand on the horizontal table and hold it still at
different headings. Magnetic components should generally change. Do not expect
gravity-only tilt to report this yaw rotation. A repeatable, undistorted compass
heading would need separate mounting and magnetic calibration checks, including
motor-on interference later. Do not put a strong magnet beside the module.

## Commands

| Key | Operation |
|---|---|
| `h` | Help and cached identity/configuration status |
| `i` | Read-probe I2C bus scan; cancels an in-progress reference capture |
| `r` | Re-read IDs, configure recognised parts and reset statistics/reference |
| `c` | Capture stationary gravity reference from 100 new samples, 3 s timeout |
| `z` | Clear/cancel reference without changing raw sensor readings |
| `p` | Raw XYZ snapshot and cumulative diagnostic statistics |
| `s` | Pause printing; sampling continues |
| `g` | Resume printing |

No Enter is required. No GP20/GP21 sequence is used. Save logs in PuTTY; this
USB-only diagnostic does not save offline logs. `ACC_STATS` includes all movement
since boot/`r`; only interpret it as noise if that entire interval was stationary.

## Failure interpretation

| Observation | Next check |
|---|---|
| No addresses | Power, common ground, SDA/SCL order, connector contact and GPIO labels; a module power LED does not prove bus communication. |
| SDA/SCL stuck low | Wiring short, missing power, a device holding the bus, or another connection; power off and inspect. |
| Other addresses / unexpected IDs | Send the full scan and ID output; a different chip variant is not automatically a failed IMU. No configuration is written to an unrecognised part. |
| Only one part configures | Diagnose acceleration and magnetometer separately; the other part remains explicitly invalid. |
| Fresh counters stop, ages rise | Data-ready or communication fault; send `p`, `h`, `i`, then try `r`. |
| Gravity near zero or a fixed implausible scale | Check raw values, IDs and configuration; do not calibrate bad data into an apparently good result. |
| `CAL` rejects | Inspect the reason, keep still and check sample freshness, clipping, noise and norm first. |

The I2C bus uses 100 kHz, internal weak pull-ups as a supplement to module
pull-ups, and 2 ms timeouts per transaction phase. A software timeout keeps the
console responsive but does not physically repair a stuck bus.

## Implementation and sources

The supported register layout is LSM303DLHC-compatible. It has an accelerometer
and magnetometer, **no gyroscope**. Acceleration uses 50 Hz, +/-2 g, high-resolution
mode with block update and a six-byte little-endian burst. The magnetic block
uses 30 Hz, +/-1.3 gauss, continuous conversion, big-endian **X,Z,Y** order, with
independent ready/lock/overflow handling. Configuration writes are limited to
known control registers and checked by readback. Compatible IDs do not uniquely
prove the precise chip marking; other variants are deliberately not guessed.

- [ST LSM303DLHC datasheet](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf)
- [ST component driver](https://github.com/STMicroelectronics/stm32-lsm303dlhc)

GP8–GP11 are set to SIO output LOW as the first application action and never
driven high. No PWM, ADC/line following, encoder ISR, servo, network or RTOS task
is started. UART stdio is disabled so GP0/GP1 are not repurposed for a console.

The image excludes the final two 4 KiB flash sectors used by the hump and
junction tests, and there are no application flash erase/program calls. Existing
Buddy2/3 calibration records are retained when this UF2 is flashed normally.

## V2 diagnostic correction after the first hardware log

The first user V1 log found no devices, with `init_errors=2`, no successful
samples and both configuration flags false. Its `nan/inf` values were invalid
placeholders, not measured acceleration or magnetism. Zero sampling-error counts
occurred because unconfigured sensors were never sampled; initialization failed
before that stage.

V2 explicitly reports `NO_CONFIGURED_DEVICE`, ID-read success flags, initialization
errors alongside sampling errors, transaction failure phase/code, and scan
timeout versus NACK/abort counts. Missing/non-finite values are formatted as
literal `NA` without using the Pico floating-point formatter. Sensor register
settings, pins and motor-off behaviour are unchanged.

This diagnostic change does not establish or repair the cause of absent bus
responses. GP2/GP3 indicator lights and idle-high bus lines do not prove that
power and ground reach the module. Check the four physical connections first.
After correcting wiring with power off and reconnecting, use `r`, `i`, `h`, `p`.
If `found=0` persists, a photo and module-side supply-voltage measurement are
more useful than attempting gravity calibration.

See `firmware/BUILD_V2.md` for current artifact identity and software verification.
V1's artifact/build record are retained. Working physical sensor communication
has not yet been demonstrated. See `MEASUREMENTS.md` for the received evidence.
