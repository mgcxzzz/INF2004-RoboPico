# Subsystem 4 (IMU Terrain): how to test

Code: `app_program/subsystem4/` (design in its `README.md`). Results go in
`TEST_LOG.md` next to this file, one log file per test ID in `logs/`,
named `YYYY-MM-DD_<TEST-ID>_<what>.txt`.

## 0. Power rule (every test)

- The **Pico W's own USB** may go to the computer.
- The **Robo Pico's power input** goes to the **power bank**.
- **Never both at once**, and never the Robo Pico's power USB into the
  computer.

Every test below runs with the Pico W's USB in the computer only (the
GY-511 needs only 3V3). Anything with the motors turning runs from the power
bank, with no computer.

## 1. What you need

- GY-511 on **Grove 2** (GP2 = SDA, GP3 = SCL), 3V3 and GND.
- Tape measure or ruler, sticky tape for marks on the floor.
- Two flat blocks of known thickness (books are fine), measured with a
  ruler, about 20 mm and 40 mm.
- A test hump: something firm the car can roll over, with a known height
  (for example a book 15-30 mm thick with a sheet of card as a ramp each
  side). Measure its top height with the ruler: that is the reference.
- Three programs:

| Program | Where | For |
|---|---|---|
| `imu_bench.uf2` (bench tool) | `Embedded/bench_tools/` (not in the repo) | calibration, frames, shock, pitch, hand-pushed hump, raw logs |
| test image `mtk3pico_smp1_usb_cdc_wifi_test.uf2` | `cd build_make && make TEST=1 -j8` | on-target tests S4-U01..U06 |
| normal image `mtk3pico_smp1_usb_cdc_wifi.uf2` | `cd build_make && make -j8` | the real task with the coordinator and Safety (S4-U11) |
| host tests | `make -C testing/subsystem4/host` | statechart, hump maths, frame packing (S4-H01) |

Flashing: hold BOOTSEL, plug the Pico W's USB in, drag the `.uf2` onto
the RPI-RP2 drive.

Serial console on the Mac (from the `serial_logs` folder). The bench tool
shows up as `/dev/cu.usbmodem1101` (or similar), the firmware as
`/dev/cu.usbmodemMTK3PICO1`:

```sh
ls /dev/cu.usbmodem*
screen -L /dev/cu.usbmodem1101 115200      # quit: Ctrl-A K Y; log: screenlog.0
```

For the firmware, whose port disappears on reset, use the waiting loop:

```sh
P=/dev/cu.usbmodemMTK3PICO1; until [ -e $P ]; do sleep 0.1; done; cat $P | tee S4-U11.txt
```

Bench tool keys: `?` help, `s` scan, `o` mounting, `c` calibration check,
`n` noise, `f` frame test, `r` raw log, `p` pitch / block test, `k` shock,
`h` hand-pushed hump, `m` magnetometer. Any key stops a running test.

## 2. Calibration (do these first, in this order)

### S4-C00 Wiring and chip
Bench tool, `s`. Expect `device at 0x19 <- LSM303 accelerometer` and
`0x1e <- LSM303 magnetometer`, then `accelerometer OK ... FIFO stream: OK`
and the chip name. Nothing found: check the cable is in Grove 2, power,
SDA / SCL not swapped.

### S4-C01 Mounting (which axis is FORWARD and which is UP)
Mount the GY-511 firmly, flat, and do not move it after this test.
Bench tool `o`: car flat and still, key; then lift the **front** 3-5 cm
(rear wheels down), hold still, key. It prints four lines like
`#define B4_AXIS_FWD (0)`: put them in `app_program/subsystem4/b4_config.h`
(they replace the starting values) and write them in the Calibration
record of `TEST_LOG.md`. Take a photo of the mounting for `logs/`.

### S4-C02 Calibration window
Bench tool `c`, car still on the floor. PASS = the car would calibrate:
|g| 850-1150 mg, >= 90 % on UP, peak-to-peak <= 40 mg. Run it 3 times.
Then nudge the table during one run: it must FAIL (that is the stability
guard working). If a still car fails on peak-to-peak, raise
`B4_CAL_STABLE_MG` a little and record why.

### S4-C03 Noise
Bench tool `n`, 10 s, car still. Note the 50 Hz sd per axis and the pitch
noise it prints. Then again while you push the car slowly along the floor
by hand. The hump start threshold (4.0 deg) should be at least 5 x the
pitch noise. With the motors running (power bank, so over MQTT, see
section 4) check it again before changing `B4_TILT_ON_DD`.

### S4-C04 Shock threshold
Bench tool `k`. Watch "biggest jolt this second" while you (a) push the
car over the test hump, (b) drop the front 1 cm onto the table, (c) bump
it into a wall at walking pace. `B4_SHOCK_MG` (1200) must be above (a) and
below (c). Record the three values; change the threshold if needed.

### S4-C05 Wheelbase and block test
1. Measure the wheelbase: from the drive wheels' axle to where the front
   caster touches the floor, mm. Put it in `B4_WHEELBASE_MM` (it must be a
   multiple of 4) and rebuild the bench tool with
   `cmake -DWHEELBASE_MM=<value> ..` (or ask for a rebuilt `.uf2`).
2. Bench tool `p`: baseline 2 s on the floor, then put the front wheel(s)
   on the 20 mm block, rear on the floor. Read "front height". Then the
   40 mm block. PASS = within max(5 mm, 10 %) of the block (NFR-IMU-02
   bound). A constant error in both means the wheelbase is wrong.

## 3. Unit tests

### S4-H01 Host tests (PC, no Pico)
```sh
make -C testing/subsystem4/host 2>&1 | tee testing/subsystem4/logs/<date>_S4-H01_host_tests.txt
```
PASS = `69 passed, 0 failed`.

### S4-U01..U06 On-target tests (test image)
Flash the test image, car still on the floor, start the waiting loop,
reset. Lines `[TEST] S4-U01 ... PASS` to `S4-U06`:
U01 sensor found and calibrated, U02 250 periods in 5 s with none missed
(NFR-IMU-01), U03 pitch maths, U04 a 20 mm hump through the real
statechart code within NFR-IMU-02, U05 shock flagged in the same period,
U06 telemetry frame round trip. One log for all six:
`<date>_S4-U01-06_target.txt`.

### S4-U07 50 Hz frame test, 60 s (NFR-IMU-01, A4)
Bench tool `f`, let it run 60 s. PASS = 0 late periods, 0 overruns, 0
empty reads, sensor rate about 400 Hz. Note the longest read time (it
should be well under 2 ms of the 20 ms).

### S4-U08 Shock notify time (NFR-IMU-03)
Bench tool `k`. Give 10 sharp taps / bumps. Each line says `flagged N ms
after the sample` and PASS / FAIL against 50 ms. PASS = 10 of 10. In the
firmware the time is measured the same way and shown as
`Safety warned N ms after the sample` (S4-U11).

### S4-U09 Hump pushed by hand (FR-IMU-01/02, rough NFR-IMU-02)
1. Tape two marks on the floor: the first about 20 cm before the hump,
   the second about 40 cm after it. Measure the distance between them.
2. Bench tool `h`, type the distance. Front wheel on the first mark, still:
   it calibrates.
3. Key, push the car at a **steady** walking-slow pace straight over the
   hump, key when the front wheel reaches the second mark.
4. It replays the push through the car's own code (distance spread evenly
   over the time) and prints the height profile and `hump peak: N mm` or
   the reject reason.
5. 5 runs. Compare with the ruler height. Pace by hand is never perfectly
   steady, so this is a rough check; the accurate one is S4-U10.

### S4-U10 Hump on the car (NFR-IMU-02) - needs Buddy 2
Normal image, Buddy 2's motion estimate, car driving the line over the test
hump (power bank, no computer: read the result over MQTT, or reconnect
afterwards). 10 traversals. PASS = every `HUMP N mm` within max(5 mm,
10 %) of the ruler, no false hump on the flat parts.

### S4-U11 Statechart on the real hardware (normal image)
Flash the normal image (optional: set `SYS_CONSOLE_SENSORS` to 1 in
`system/sys_config.h`, local only, to also print the acceleration and
pitch every 0.5 s), start the waiting loop, reset. Car still. Expect, in
order:
1. `[imu] GY-511 found ...`, `[imu] Calibrating`, then `[imu] Idle` after
   about 2 s.
2. GP21 (reset and start): `[nav] Running`, `[imu] Motion Tracking`.
3. Lift the FRONT about 3 cm for a second and put it down, then lift the
   REAR about 3 cm for a second and put it down (a hump, by hand):
   `[imu] motion: climbing`, `[imu] hump start ... (no odometry)`,
   `motion: descending`, then `hump rejected: no odometry` (correct: with
   no Buddy 2 distance the hump is detected but its height is not guessed).
4. Tap the car hard: `[imu] IMPACT ... Safety warned N ms after the sample`
   and `[safety] warning from source 3, code 0x8`. After 0.2 s back to
   `Motion Tracking`.
5. GP20 (e-stop): `[imu] Idle`.
6. Unplug the Grove cable: within 60 ms `[imu] FAULT 0x4`, `[safety]
   warning from source 3, code 0x4`, `shared fault mask 0x04`. Plug it back:
   still Fault (no self-recovery).
7. GP21: the reset sends the coordinator to Ready, the IMU re-initialises
   and calibrates again (`Calibrating`, `Idle`, then `Motion Tracking`),
   mask back to 0x00.

(Safety numbers sources from 0: WiFi 0, Motion 1, Line 2, IMU 3.)

## 4. Logs for assumption A4 (Tribunal 1 feedback)

Bench tool `r` (CSV at 50 Hz; keep the car still for the first second),
one log each, `<date>_A4_<what>.txt`: stationary 30 s; pushed straight
about 3 m; turned on the spot by hand; pushed over the test hump. Then `f`
(missed frames) and `h` / S4-U10 (height against the ruler).

Runs with the motors turning need the power bank, so no USB console
(power rule): log those through the firmware's telemetry over MQTT once
Buddy 1's code is in.

## 5. Integration (later, with Buddy 2)

INT (IMU + Motion Control): drive over the hump at nominal speed; both
log with the shared `sys_time_us()` clock. PASS = the IMU's `HUMP START`
event time is within +-100 ms of Motion Control's logged position at the
hump, and neither task misses a period.
