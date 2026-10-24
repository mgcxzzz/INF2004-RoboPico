# Barcode analogue probe V3: GP27 white/black calibration

Stationary USB diagnostic for the existing offset IR module. V2/V3 use the spare
ADC1 input on GP27, so both front line sensor Grove connections remain unchanged.
The current digital setting reportedly detects white gaps and wide black bars
but misses narrow black bars. Measure the actual narrow gap/bar contrast before
changing the driving decoder. This firmware does not decode or drive motors.

## Wiring: disconnect all power first

Move the existing barcode signal wire at BOTH ends:

- Module end: **D0 -> A0** (analogue output).
- Pico W end: **GP19 / physical pin 25 -> GP27 / physical pin 32**.
- Keep barcode VCC at **3V3 / physical pin 36** and its existing common GND.
- Leave both front IR Grove connections unchanged. D0 is unused for V2/V3;
  no extra D0 comparison wire is needed.

GP27 means the GPIO label, **not physical header pin 27**. Verify against the
[official Pico W pinout](https://datasheets.raspberrypi.com/picow/PicoW-A4-Pinout.pdf).
GP19 cannot measure analogue voltage. Merely moving D0 to GP27 still gives a
binary signal; the sensor end must use A0. Supply the module at 3.3 V for this
direct ADC test; never apply a 5 V output to the Pico ADC.

The attached older system diagram labels the front modules D0/GP6/GP7; that is
not the newer A0/GP26/GP28 line-following wiring. Follow the actual working front
wiring and leave it unchanged. GP27 is separate from GP26 and GP28.

## Run

1. Flash `firmware/buddy23_barcode_analog_probe_v3.uf2` using BOOTSEL.
2. Use computer USB and serial at 115200, 8N1, flow control None. Press `h` and
   verify **BUDDY23 BARCODE ANALOG PROBE v3** and **GP27/ADC1**. Motors stay off.
3. Keep the sensor at normal mounted height/angle and the same lighting.
   Move the whole robot by hand, then hold it stationary for each capture.
4. Broad white: press `w`; wait for the one-second CAL/CAPTURE result. This
   sets the white reference and clears the previous black reference.
5. Broad black: press `b`; wait for CAL with `mask=3 valid=1`. White must be
   captured first. Do not use the narrow bar for this broad black reference.
6. Narrowest white gap on the actual 141 mm barcode: press `g`.
7. Adjacent narrow black bar: press `n`. Repeat g/n at least three times,
   including bars/gaps that D0 could not distinguish. Send the CAPTURE lines.

`p` reprints the last result plus calibration; `c` prints calibration;
`r` resets RAM calibration. Each reference capture needs at least 500 samples
in its one-second window. A completed reference uses the measured mean and
standard deviation. A calibration is valid when both references exist, their
mean separation is at least 100 raw ADC counts, and at least six times the
larger reference SD. These are diagnostic checks, not a barcode accuracy claim.

`level_1000` normalizes measured means: white reference=0, black reference=1000,
clipped to that range, regardless of voltage polarity. `-1` means no valid
calibration. A new white capture invalidates the old black reference. Gap/bar
captures measure results without changing either reference. Keep the screw,
height and lighting unchanged throughout calibration and the comparison.

ADC_mean/min/max/sd remain raw 12-bit values (0..4095). Sampling is nominally
1 kHz, live windows 100 ms; USB output can reduce achieved rate. `n` in each
result is the actual sample count. Raw voltage polarity is learned from the
references. There is no black/white decision threshold or barcode decoder yet.

**Calibration is RAM only.** No `k` save; recalibrate after power-off. Save the
serial CAL/CAPTURE lines before switching power. Reopening serial without
power cycling keeps calibration in RAM. No offline drive/log mode exists.

A repeatable narrow-gap/bar separation greater than noise/positioning variation
supports a software threshold/hysteresis trial. Broad white/black contrast alone
is insufficient. Overlap means A0 may not resolve this print with the current
height/optical footprint. Analogue output does not guarantee recovery.

## Return to the digital circuit firmware

Power off, move the barcode wire back **A0 -> D0** and **GP27 -> GP19**, then
flash Circuit With Barcode V3. Do not use V3's unchanged digital code with the
signal on GP27. Both front sensors stay connected throughout this V2 test.

V1 UF2 remains available unchanged, but uses GP26 temporarily and requires
front-left disconnection. V2 also uses GP27 but has no reference calibration. **Use V3 for this test.**

## Isolation

Only this standalone executable owns ADC1/GP27. GPIO8..11 are output LOW before
USB initialization and held LOW. No GP19, front ADC input, motor PWM, encoder
interrupt, radio or flash-write routine is used. Saved circuit calibration/log
banks are not imported or modified. Pico W / SDK 2.2.0 / C11 / Release /
`-Wall -Wextra -Werror`. See BUILD_V3.md for build/UF2 evidence.
