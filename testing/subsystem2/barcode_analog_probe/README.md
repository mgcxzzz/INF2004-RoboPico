# Barcode analogue probe V1

Stationary USB diagnostic for the existing offset IR module. The 141 mm A
produced merged D0 black runs, and narrow white gaps remain D0=1 even while
stationary. This test measures whether A0 retains contrast in those gaps.
It does not decode letters or drive the robot. Existing circuit firmware is unchanged.

## Power off before wiring

For this isolated test, temporarily reuse the known front-left ADC connection.
Disconnect the **front-left sensor's A0 signal from GP26** first. Never connect
two sensor outputs together. Leave its power and the right sensor's wiring alone.

| Offset barcode module | Board connection |
|---|---|
| A0 (analogue output) | GP26 / ADC0, temporarily replacing front-left A0 |
| D0 (digital output) | Keep existing GP19 connection for comparison |
| VCC | 3.3 V |
| GND | Common GND |

GP19 cannot read analogue voltage. Moving D0 to an ADC input does not produce
an analogue measurement: use the module's actual **A0** pin. Verify pin labels
and Grove signal positions, rather than assuming cable colours. Supply the
module at 3.3 V for this direct test; do not apply a 5 V output to the Pico ADC.
This temporary pin assignment is not the final integrated wiring.

## Run

1. Flash `firmware/buddy23_barcode_analog_probe_v1.uf2` using BOOTSEL.
2. Use computer USB, open serial at 115200, 8N1, flow control None. Press `h`
   and verify **BUDDY23 BARCODE ANALOG PROBE v1**. No button can start motors.
3. Keep the sensor at its normal mounted height and angle, with the same
   lighting. Move the whole robot by hand; do not lift the module between tests.
4. Centre the offset sensing point over broad white, hold still, press `w`.
   Wait for its one-second `CAPTURE` result.
5. Repeat on broad black with `b`.
6. On the actual 141 mm barcode, centre the sensing point over the narrowest
   white gap, hold still and press `g`. Then adjacent black bar, press `n`.
   Repeat the gap/bar pair at least three times, including gaps that D0 misses.
7. Send the `CAPTURE` lines with a note about any difficulty positioning the
   sensor. `p` repeats the last capture. Serial text/RAM only: no offline storage.

`ADC_mean`, `min`, `max`, `sd` are raw 12-bit readings (0..4095).
`D0_high_pct` reports the fraction of samples whose GP19 input was high, not
an assumed black percentage. Sampling is nominally 1 kHz, live windows 100 ms;
USB output can reduce the achieved rate, and `n` shows the actual sample count.
No smoothing or black/white threshold is applied to A0. No calibration imported.

A repeatable separation between narrow-gap and adjacent-bar readings, larger
than noise and positioning variation, supports a software threshold/hysteresis
trial. Broad white/black contrast alone is insufficient. Overlapping narrow-gap
and bar readings mean A0 may not resolve this print at the current optical
footprint/height. Analogue output cannot guarantee recovery of missing detail.

Before using Circuit With Barcode V3 again: power off, remove barcode A0 from
GP26, restore front-left A0 to GP26, leave barcode D0 on GP19, then flash V3.
This probe performs no flash erase/program itself; it does not import or modify
the saved circuit calibration/log banks.

## Isolation and verification

Only this standalone executable owns ADC0/GP26 and input GP19 for the test.
GPIO8..11 are configured output LOW before USB initialization and held LOW.
No PWM setup, motor-start path, encoder interrupts, radio or flash writes exist.
Build using Pico SDK 2.2.0, Pico W, C11, Release, `-Wall -Wextra -Werror`.
Build results and artifact hash are recorded in `BUILD_V1.md`.
