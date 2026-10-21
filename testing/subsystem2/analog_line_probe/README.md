# Front analog IR calibration probe v1

User confirmed the modules expose A0, D0, GND, VCC and are supplied at 3.3 V.
This standalone Pico SDK program tests actual analog contrast, independently of
the unresolved offline-start issue in the RTOS integration. It is NOT a line
follower, navigation coordinator, or barcode reader. No motor-start command.

## Wire with all power disconnected

| Module | Module pin | Robo Pico/Pico connection |
|---|---|---|
| Left front | A0 (analog output) | GP26 / ADC0 |
| Right front | A0 (analog output) | GP28 / ADC2 |
| Both | VCC | Existing 3.3 V supply |
| Both | GND | Existing common GND |
| Both | D0 | Leave unused |

Move the former digital signal wires at BOTH ends: module D0 -> A0 and board
GP6 -> GP26 (left), GP7 -> GP28 (right). If these pins share Grove connectors,
verify the actual pin labels/cable signal positions; do not assume wire colours.
No modification to GPIO0/1 encoders, motor wires or GP19 barcode sensor.
An ADC pin does not make a digital D0 signal analog: use the module's A0.
Do not supply the sensor at 5 V for this direct ADC test.

## First test: motors OFF, computer USB only

1. Disconnect bank. Flash firmware/buddy23_analog_line_probe_v1.uf2 via BOOTSEL.
2. Open the current COM port in PuTTY, Serial115200, 8N1, flow None.
3. No buttons needed. Repeated ADC lines should continue; h shows help.
4. At normal driving height, put white beneath EACH sensing point for5 seconds.
   Send5-10 lines labelled BOTH WHITE.
5. Put black under each point separately for5 seconds, or use a broad black
   sample beneath both. With20mm tape/35mm spacing, do not force both onto the
   same narrow line. Label LEFT BLACK and RIGHT BLACK (or BOTH BLACK).
6. Return to actual aligned mounting: left spot over black, right over white.
   Slowly slide sideways across the black/white edge. Send the ADC stream and
   describe the movement. Keep height/lighting fixed; leave potentiometers at
   their existing settings during the comparison.

L and R are 32-sample averages of 12-bit ADC results (0..4095). Min/max show
within-burst variation. Higher does not automatically mean black. Millivolts
assume a3300mV reference and are approximate. Flat/railed results need a wiring,
height or sensor check before steering code. This probe does not set thresholds,
normalise blackness or save calibration. User logs provide the calibration.
GPIO8..11 are held low continuously, including before USB initialisation. No
PWM, encoder interrupt, RTOS, radio, flash erase/program or offline run exists.
Use only one power source. No floor-motion claim is made by this diagnostic.

Next: verify contrast/noise and edge response, choose an operating edge/spacing,
then add proportional wheel-speed correction and bounded loss handling. Analog
intensity does not uniquely locate a line that lies outside both sensing spots.

## Build

Pico SDK2.2.0, Arm GCC14.2, CMake/Ninja; PICO_BOARD=pico_w.
UART stdio disabled; USB stdio enabled without waiting for a host on boot.
Compile application with -Wall -Wextra -Werror. Link hardware_adc/pico_stdlib.
