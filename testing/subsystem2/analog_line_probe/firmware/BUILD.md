# Analog line probe v1 build

21 October 2026. Standalone motor-disabled ADC diagnostic, hardware validation pending.

- Pico W/RP2040, Pico SDK2.2.0, Arm GCC14.2.1, Release, CMake4.4.4/Ninja.
- Strict application build: -Wall -Wextra -Werror, PASS.
- GPIO26 -> ADC0 left, GPIO28 -> ADC2 right; adc_gpio_init configures analog pins.
- First conversion after each channel switch discarded, then32 samples averaged.
- GPIO8..11 explicitly driven low before ADC/USB init and throughout the loop.
- No application PWM/motion, GP19 capture, RTOS, UART stdio or flash-log writes.
- ADC values/min/max and approximate millivolts printed every200 ms over USB.
- Expected input range0..3.3 V; sensor power was confirmed3.3 V by the user.
- UF2 length68096 bytes. SDK warnings about absent wireless submodules are
  irrelevant to this USB/ADC-only build; no radio library is linked.

SHA256 buddy23_analog_line_probe_v1.uf2:

```
60581b1837acb39675fb5e0dccfcdb524df7a8560be13c2558a030b6770ac4ea
```

Measurements, black/white polarity, usable contrast and noise must be checked
on the robot before any line-following firmware is derived from these readings.
