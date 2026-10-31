# Pico platform boundary

This is the Pico W/RP2040 SDK build. Hardware register access is restricted to the application output/guard adapter, Wi-Fi owner and IMU driver shim. Buddy modules request motion through the public facade. The existing kernel/RTOS project at the repository root remains a separate build.

`motor_output.c` is the compiled hardware port for motor GPIO8–11. It initializes both channels OFF, retains PWM wrap999 and10kHz timing, writes the requested channel pair and reads signed actual PWM for diagnostics. The application remains the one owner: only its generation-checked commit and stop/safety paths call this private port. Moving the register access here does not add another command source.

Encoder polarity/conversion, ADC pins, watchdog/timebase and two8KiB stack allocation are inherited from the working combined build. They are not retuned. The application still owns encoder interrupts, ADC scheduling and run guards. This directory is an internal platform boundary, not a buddy-facing motion API.
