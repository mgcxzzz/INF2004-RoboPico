# Historical SDK Pico platform boundary

This directory documents the standalone Pico SDK hardware adapter. The combined
RTOS build uses the [RTOS board port](../../../../../app_program/rtos_bringup/README.md)
and the [Buddy 5 obstacle adapter](../../../../../app_program/obstacle_navigation/README.md).

`motor_output.c` is the compiled hardware port for motor GPIO8–11. It initializes both channels OFF, retains PWM wrap999 and10kHz timing, writes the requested channel pair and reads signed actual PWM for diagnostics. The application remains the one owner: only its generation-checked commit and stop/safety paths call this private port. Moving the register access here does not add another command source.

Encoder polarity/conversion, ADC pins, watchdog/timebase and two8KiB stack allocation are inherited from the working combined build. They are not retuned. The application still owns encoder interrupts, ADC scheduling and run guards. This directory is an internal platform boundary, not a buddy-facing motion API.
