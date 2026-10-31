# RTOS Step1 application

`app_main.c` creates the five application tasks and prints USB diagnostics.
`control.c` owns the kernel command buffer, stop state and motor task. USB
commands are only bench target proposals; the unchanged main-branch motion API
and nav coordinator authorize them before the motor task can commit output.
The target provider refreshes every~10ms so the coordinator's100ms stale-input
watchdog remains effective.

`board_port.c` contains the RTOS GPIO/encoder/PWM port and SMP lock adapter.
IO_IRQ_BANK0 is attached on core1. It dispatches both encoders and a GP20 OFF-only
emergency latch. No task-only motion API is called from an IRQ. The motor task
and stop path invalidate stale output through the coordinator generation, and
the final hardware commit also checks the IRQ latch, clock, buttons and USB.

`bringup_policy.h` contains portable start, output and clock-validation rules
used by production code and offline regression tests. No file here reads ADC,
writes flash or modifies the SDK reference's algorithms/calibration.

The default example build excludes this directory. Build `APP=rtos_bringup`
with the exact profile in the [checkpoint guide](../../firmware/migration/micro_tkernel/step_01_motor_bringup/README.md).
This is not the final seven-task course architecture; subsequent checkpoints
introduce the real buddy tasks and kernel-owned network bridge.

Runtime state uses7 flag bits,2 clock-good bits (0..3), and4 stop-reason bits
(0..10) in one packed group. Every task access to this group, including
read-modify-write operations and diagnostic snapshots, uses the same short SMP
lock. No C bitfield layout is sent over USB; the console prints named values.
The IRQ has a separate aligned stop word; the only task that clears it runs on
the same core with interrupts masked. Counter/clock fields retain their full
32-bit ranges; odometry accumulation remains64-bit as in the reference.
