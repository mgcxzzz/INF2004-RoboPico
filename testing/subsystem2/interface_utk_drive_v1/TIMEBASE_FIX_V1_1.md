# V1.1 hardware timer correction

## Evidence and scope

The user's teammate tested the unchanged e7d78e3 V1 bundle. Network polling,
MQTT publication, subscription and source-9/source-10 frames continued, but
controller uptime advanced only 1153 ms over 49.217 wall seconds. The ratio
is about 42.686. Motion arrivals were about 2.140 seconds apart although
scheduled for 50000 us; one-second heartbeats arrived about 42.7 seconds
apart. The CPU/register state was not captured. The hypothesis that an
inherited zero watchdog divider acts as 512 cycles fits 512/12=42.6667,
but the prior register value is not hardware-confirmed.

The confirmed code defect was `sys_hal_init` accepting RUNNING without
checking the divider. SDK register definitions show that ENABLE and CYCLES
are separate fields. Merely dividing displayed timestamps or increasing the
heartbeat frequency would leave control, ADC, speed estimation and safety
deadlines wrong, so V1.1 corrects the shared hardware clock at its source.

## Initialization and start gate

- After selecting the 12 MHz XOSC reference, `knl_startup_hw` unconditionally
  establishes ENABLE plus CYCLES=12 and verifies the readback. TIMER is also
  reset/released there, before USB/Wi-Fi services or application tasks run.
  This pre-C-runtime code uses only MMIO, flash constants and stack locals.
- `sys_hal_init` repeats the configuration/readback check and validates
  clk_ref source/divider. A failed write, non-running tick or unexpected
  reference configuration returns an error rather than accepting it.
- With motor pins low, `sys_timebase_validate` compares a 250 ms kernel task
  delay against the hardware TIMER. The kernel SysTick selects CPU clock
  (`SYST_CSR=0x7` in `sys_timer.h`), so this is independent of the watchdog
  tick divider. The allowed endpoint error is 2 ms plus 2% of elapsed time.
- No drive task is created until the check passes. On failure, diagnostics
  repeat using kernel delays, with driving disabled.
- PuTTY `n` repeats the recorded timing result and current clock registers.
  It performs no motor commands and does not repeat/reconfigure the clock.

Expected status is `TIMEBASE: PASS cycles=12 enable=1 running=1`, with
`ref_selected=0x4`, `ref_div=0x100`, roughly 250 `kernel_ms` and roughly
250000 `timer_us`. Full tick-register count bits are not compared as a
constant because they legitimately change while the generator runs.

## Preserved behaviour and update path

The three Continuous V2 steering/drive headers, gains, motor caps, calibration
preset, record27 persistence, restore key `c`, nav interfaces, MQTT topics,
binary telemetry layouts and dashboard remain unchanged. Existing V1 laptop
files are compatible. Only the new `buddy123_utk_drive_v1_1.uf2` is required
on the robot. Wi-Fi settings and saved calibration/log flash regions are
outside the firmware image and are preserved. Barcode mapping is RAM state
and must be reapplied after reboot as before.

The old V1 artifacts remain for comparison; use the explicitly named V1.1
image/bundle for the timing retest.

## Verification

- `test_timebase.c` executes production `sys_hal_init`/validation against
  modeled MMIO/kernel calls. It covers inherited RUNNING with wrong divider,
  correct/disabled ticks, refused writes, missing RUNNING, wrong reference,
  slow/frozen/fast counters, elapsed-counter wraparound and endpoint allowance.
- `test_timebase_gate.c` includes the actual application startup and checks
  that configuration/clock-check failures cannot create/start the drive task.
- Both strict host regressions pass. ARM firmware builds. Compiler stack
  frames for the added report/validation are 48/32 bytes; usermain is 48 bytes
  and tm_printf/tm_vsprintf are 32/96 bytes before deeper callees. These are
  compiler frame figures, not a complete runtime stack measurement.
- The actual RP2040 clock readback, heartbeat rate and motion cadence need
  the friend's hardware retest. No hardware timing result is claimed here.

Host commands from this folder (GCC and Python available):

```text
gcc -std=c11 -O2 -Wall -Wextra -Werror -Itest_timebase_stubs -Isource/include -include test_timebase_port.h test_timebase.c source/app_program/system/sys_hal.c -o build/test_timebase.exe
gcc -std=c11 -O2 -Wall -Wextra -Werror -Itest_timebase_stubs test_timebase_gate.c -o build/test_timebase_gate.exe
```

Then run both executables. The host tests do not validate the radio or actual
kernel scheduling. The first physical retest should be idle, without Start:
use `n`, then compare controller uptime with wall time and observe roughly
one heartbeat per second and motion frames about every 50 ms. After that
passes, check the two-second Start countdown and normal profile1 driving.
