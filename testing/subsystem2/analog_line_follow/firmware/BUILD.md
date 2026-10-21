# Analog line-follow v1,21 October2026

Standalone Pico W/RP2040 SDK2.2.0 test; NOT the RTOS Nav/S3 integration.
Arm GCC14.2.1, CMake4.4.4/Ninja, Release. UART off; USB stdio on.
Application -Wall -Wextra -Werror: PASS. ELF text54216/data0/BSS12028.
UF2 length100352 bytes, all RP2040 blocks below reserved flash log sector.

Host test_line.c PASS: per-sensor calibration, valid ADC endpoints and minimum
span, normalized saturation/midpoint, edge steering sign, right assistance,
white-recovery distance/time bounds, contact reset, transient/sustained both-dark,
target bounds across52056 ADC pairs, GP21 release/GP20/late-control guard,
deliberate hold/release/tap/countdown offline sequence.
Static source gates checked: motor zero before operation, calibration/one-run
requirement, correct ADC GPIO26/28, enumerated host blocks offline start, active
and generation recheck at PWM commit. UF2 family/headers/addresses verified.

These are software checks, NOT hardware validation. Physical new analog steering,
calibration persistence, bank-only boot, stopped-wheel behaviour, recovery and
curve tracking must still be checked. No successful analog motion claimed.

Motion helpers included from ../dual_wheel_bench/dual_logic.h, encoder helpers
from ../left_motor_bench/bench_logic.h and retryable arm from
../line_motion_integration/logic.h. Those tracked files were not changed.
Source motion/flash record handling derives from the existing V7 bench, with
new analog demand/limits/calibration and a distinct CRC flash format (ALF1).

SHA256 buddy23_analog_line_follow_v1.uf2:

```
1af951d18ac667486f2a91f1c79b86670ab47e18fa7d142dd98f275caf95ccaa
```

First physical stage: computer USB only, no buttons/f, capture w/l/r calibration
at normal height, select1, save k, inspect h CAL line before movement. See README.
