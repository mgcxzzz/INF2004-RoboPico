# Analog line-follow V11 - 21 October 2026

Compiled and software-verified; adaptive steering physical test pending.

Profile6 adaptive forward speed by normalized absolute L-R difference:
0..80 ->inner/outer45/45;250 ->40/50;500 ->25/50;1000 ->0/45mm/s.
Piecewise integer interpolation and coupled quantization produce mirror-symmetric
requests with at most1mm/s adjacent target change, nonincreasing average forward
speed and nondecreasing curvature as imbalance grows. Left greater ->left inner;
right greater ->right inner. Extreme zero request resets that wheel PI/kick and
sets both PWM channels0 (coast, not active brake or reverse).

Recent-direction white recovery changes30/60to0/45left,60/30to45/0right; same
500ms memory,650ms/25mm bounds, unknown-side stop. Keep10s/500mm,125ms trace,
calibration/start/rearm, ADC filters, PI gains,100mm/s^2 nonzero-target ramp,
25% PWM cap, startup boost and motor/USB/button/control-late guards.
Higher PWM was permitted if necessary but is not used: tighter turning comes
from the lower inner-wheel speed. Power/torque needs require actual V11 logs.
No claim that a previously successful V10 run was a failure; latest user
clarification is recorded alongside archived successful evidence in README.

Checks passed:
-Host strict C11 suite: all1001 magnitude steps monotonic forward-speed/curvature,
  mirrored signs/anchors/continuity,10s command sequence with both inner-zero
  directions verifying PI/kick reset and actual bridge-output zero, PWM0..250;
  prior loss/start/calibration/PI/guard/controller tests.
-Pico W Release SDK2.2.0 Arm GCC14.2, application -Wall -Wextra -Werror.
-ELF text63848/data0/BSS12092; USBstdio enabled, UART disabled.
-UF2 119808 bytes, RP2040 family/headers/sequence/payload below0x101ff000.
-Trace/SavedLog structure/version unchanged; old calibration/logs readable.
-Historical controllers and wheel_control.h unchanged; motor commit/stop gates retained.
-No actual hardware verification at build host. Watch for low-speed stiction,
  overshoot and repeated kicks in the next physical run.

SHA256 buddy23_analog_line_follow_v11.uf2:

```
96f70ff371369a7f890e70aac1f643a7a34285dfadc2e71cf50d08068682a6a6
```

Test profile6: h verify V11/valid1;6 then k. Same symmetric mounting/load,
centred nearly parallel on straight first. About600mm clear continuous line.
One power source; normal bank button sequence; either button stops. Wait3s,
switch separately back to computerUSB, p. Save fresh complete log before rerun.
After centred review, small parallel left/right offsets; no steep stationary
acquisition assumption and no claim of continuous-bend success from those tests.
