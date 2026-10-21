# Analog line-follow V6 - 21 October 2026

Compiled and software-verified; physical validation pending.

## Change

Profiles1/2 only: cruise65->45mm/s; proportional differential+/-30->+/-15mm/s.
Wheel requests30..60mm/s, with edge target500/deadband60 unchanged.
Right assist60/30; both-white seek30/60; both-dark pending35/35 unchanged.
No derivative term or additional smoothing. V5 wheel PI, feedforward, ramp,
25% PWM cap, bounded breakaway, calibration and guard thresholds unchanged.
Profile3 wheel-response sequence byte-for-byte unchanged in source.

Most recent close-height V5 run had strong alternating correction requests and
right PWM saturation, then NORMAL3s stop. New mapping aims to reduce demand
reversals and speed; no hardware improvement is claimed until tested. Existing
startup kicks can still produce stutter; mounting drag is not solved in software.

## Checks passed

- Host C11 -Wall -Wextra -Werror:45mm/s midpoint; extrema60/30 and30/60;
  reduced request reversal; independent calibrated close-height midpoint;
  all52k+ sampled input pairs within0..60/forward30..60; original calibration,
  loss time/distance, both-dark stop, kick/cooldown, rearm, mode/button gates,
  profile3 stages and V5 PI saturation/unwinding tests passed.
- Pico W Release, SDK2.2.0, Arm GCC14.2, application strict warnings.
- ELF text57472/data0/BSS12068; UART off, USBstdio enabled.
- UF2 107008 bytes; RP2040 family, valid block sequence/headers, payloads below
  reserved flash0x101ff000. Trace/SavedLog layout unchanged; saved calibration loads.
- Source comparison: wheel_control.h, breakaway function and response_demand
  unchanged. Active/generation PWM commit, no-motion, direction and button/
  USB-disconnection cancellation gates retained.
- No connected robot on build host; only user can validate motion on the mat.

SHA256 buddy23_analog_line_follow_v6.uf2:

```
e0d620fc59299520861a041ff572e8c071eb44c5449ed2d3b014eceec652d486
```

First physical test: profile1 on short straight with same mounting/load. h
verify V6/valid1 and close-height calibration;1 then k; levelL450..550/rightwhite.
Computer USB OFF before bank ON. Wait5s, BOTH3s, release BOTH, wait1s, GP21tap/
release,2s countdown. Either button stops. Profile1 remains3s/150mm: lower travel
at time-limit is expected. Wait3s after stop, bank OFF, computer USB ON, p.
Send complete log and observe steering smoothness separately from startup stutter.
