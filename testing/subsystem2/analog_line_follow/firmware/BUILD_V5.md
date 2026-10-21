# Analog line-follow V5 - 21 October 2026

Built and software-verified; hardware validation pending.

## Change

Provisional local forward-only wheel-speed PI: Kp0.6->1.0, Ki0.8->3.0,
integral bounds+/-80->+/-140permille. Conditional antiwindup retained;
opposite error unwinds integral; zero demand resets controller and output.
Original feedforward,100mm/s^2 ramp,25% PWM cap, breakaway and all motor guards
unchanged. Profiles1/2 use the new speed PI but unchanged optical steering;
repeat profile3 before further line tests. NO steering PID/D term added.
Shared historical dual-wheel logic is unchanged; local wheel_control.h owns tuning.

V4 measured baseline is saved under ../logs/. README includes comparison windows
and the exact repeat test. Hold load/mat/power source constant for comparison.
Physical improvement/oscillation and steady-state accuracy remain unverified.

## Checks passed

- Host C11 strict warnings: correction of fixed underspeed errors versus V4,
  overspeed integral reduction, zero reset, upper/lower saturation recovery,
 10000 variable-time-step bound checks per wheel, original phase/start/rearm/
  calibration/optical recovery/kick/button/guard tests.
- Pico W Release: SDK2.2.0, Arm GCC14.2, application -Wall -Wextra -Werror.
- ELF text57448/data0/BSS12068.
- UF2 107008 bytes: RP2040 family, block sequence, headers, all payloads below
  reserved0x101ff000 flash sector. Trace/SavedLog layout unchanged from V4.
- Source: active/generation PWM commit, no-motion, invalid/direction, independent
  watchdog/button guards, USB disconnection/countdown cancellation retained.
- No hardware actuation on build host; compile/tests are not physical validation.

SHA256 buddy23_analog_line_follow_v5.uf2:

```
00148a11fb0526e29368dae0c8288f2dcdeea0c306fdc5c4bd9e72b9ffe09fe6
```

First test: h confirm V5/valid1,3 then k; computer USB OFF before bank ON;
normal mat/load, clear1m square; wait5s, BOTH3s, release, wait1s, GP21tap/release,
2s countdown. Either button stops. Wait3s after stop, bank OFF, computer USB ON,
PuTTY1152008N1 flowNone, p. Send entire profile3 log and motion observation.
