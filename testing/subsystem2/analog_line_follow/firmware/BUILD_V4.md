# Analog line-follow V4 - 21 October 2026

Compiled and software-verified; physical wheel-response results pending.

## Purpose

Isolate wheel-speed response before changing steering PID. Profile3 runs the
UNCHANGED V3 wheel PI/feedforward/ramp and25% PWM cap with fixed demands:
0..2s65/65mm/s,2..4s35/95mm/s,4..6s95/35mm/s. Forward-only left/right arcs.
Hard stop6s or450mm average absolute encoder path, plus existing motor guards.
Optical alignment/loss/crossing guards are bypassed ONLY for profile3; ADC still
logged. Calibration, rearm, buttons, USB-disconnect, encoder direction/invalid,
no-motion and independent100ms control-late cutoff remain enabled.
Profiles1/2 retain V3 line following. No derivative term or gain tuning added.
Select3 then k for offline, use normal bank-only start sequence. USB g has2s
countdown. See ../README.md for complete instructions; keep one power source.

## Verification

- Host GCC C11 -Wall -Wextra -Werror: phase boundaries, forward PWM bounds,
  optical bypass with stop/rearm/calibration/settling/button gates retained;
  all prior steering/calibration/recovery/kick/repeat tests passed.
- Pico SDK2.2.0, Arm GCC14.2, Pico W Release, application strict warnings passed.
- ELF text57376/data0/BSS12068. UART disabled; USBstdio enabled.
- UF2 107008 bytes, RP2040 family0xe48bff56, all payloads below0x101ff000.
- Trace/SavedLog structure unchanged. V4 uses the former reserved byte to retain
  recorded profile separately from newly selected profile (0 means old format).
  Old V1-V3 records/calibration read; older firmware cannot read selected profile3.
- Inspected both USB preflight/start alignment paths, duration/distance limits,
  active/generation PWM commit and retained stop/watchdog/feedback gates.
- Hardware not connected to build machine; no physical validation claimed.

SHA256 buddy23_analog_line_follow_v4.uf2:

```
b1a0e1333e84ac3c6c319f9c53470fe50acd90d550db4a0f22c0b3a6047159e6
```
