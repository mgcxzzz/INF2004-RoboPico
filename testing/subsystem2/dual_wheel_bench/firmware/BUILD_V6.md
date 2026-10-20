# V6 build verification — 20 October 2026

- Pico W/WH RP2040, SDK 2.2.0, GCC 14.2.1, CMake/Ninja Release.
- Firmware compile/link/UF2 PASS, warnings treated as errors.
- Host C11 tests PASS: existing calibration, polarity, output cap, stop guards,
  offline countdown cancellation, boot arm, CRC and initial PI model; plus
  debounced selection baseline/edge/one-shot/USB cancel, g range and deadline,
  signed finish duty and target threshold, and long trace capacity.
- These checks are not physical motor tuning, range or floor accuracy evidence.
- picotool confirms buddy2_dual_wheel_v6, pico_w, USB stdin/stdout.
- ELF text 51,056 bytes, data 0, BSS 10,748.
- Image ends 0x1000b774, below reserved log sector at 0x101ff000.
- Saved format remains V5-compatible; new runs identify g/q/e/d in the log.
- All hardware tests pending. Short raised d run first.

SHA256 buddy2_dual_wheel_v6.uf2:

```
c329b8889ce934787e06a778030969a4f24cbb36b5525a4284927819f55f2c49
```

Source: adjacent main.c, dual_logic.h, log_store.h and CMakeLists.txt plus V3
encoder helper. Only profile g extends the deadline to 6 s; output remains
capped at 25%. Flash writing remains restricted to drive off after settling.
