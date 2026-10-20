# V7 build verification — 20 October 2026

- Target RP2040 Pico W/WH; SDK 2.2.0, GCC 14.2.1, CMake/Ninja Release.
- Compile/link/UF2 PASS with -Wall -Wextra -Werror.
- Native C11 tests PASS: inherited geometry, polarities, cap, guards, profile
  selection, CRC and PI model; new turn assist ramp threshold, speed release,
  pulse expiry/cooldown, opposing signs, target-done cancellation, wrong-way
  non-boost, 25% cap, straight-path bypass, and reduced final turn demand.
- Physical loaded turn start, overshoot and angle accuracy remain unverified.
- picotool confirms buddy2_dual_wheel_v7, pico_w, USB stdin/stdout.
- ELF text 51,824 bytes, data 0, BSS 10,764.
- Image end 0x1000ba74 is below flash log sector 0x101ff000.
- Flash format remains V5-compatible; keep original run key/version context.

SHA256 of buddy2_dual_wheel_v7.uf2:

```
0aa853555771deaba3f53d44af8e24cd0f59fcc5c560c3d8eaace6c848d9a546
```

Source: adjacent main.c, dual_logic.h, log_store.h, CMakeLists.txt and V3 decoder.
Existing 3 s turn/short limits and 6 s long straight limit retained. USB only;
no UART console on the encoder pins. No final RTOS implementation is claimed.
