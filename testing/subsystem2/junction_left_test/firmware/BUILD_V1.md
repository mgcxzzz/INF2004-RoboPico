# Junction left V1 — 21 October 2026

Built from isolated testing/subsystem2/junction_left_test sources. Reuses V11
symmetric_logic.h/line_logic.h and common Buddy2 encoder/pin/guard definitions;
does not change their controllers, any production subsystem or teammate branch.

Artifact: buddy23_junction_left_v1.uf2, 113664 bytes.

SHA256:

```
dbe67703ffcfd108ffbf679903fed95b10ea9a5fd14b7a367e04edf5c30f2605
```

UF2 headers, sequence, 256-byte payloads, RP2040 family0xe48bff56 and all flash
payload addresses below the reserved sector were verified against this artifact.

Software checks passed:

- Strict C11 host suite (-Wall -Wextra -Werror -O2): cross qualification and pulse
  rejection; midpoint+92 mm geometry; centre/settle/left-turn/exit sequences;
  stop-only mode; missing branch, missing trailing edge, heading drift, settle
  timeout and turn timeout rejection; original black cannot complete a turn;
  one-black cannot confirm branch; both side polarities and invalid captures.
- Signed PI/output sweep -50..50 mm/s on each wheel, PWM sign/cap, bridge mapping,
  zero clears integral/ramp/boost; original GP20/release/late-control guards.
- Ideal kinematic sequence tests are logic checks, NOT physical floor simulation.
- Pico W/RP2040 Release, SDK2.2.0/Arm GCC14.2.1; application -Wall -Wextra -Werror.
- ELF text60760, data0, BSS12184. USB stdio symbols present; UART init absent.
- Compile-time SavedLog<=4096, Trace36 bytes, run+phase/settling trace budget.
- Firmware ends0x1000dd5c, below reserved flash sector0x101ff000.

No robot attached to build host and no hardware actuation performed. Side D0
polarity, measured centring, loaded reverse response and optical left turn all
require the user's physical tests. Start with stationary o/b check and mode1.

Build: configure CMake/Ninja with PICO_BOARD=pico_w, PICO_SDK_PATH, installed Arm
toolchain and picotool; cmake --build build -j8. Host test:

```
gcc -std=c11 -Wall -Wextra -Werror -O2 test_junction.c -lm -o test_junction.exe
./test_junction.exe
```
