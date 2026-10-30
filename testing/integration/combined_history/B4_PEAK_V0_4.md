# Combined build v0.4 — ramp assist v0.5: tilt floor (300 PWM per wheel until flat)

Folder `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_4/` (2026-10-31) = combined v0.3 + tilt floor. Earlier folders and all UF2s preserved. Diff vs v0.3: `B4_PEAK_V0_4.patch`; pre-change files in `archive/ra_v0_4/`. **Host-tested only.**

## Firmware

`bench_tools/buddy4/buddy1234_peak_v0_4_ra_active_bp_active.uf2` — sha256 `d5bcd275a4930ec93ee44736f7c87c588cd1ed3d5e29240f3aa00c334fc04271` (ELF `8f2b130459b04e605a48516a2fd2bd4dadb31c4ac019e7b1649e2f8685bc3a6c`), 1,040,896 bytes / 2,033 blocks, flash 0x10000000–0x1007f100. Ramp assist ACTIVE (detector 5), peak decoder ACTIVE. Contains local Wi-Fi credentials (verified in the flat image).

## Why (runs 01:49–01:51, bracket tilted slightly up)

4 of 5 runs cleared (humps accepted 14–16 mm). The failing run (01:50:15) was pushed free twice, then stuck at 225 mm with the left wheel at 295 PWM but the **ramp wheel at 12–51 PWM**: line steering asked it for 2–9 mm/s because the tilted car saw the line off-centre (right sensor ~1,300).

## Change (ramp assist v0.5)

- From assist activation until the tilt is back to flat (median |h| ≤ 2 mm for 100 ms — **not** the earlier descent rule), each wheel's requested output is **at least 300 PWM** (floor ramped 0→300 in 0.3 s and back). Steering can still ask for more than 300 but cannot starve either wheel.
- Bounded: 5 s per ramp (from activation); ends on stale Buddy4 (60 ms), other invalid tilt (40 ms) or ACCEL/SPREAD exclusion > 150 ms. Stop / Pause / ESTOP / line search / junction remove it at once. 0–420 clamp and 1200/s slew apply.
- The floor does not wind the integral (normal PI update while floored), so releasing it leaves the controller as if there had been no floor.
- Unchanged and still active: +20 % feedforward assist (2.5 s), 150 ms ACCEL/SPREAD hold, +60 stall push (1.5 s).
- Trade-off: while tilted both wheels run ≥ 300, so steering is weaker on the ramp; line following resumes at flat.
- Telemetry: ramp source-15 event 4 = "tilt floor end" (it starts at "activate"); detector bits = 5.
- Code: `continuous_drive.h` `floor_pwm` (zero after reset; floor 0 = identical controller); `ramp_assist.h` floor state; harness sets `floor_pwm`.

## Host evidence

Ramp-assist suite 360,898 checks (clang/gcc/ASan): floor to 300 from activation; held through crest/descent, ends ~100 ms after flat; 5 s budget; Stop/Pause immediate; ACCEL/SPREAD < 150 ms held, longer ends; other invalid ends after 40 ms; no floor before a ramp; starved wheel (target 3 mm/s, stalled) 28 → 300 PWM; ≤ 420; no integral wind / residual; reset clears it; all earlier checks retained. `run_combined_tests.sh` passes (cloud + Mac VM); dashboard tests pass. ARM: 0 warnings; bss 202,160; stack main 2,368 B, control_step 520 B.

## Build

```sh
cp ../interface_wifi_v2_3/wifi_config.local.h .
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w -DRAMP_ASSIST_ACTIVE=ON -DBARCODE_PEAK_ACTIVE=ON
cmake --build build
```

## Short hardware test

1. Recorder on; flash; Reset and keep still 5 s (IMU not Fault); dashboard shows **detector 5**.
2. Flat run: no assist, no floor.
3. 10 mm ramp ×3–5 (same bracket tilt). Expect both wheels ≥ 300 PWM on the ramp, crossing without long stalls, floor ending at flat ("tilt floor end"). Stop at once if it drives off the line or stays stuck 3 s.
4. Send the capture.
