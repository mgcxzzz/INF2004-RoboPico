# Combined build v0.3 — ramp assist v0.4: crest hold + bounded stall push

Folder `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_3/` (2026-10-31) = combined v0.2 + stall push. v0.1/v0.2 folders and all earlier UF2s preserved. Diff vs v0.2: `B4_PEAK_V0_3.patch`; pre-change files in `archive/ra_v0_3/`. **Host-tested only.**

## Firmware

`bench_tools/buddy4/buddy1234_peak_v0_3_ra_active_bp_active.uf2` — sha256 `bc376eeca37798f410523a84b6888a60c808cc480314ff5b93dad2799d2447b5` (ELF `6e975d3e01821a01988787659029c825900088dcc5761e8c869dff80c6af27c2`), 1,039,360 bytes / 2,030 blocks, flash 0x10000000–0x1007ee00. Ramp assist ACTIVE (detector 4), peak decoder ACTIVE. Contains local Wi-Fi credentials.

## Why (run at 01:21, v0.1 firmware, 10 mm ramp)

Stopped at 201 mm with the right wheel still up (tilt 8–11 mm) and the front settling onto the mat (barcode ADC 430→1,230) for ~20 s, at **295/165 PWM** (limit 420). The controller output is ff + error + integral (integral capped at 100); steering asked the ramp wheel for 13 mm/s; the +20 % assist had chattered (fixed in v0.2's hold) and then timed out. The host model of the existing controller reproduces exactly 295/165 at that stall.

## Change (ramp assist v0.4)

Everything from v0.2/v0.3 (+20 % feedforward assist, cap 40, 150 ms ACCEL/SPREAD hold) plus a **stall push**:
- Only while ramp assist is active (tilt elevated). If both wheels measure < 5 mm/s for 200 ms → add **+60 PWM to both wheels** (steering difference kept), ramped at 200 PWM/s (0.3 s).
- Released when a wheel exceeds 15 mm/s, or the assist ends (level/descent/invalid/stale/timeout); ramps out in 0.3 s.
- Budget 1.5 s of push per ramp; push time does not count toward the 2.5 s assist timeout → at most 4.0 s of help per ramp.
- Stop / Pause / ESTOP / fault / line search / junction remove the push at once. The controller's 0–420 clamp, 1200/s slew and anti-windup apply to the total; controller reset clears the push.
- At the 01:21 stall this gives 385/235 PWM instead of 295/165 (+30+60 / +10+60).
- Telemetry: ramp source-15 event 3 = "stall push"; added-PWM fields include the push (can exceed 40); detector bits = 4. Dashboard timeline shows "Ramp assist stall push".
- Code: `continuous_drive.h` gains `boost_add` (absolute, ≥ 0, zero after reset; folded into the existing boost term); `ramp_assist.h` push logic; harness passes `velocity[]` and sets `boost_add`. Gains, targets, steering, Buddy4, barcode and stop paths unchanged.

## Host evidence (`B4_PEAK_HOST_TESTS_v0_3.txt`)

Ramp-assist suite 360,883 checks (clang/gcc/ASan): stall → one push to +60; 1.5 s budget; assist ends at 2.5 s + 1.5 s; moving releases; < 200 ms stop is not a stall; level return / Stop / Pause remove it (Stop/Pause immediately); never pushes unless assisting; push adds exactly +60 below the 420 clamp, never exceeds 420, no residual integral after release; 01:21 stall reproduced (295/165 → 385/235). All v0.2/v0.3 checks retained. `run_combined_tests.sh` passes (cloud + Mac VM). Dashboard tests pass. ARM: 0 warnings; bss 202,128; stack main 2,368 B, control_step 528 B.

## Build

```sh
cp ../interface_wifi_v2_3/wifi_config.local.h .
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w -DRAMP_ASSIST_ACTIVE=ON -DBARCODE_PEAK_ACTIVE=ON
cmake --build build
```

## Short hardware test

1. Recorder on. Flash; Reset and keep still 5 s; IMU (B4) not Fault; dashboard build shows **detector 4**.
2. Flat run: no assist, no push, normal following.
3. 10 mm ramp, 1–2 runs. Expect assist through the crest; if it stops, a "stall push" event and the car moving again within ~0.5 s. Stop at once if it lurches off the line or stays stalled 3 s after the push.
4. Send the capture.
