# Combined build v0.5 — ramp assist v0.6: speed hold (replaces the 300 PWM floor)

Folder `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5/` (2026-10-31) = combined v0.4 with the tilt floor replaced by a speed hold. Diff vs v0.4: `B4_PEAK_V0_5.patch` (includes the dashboard label change); pre-change files in `archive/ra_v0_5/`; earlier versions' notes and patches in `testing/integration/combined_history/`. **Hardware-tested 2026-10-31 (results below).** Both features are ON by default in `CMakeLists.txt` (same UF2 as the explicit `-D...=ON` build).

## Firmware

`bench_tools/buddy4/buddy1234_peak_v0_5_ra_active_bp_active.uf2` — sha256 `dacc4e768ccbcdfa7485ec410e1b2ec4defff48c60b3ab65584292ac66a15118` (ELF `609a500c2cbcef7b67eaaf7c06a3d0a7b769a98225e3967c630c53d6543d2da6`), 1,040,896 bytes / 2,033 blocks, flash 0x10000000–0x1007f100. Ramp assist ACTIVE (detector 6), peak decoder ACTIVE. Contains local Wi-Fi credentials (verified in the flat image; none in build/test logs).

## Hardware result (2026-10-31, power bank, 10 mm right-wheel ramp, bracket tilted slightly up)

Excerpt: `testing/integration/logs/2026-10-31_combined_v0_5_speed_hold_runs_native_excerpt.txt`.

| Run | Ramp detected | Crossed | Slowest at crest | Stall push | Buddy4 hump |
|---|---|---|---|---|---|
| 02:26:27 | yes (detector 6) | yes | ~22 mm/s | no | accepted 12 mm |
| 02:27:42 | yes | yes | ~22 mm/s | no | accepted 11 mm |
| 02:33:11 | yes | yes | ~31 mm/s | no | accepted 13 mm |
| 02:38:44 (barcode run) | — | — | — | — | peak decoder read **Y** (reverse direction); not mapped, so no movement |

At the crest the steering again asked the ramp wheel for 0–4 mm/s and the front touched the mat (barcode ADC ~930–1,020); the hold kept that wheel at 250–280 PWM and both wheels moving. Climb peak ~60 mm/s (v0.4: 107–131). After the ramp both wheels returned to ~200 PWM at 45 mm/s. In the 02:33 run the hold limited a sharp correction just after the descent (left wheel kept at 35 mm/s); the car recovered the line by ~300 mm. Further runs by the user after these were reported OK (not all recorded).

## Why (v0.4 runs 02:07–02:08, capture `2026-10-31_buddy1234_peak_v0_4_capture.txt`)

All four ramp runs did the same thing:
1. The 300 PWM floor drove the climb at **107–131 mm/s** (target 45): the climb never needed that much.
2. ~0.47 s in, Buddy4 reported invalid tilt for a reason other than ACCEL/SPREAD; assist and floor ended after 40 ms and the floor (single use) never returned.
3. The car reached the crest fast, nose on the mat (barcode ADC ~1,000–1,100), and stopped at 193–216 mm.
4. Re-activation had no floor; steering asked the ramp wheel for 0–3 mm/s (0–25 PWM) until the stall push freed it ~1 s later.
5. Buddy4 rejected every hump (detail 3, 7–12 mm): too much jolting at that speed.

Teammate's point: hold a speed, not a fixed PWM.

## Change (ramp assist v0.6)

From assist activation until the tilt has been within 2 mm for 200 ms (episode end), the existing continuous speed controller gets, ramped in/out over 0.2 s:
- **minimum speed target 35 mm/s per wheel** — steering can ask for more, never for less (also when it asks for 0);
- **integral limit 300** instead of 100 and **3× integral gain**, so the speed loop itself adds whatever PWM is needed (up to the unchanged 420 clamp) to hold the speed.

Ends: flat 200 ms, 5 s per ramp episode, no Buddy4 sample for 200 ms, or invalid tilt of **any** reason for more than 500 ms (shorter invalid only keeps it). A re-activation in the same episode restarts it within the budget. Stop / Pause / ESTOP / line search / junction remove it at once. When it ramps out, the integral is clamped back to ±100.

Removed: the 300 PWM floor. Unchanged and still active: +20 % feedforward assist (2.5 s), 150 ms ACCEL/SPREAD hold, +60 stall push. Gains, 420 clamp and 1200/s slew are otherwise unchanged; with the hold fields at zero the controller is bit-identical to the original (test A).

Trade-off: while held, neither wheel goes below 35 mm/s, so steering on the ramp is limited to 35–45.

Telemetry: detector bits = 6; ramp event 4 = "floor/hold end" (dashboard label updated); max |I| fields now report up to 255.

## Host evidence (`B4_PEAK_HOST_TESTS_v0_5.txt`)

Ramp-assist suite 360,906 checks (clang, gcc, ASan/UBSan), including:
- hold on from activation, kept through crest/descent, ends 200 ms after flat; 5 s budget; Stop/Pause immediate; 300 ms invalid (any reason) kept; > 500 ms ends; re-activation restarts it; stale 200 ms ends; none on the flat;
- 02:07 descent stall (steering target 2 mm/s, 250 PWM nose load): without hold 0 mm/s (as logged); with hold ~33–35 mm/s, output ≤ 390, |I| ≤ 250;
- climb at 45 with 60 PWM load: ≤ 45 mm/s while loaded (v0.5 floor ≈ 90+ mm/s); overshoot on sudden load removal within 2 mm/s of no hold;
- after the hold: integral back within 100, output converges to the no-hold controller; reset clears it; target 0 with hold drives toward 35; negative target resets.

`run_combined_tests.sh` passes (cloud clang+gcc, Mac VM gcc). ARM Release: 0 warnings; text 530,680, bss 202,176; stack main 2,368 B, control_step 536 B.

## Build

```sh
cp ../interface_wifi_v2_3/wifi_config.local.h .
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w -DRAMP_ASSIST_ACTIVE=ON -DBARCODE_PEAK_ACTIVE=ON
cmake --build build
```

## Re-test procedure

1. Recorder on; flash; Reset and keep still 5 s (IMU not Fault); dashboard shows **detector 6**.
2. Flat run: no assist.
3. 10 mm ramp ×3–5 (same bracket). Expect the climb near 45 mm/s (not 100+), no long stop at the crest, both wheels moving on the descent, Buddy4 hump accepted. Stop if it leaves the line or is stuck 3 s.
4. Send the capture.
