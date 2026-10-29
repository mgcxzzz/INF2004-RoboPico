# Ramp assist v0.2 — crest-hold detector fix (v2.3 + Buddy4 add-on v0.1)

Date: 2026-10-30. Base: worktree at 8d7f3aa + Buddy4 add-on v0.1 + ramp assist v0.1.
Builds: `buddy123_interface_wifi_v2_3_b4_v0_1_ra_v0_2_{shadow,active}`.
v0.1 files are kept in `archive/ra_v0_1/`; full diff in `RAMP_ASSIST_V0_2.patch`.
`RAMP_ASSIST_V0_1.md` still describes the design; only the changes are below.

## Why

Shadow run 1 (02:27, lower bracket, 10 mm right-wheel ramp, original preset;
`testing/integration/logs/2026-10-30_ramp_assist_shadow_ramp10_run1_review.json`):

- v0.1 would have activated at uptime 254922 (height 4 mm, 4 samples >= 4).
- It ended with DESCENT 241 ms later (peak 7, current 4) and latched. The
  tilt was still rocking 4..7 mm on the way up; the crest had not been reached.
- Afterwards the heights rose to 10–11 mm and stayed at 8–9 mm while the car
  stalled at the crest: both |I| = 100, I-limit 50 ticks per 500 ms,
  PWM 295/165 at targets 45/13. No invalid samples during the climb.
- The user observed the stop at the top with the barcode IR touching the mat.
  The bracket is shared, so the lower setup stays; this test evaluates whether
  the bounded assist overcomes that drag.

Cause: the v0.1 descent rule (raw |height| 3 mm below the raw peak for 2
samples = 40 ms) is shorter than the normal rocking on the climb.

## What changed (detector only)

| | v0.1 | v0.2 |
|---|---|---|
| Height used for level/descent/peak | raw \|h\| | median of the last 3 valid \|h\| (cleared on invalid) |
| DESCENT | raw peak − \|h\| >= 3 mm for 2 samples (40 ms) | filtered peak >= 6 mm **and** filtered \|h\| <= peak/2 for 5 samples (100 ms) |
| LEVEL end of activation | raw \|h\| <= 2 for 2 samples (40 ms) | filtered \|h\| <= 2 for 5 samples (100 ms) |
| Telemetry | flags bits 4–6 = 0 | flags bits 4–6 = 2 (detector version); peak_mm is the filtered peak |

A single dip or spike cannot move a 3-sample median; a real descent (10 → 0 mm
as the right wheel leaves the ramp) still ends the assist about 120 ms after the height falls below half
the peak (one-sample median delay + 100 ms).

## Unchanged (by design)

- Boost magnitude: alpha 0.20 of the existing feedforward, cap 40 PWM per
  wheel, ramp in/out 1.0 /s (200 ms). **Not increased.**
- Activation: |h| >= 4 mm for 4 consecutive valid samples (80 ms), same sign.
- Timeout: 2.5 s cumulative assisted time per episode, then latched until 200 ms level.
- Invalid tilt > 40 ms and Buddy4 stale > 60 ms end the assist.
- Immediate end on Pause, Stop, ESTOP, fault, run end, line search, line
  recovery and junction manoeuvres (eligibility gate unchanged).
- Controller limits on the total output: 0..420 PWM, |I| <= 100, 1200 /s slew,
  anti-windup; legacy controller seeded without the added PWM.
- Targets, gains, steering, barcode, coordinator, MQTT, calibration commands.
- Shadow build: motors identical to v2.3.

Expected added PWM at the crest-stall targets 45/13 mm/s: ff 150/52 →
about **+30 left, +10 right** (both below the 40 cap).

## Host evidence (`run_ramp_assist_tests.sh`, `RAMP_ASSIST_HOST_TESTS_v0_2.txt`)

- **Captured-climb replay** (20 ms heights reconstructed from the run-1
  source-15 intervals, then 7 s stalled at 8/9 mm):
  - v0.1 reference (`ramp_assist_tests/ramp_assist_v0_1_ref.h`, the exact v0.1
    logic renamed): DESCENT after 220 ms, matching the captured 241 ms.
  - v0.2: keeps full alpha (200) through the climb and stall, ends on TIMEOUT
    at 2490 ms.
- Noisy 10 mm climb (±3 mm rocking, 1 s crest, real descent): one activation,
  ends only on the real descent/level, 2.1–2.5 s active.
- 60 ms dip to 3 mm mid-climb: assistance continues; ends at level.
- All v0.1 checks retained (boost-0 identity vs 8d7f3aa controller, stop/
  pause/fault/search/junction/invalid/stale ends, caps, slew, no residual
  integral, telemetry offsets). One bound relaxed with reason: post-ramp
  overshoot with an *abrupt* (worst-case) simulated load removal is
  11.2 vs 8.5 mm/s without assist (bound 3 mm/s), because release now
  waits for the 100 ms filtered level before the 200 ms ramp-out. Integral
  excess after release is still 0.
- clang and gcc `-Wall -Wextra -Werror`, gcc ASan/UBSan, decoder round trip,
  existing `run_tests.py` (35 executables): pass.

## Risks to watch on the bench

- With contact drag, the assist may still be insufficient. It then releases
  at 2.5 s (TIMEOUT) and the car behaves as v2.3. That is a valid result.
- If the crossing succeeds, the release comes ~100 ms later than in v0.1;
  watch for a small speed bump just after the crest.
