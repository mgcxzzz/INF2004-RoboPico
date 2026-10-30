# Barcode peak decoder v0.1 (original SDK Wi-Fi v2.3 + peak decoder only)

Date: 2026-10-30. Base: the team's original `testing/subsystem2/interface_wifi_v2_3` at
8d7f3aa (git archive). **No Buddy4 add-on and no ramp assist.** Full diff: `BARCODE_PEAK_V0_1.patch`.

## Why

The 2026-10-30 raw GP27 captures show the following:

- 9 mm wide bars give peaks of ~1,700–2,300 ADC.
- 3 mm narrow bars are only small bumps (204–225 ADC) on a ~175 white level.
- The gaps next to narrow bars dip to ~190–209.

The original decoder needs one ADC threshold that sits between every narrow bar and every gap.
Its margin was ~10–15 ADC, and it is now ~0, so all four recent captures fail (Z ×3 and Y).
The bar positions are still clear (see `claude_output/HANDOFF_2026-10-30_BARCODE_DIAGNOSIS.md`).

## What it does (`barcode_peak.h`, pure C, host-tested)

1. **Smoothing:** a moving average over 0.5 mm of travel (at least 5 samples) of the same
   2 ms GP27 average the original decoder uses.
2. **Bars:** every local maximum is a bar when:
   - its prominence is at least 4 ADC, or 5 % of its height for tall wide bars;
   - it rises at least 20 ADC above the white baseline (minimum over the last 60 mm);
   - peaks closer than 3 mm are treated as one bar.
3. **Groups:**
   - A group ends after 24 mm with no bar.
   - When a group already has ≥ 15 bars, one early attempt is made after 12 mm of quiet.
     Decoding then completes ~11 mm after the label.
4. **Classification and acceptance:**
   - A bar is wide if its rise is > 50 % of the tallest bar in the group and narrow if < 15 %.
     Anything in between is rejected. Every narrow bar must reach ≥ 35 % of the median narrow rise.
   - Spaces are measured from bar-centre spacing in units of the frame's own narrow width
     (span / 46), so wheel-scale errors cancel. A space is wide if > 2 units; within ±0.4 of
     the cut it is rejected.
   - Only a complete `*X*` frame is accepted, read in either direction: 15 bars, 6 of them
     wide, narrow inter-character gaps, a valid Code39 character in the middle.
   - If a group of 16–18 bars does not decode, the 1–2 weakest narrow bars (noise bumps) are
     dropped and decoding is retried. Every successful retry must give the same letter, or the
     frame is rejected.

**Limits:**

- Full-size labels (~3 mm narrow) are assumed.
- `$ / + %` are not supported, because those characters have no wide bars.

## Firmware integration (`barcode_peak_harness.h`, 3 lines in the v12_2 main)

- It runs on every GP27 sample during an AUTO run, next to the **unchanged** original decoder.
  It is reset while the car is turning (`nav.busy`).
- **`…_barcode_peak_v0_1_shadow`:** logs only. Navigation is exactly v2.3.
- **`…_barcode_peak_v0_1_active`:**
  - When the peak decoder reads a frame the original decoder has *not* accepted, the letter
    is queued through the original path (`barcode_queue_code` and a `B_DECODE` event with
    flag 0x40). The dashboard shows it and the junction logic uses it.
  - If the original decoder already accepted that frame, nothing changes.
  - If the original decoder later reads a different letter for the same frame, its existing
    conflict rule cancels the pending action.
- **Telemetry:** every group of ≥ 6 bars sends one **source-15** frame (`0xF2`, length 24,
  subtype `0xB1`). Decode it with `python3 barcode_peak_decode.py <capture> [FROM TO]`.
  On USB there is also one `BPEAK …` line per group.
- **Unchanged:** targets, steering, line search, junction/turn logic, the original decoder,
  calibration, MQTT, stop paths, and the raw 2048-sample capture.

## Evidence (`run_barcode_peak_tests.sh`, `BARCODE_PEAK_HOST_TESTS_v0_1.txt`)

- **Four real failed captures** (streamed through the C code), each decoded exactly once:

  | Capture | Result | Max narrow space | Min wide space |
  |---|---|---|---|
  | Z original v2.3 | `*Z*` | 1.17 u | 3.01 u |
  | Z fine v0.2 run 3 | `*Z*` | 1.50 u | 3.05 u |
  | Z fine v0.3 run 1 | `*Z*` | 1.15 u | 3.08 u |
  | Y original v2.3 | `*Y*` | 1.19 u | 3.06 u |

  Each still decodes with odometry scaled ±10 %, at half the sample rate, and with ±15 ADC offset.
- **Synthetic labels:** every supported character, both directions, with a blur/halo optical
  model matched to the captures, plus noise, speed changes and stutter. 930/936 decoded,
  **0 misreads**.
- **Harsh conditions:** 2,000 trials with low contrast, noise up to 9 ADC, and speed/scale
  extremes. ~42 % decoded, **0 misreads**. An offline run of 20,000 trials also gave 0 misreads.
- **Negatives:** white, junction stripes, a truncated `*Z`, a missing bar, an extra bar,
  random bar patterns (0/300), bar overflow, backwards odometry, and a stopped car on a
  noisy edge all produce no decode.
- **Latency:** decoding finishes ≤ 11 mm after the label, with a junction 30 mm behind kept separate.
- The existing v2.3 suite still passes (35 executables). Its frozen-algorithm check allows
  `main.c` to change.
- **Build:** both variants have 0 warnings. `barcode_peak_sample` uses 592 B of stack
  (main 2,432 B, 8 KB core-0 stack).

## Not yet known (needs the car)

- Whether real bars on more letters and other runs match the captures.
- Whether a junction ever falls within ~12 mm of a label's last bar on the real track (it would
  join the bar group; a junction 12–24 mm behind is decoded separately).
- Behaviour on curves, ramps and line-recovery wobble.
