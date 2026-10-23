# Circuit with barcode V3 build evidence

Built 2026-10-23 for Pico W / RP2040 using Pico SDK 2.2.0, Arm GNU,
CMake/Ninja Release and C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_with_barcode_v3.uf2`

- Size 177,664 bytes; 347 sequential RP2040 UF2 blocks.
- SHA-256: `9946f4d3aaafea3769803a8ad6a406c3ce20be4adb0344a4f2ae9721356ad6bd`.
- Padded end `0x10015b00`; linker end `0x10015a9c`.
- ELF text 92,824 bytes; BSS 144,160 bytes; BSS end `0x20024620`;
  stack limit `0x20040000`. USB init/write symbols verified.
- Payload remains below the barcode record bank at `0x101d0000`.
- Record ABI unchanged: 64,840 bytes; trace 56, summary 40, event 32.
  V1–V3 saved records read correctly; unsupported versions are rejected.
  Older runs retain their original version and flag meanings.
- Existing V1 and V2 firmware hashes checked unchanged against BUILD_V1/V2.

## Failure and correction

`logs/a_v2_capture_events.csv` preserves the 37 compact events from the user
run ending at 49.685 s, 1,474.0 mm. Its first A begins at 25.915 s, reports
START at 27.689 s and BAD_FRAME at 33.062 s. Its following 15.75 mm cross
passes the 4 mm white check at 36.050 s but goes straight because no command
was decoded. This establishes barcode validation failure, not failed cross
detection. The attached motion report truncates at 26.250 s; no missing rows
or physical positions were inferred.

The start's black narrow values are 3.00, 8.03 and 2.88 mm: mean 4.6367 mm.
V2's mean-based upper bound was 8.346 mm, below subsequent legitimate
narrow-class readings 9.34, 9.65 and 9.38 mm. They are all below the learned
black threshold of 11.595 mm. White threshold is 11.30 mm. Applying those
colour thresholds gives 094/109/094, a complete `*A*`.

V3 removes relative mean-based narrow bounds; each colour's learned
threshold determines its narrow/wide classes. Absolute 0.35–40 mm bounds,
wide plausibility limits, class separation, exactly three wide elements,
valid lookup, both sentinels, character gaps and bounded frame expiry remain.
No hardcoded A acceptance or nearest-character substitution is used.

The classifier completes all nine pattern bits even when a validation check
fails. Completed BAD_FRAME records include rejection bits (legend in README)
rather than misleading partial patterns. Early invalid runs and time/travel
expiry have distinct flags. Saved record layout is unchanged.

## Executed tests

Host C11 warnings-as-errors suite passes. Actual recorded RUN widths and
timestamps are replayed with 10 ms interpolated expiry checks:

```text
CAPTURE V1 REPLAY: DECODE A/LEFT at36.315s; patterns094,109,094; span262.10mm
CAPTURE V1 REPLAY: following16.27mm cross confirmed after4mm white; dispatched A/LEFT once.
CAPTURE V2 REPLAY: DECODE A/LEFT at33.061s; patterns094,109,094; span258.95mm
CAPTURE V2 REPLAY: following15.75mm cross confirmed after4mm white; dispatched A/LEFT once.
```

Replay starts at 1 ms, so elapsed times are 1 ms below the original edge
timestamps. Reconstructed spans sum the individually truncated printed RUN
widths; the original full-precision frame width can differ by accumulated
rounding. Both fixtures assert one start, one A decode, no cross candidates
inside the frame, a queued A and one left dispatch at the next confirmed cross.

Additional executed coverage:

- Both recorded A fixtures decoded in reverse as well as forward.
- PDF A/Z fixtures, generated A/F both sizes/directions, independent black/
  white bias, jitter and varying speed.
- Malformed stop and expiry rejection. Valid-looking 094/109/094 patterns
  with implausible width or invalid character gap remain rejected and retain
  all pattern bits with the relevant rejection reason.
- D0 debounce and queue overflow; 4 mm white confirmation/cancellation;
  pending command replacement, unsupported-code behaviour, no-command straight.
- Twelve alternating A/F turns per profile, variable barcode/cross distances,
  mirrored right acquisition and normal follower handover.
- V1/V2/V3 record compatibility, unsupported version rejection and CRC checks.

Arm build and UF2 format/family/address/marker/build-equality checks pass.
The follower, speed/PWM settings, sensor capture, rolling-turn controller,
calibration and V2 junction confirmation are unchanged. Physical V3 scanning
and turning remain to test: larger A, profile 1 first; smaller F later.
