# Circuit with barcode V2 build evidence

Built 2026-10-23, Pico W / RP2040, Pico SDK 2.2.0, Arm GNU, CMake/Ninja
Release, C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_with_barcode_v2.uf2`

- Size 176,640 bytes; 345 sequential RP2040 UF2 blocks.
- SHA-256: `925e6e6020ef686e136522c208a2825bdb5f6be13b67673402c99535d46a5c66`.
- Padded end `0x10015900`; linker end `0x1001586c`.
- ELF text 92,264 bytes; BSS 144,160 bytes; BSS end `0x20024620`;
  stack limit `0x20040000`. USB init/write symbols verified.
- Payload remains below the barcode bank at `0x101d0000`.
- Record ABI unchanged: 64,840 bytes, trace 56, summary 40, event 32.
  V1/V2 records read correctly; old runs retain version 1, new runs save 2.
- Original V1 UF2 unchanged; SHA-256 remains
  `ee1c7fc1991b8a78fce1d42368684e29eb9672e2a2ab989d0b141b2ae56c5fcf`.

## Recorded failure regression

`logs/a_v1_capture_events.csv` contains the 46 compact events supplied by the
user for the run ending at 63.385 s, 1,697.1 mm, no decoded letters and 7 reported
straight crosses. The user identifies its first barcode as A. The attached
motion report cuts off at 27.277 s; the compact event section is complete.
No missing motion rows or stop positions were inferred.

`fixtures/captured_a.h` preserves its 29 completed barcode RUN widths and
timestamps. These include narrow black bars near 8 mm, narrow gaps near 2 mm,
wide black bars 18.64–24.10 mm, and total measured frame 262.11 mm. A subsequent
113.29 mm white gap precedes the 16.27 mm cross. Those are encoder/D0 readings,
not externally measured print dimensions.

Executed replay results:

```text
CAPTURE REPLAY: DECODE A/LEFT at36.315s; patterns094,109,094; span262.10mm
CAPTURE REPLAY: following16.27mm cross confirmed after4mm white; dispatched A/LEFT once.
```

The replay begins at 1 ms, so its elapsed event time is 1 ms below the input
timestamp; its event width field truncates to hundredths of a millimetre.
Input completed edges retain the source timestamps. The replay includes
10 ms interpolated expiry checks, so it also verifies that the former 210 mm
frame limit no longer discards this captured A. It asserts one start and one
decode, no cross candidates inside the A frame, queued A, and a single
subsequent left dispatch. It does not assert physical turn success.

## Change and bounds

- Learn two wide black bars/one wide white gap from either-direction `*`.
  Each colour gets independent narrow/wide means and a separated threshold.
- Classify the payload/stop against those learned thresholds; require exactly
  three wide elements, valid character, both sentinels and narrow white
  inter-character gaps. No nearest-letter guess or hardcoded A bypass.
- Retain 0.35 mm minimum; broaden single-element plausibility to 40 mm. Scale
  total frame allowance from start-sentinel length, capped 600 mm; timeout 20 s.
- Cross width remains 11–30 mm. Frame/isolation context replaces the erroneous
  requirement that a cross exceed the barcode's wide bars.
- Require 4 mm white after a cross candidate. A new bar before that distance
  cancels it and retains pending. Preserve cross leading position for the
  original 92 mm side-to-axle and 55 mm rolling-start geometry.
- A/F mapping, line follower, effort limits, speed profiles, turn controller,
  calibration and GPIO wiring remain as V1.

## Executed checks

Host C11 warnings-as-errors suite passes:

- Exact captured A, forward/reverse; exact timed A-to-cross dispatch replay.
- Supplied PDF A/Z fixtures at both sizes and scan directions.
- A/F normal and independently biased black/white widths, two sizes,
  forward/reverse, changing speeds and element jitter.
- Incomplete/malformed frame rejection, D0 debounce, edge queue overflow.
- 4 mm white boundary, early black cancellation without command consumption,
  normal cross smaller than barcode bars, gate/clearance rejection.
- Twelve alternating A/F turns per profile with variable barcode/cross gaps,
  mirrored right acquisition and real-world follower handover.
- V1/V2 record compatibility and CRC corruption rejection.

Arm build, UF2 format/family/address/markers/build equality checks and memory
bounds pass. V1's firmware remains available. No motors or sensors were
exercised by these software tests: first hardware validation is larger A
in profile 1; the smaller F is explicitly deferred.
