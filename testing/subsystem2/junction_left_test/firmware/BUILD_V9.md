# Junction left V9 - 22 October 2026

Artifact `buddy23_junction_left_v9.uf2`:123904 bytes,242 UF2 blocks.

SHA256:

```
4a2e3367d3a86020cc98af9fcc2ca532b2c3fd27a07b7057d482b9a57d35e871
```

User explicitly requests6s alignment, then10s following AFTER alignment succeeds.
The earlier3s was a provisional firmware test guard, not a supplied NFR.

V8 physical evidence: entry right PWM74 shows the V7 zero-output handover was
corrected. Levels changed from left1000/right0 to left75/right743 only~0.6s before
the3s deadline. Both-sensor stable tracking never qualified; phase6 never started.
Acquisition travel57.71mm nearly reached the old60mm bound. See MEASUREMENTS.md.

## V9 change

- Phase9 alignment window6,000,000us. Completion still requires both levels>=400
  and difference<=350 for80ms; early success immediately starts phase6.
- Alignment path bound180mm, increased from60, allows6s at nominal mean<=30mm/s.
  Keeping60mm would otherwise stop this run soon after the old3s limit. Actual
  overspeed can reach the path abort early. Angle60..115deg/loss guards unchanged.
- Phase6 follows10,000,000us from its own entry timestamp. It does not include
  alignment/turn time and has no distance-based normal completion condition.
- Whole-run35s/900mm. Trace500ms (was400) fits100-row saved record plus transitions
  and settling; static budget92<=100. Control/ADC still10ms. Existing flash
 3792-byte/row192/Trace36 layout remains; V1..V8 calibration/records import.
- V8 PI-reset fix, speed targets, pins, PWM caps and motor/button/USB/late/encoder
  checks unchanged. Old records retain their revision. New whole-deadline label
  is generic to avoid attributing a new deadline length to restored older logs.

## Verification

- Strict C11 host Wall/Wextra/Werror/O2 suite passed: active at3s and5,999,999us,
  alignment timeout at6,000,000us; continued past61mm, abort at180mm. Alignment
  success at5.58s starts a separate10s clock. Follow continues at5s/9,999,999us
  and completes at10,000,000us. Prior sequences, V8 PI reset, optical/path/loss,
  angle/feedback/button guards and bounded PWM/peak tests pass.
- Pico W Release, SDK2.2.0, Arm GCC14.2.1; application warnings treated as errors.
  ELF text66024/data0/BSS12232, end0x1000f1ec below log sector0x101ff000. USBstdio
  init present, UARTstdio init absent. Static saved record and trace budget pass.
- Verified UF2 magic/end markers,242-block sequence/count,256-byte payloads,
  RP2040 family0xe48bff56 and flash ranges, padded end0x1000f200.
- No build-host hardware actuation. Physical V9 result pending. More alignment
  time is a bounded experiment and does not establish stable capture in advance.

Flash V9,h verify valid1/side_valid1,3,k,h confirm mode3. Same mounting/start;
endpoints carry over unchanged. Allow roughly450..500mm of outgoing track beyond
the completed alignment point for the10s follow. Either button/s stops; faults
can interrupt either timer. Expected1,2,4,9,6,7 then test complete. Wait3s for save
before switching sources separately, p; send fresh V9 log/video.
