# Junction left V8 - 22 October 2026

Artifact `buddy23_junction_left_v8.uf2`:123904 bytes,242 UF2 blocks.

SHA256:

```
df433ca2111edbf2183e1be926ca4cae9573b822cb7077335873a669c7fcf403
```

## Change

The V7 log reached phase9 with left794/right1, then right PWM0 and later56.
It stayed left1000/right0 and hit the3s acquisition timeout; phase6 never began.
The signed controller retained its pivot integral when changing to ordinary
forward feedforward. A representative I=-82/ramp30/speed31.8/target33 reproduces
the zero-output handover. The integral value is inferred, not directly logged.

V8 clears the PI integral on both directions of the turn/forward transition,
retains its speed ramp, and lets forward I accumulate on later ticks. Forward
operation outside that transition remains exactly equivalent to V11. No changes
to targets, optical confirmation,3s acquisition timeout, turn geometry, five-second
phase6,30s/900mm global bounds, PWM or motor/button/USB/loss guards.

Append a distinct `outgoing-line alignment timeout` stop reason for new phase9
timeouts; phase4 retains `turn timeout`. Existing reason numbers are preserved.
Saved record3792/row192/trace36 unchanged; accept V1..V7 calibration/logs and
retain their revision labels. New runs identify V8. No wiring changes.

## Verification

- Strict C11 host suite passed (Wall/Wextra/Werror/O2). New regression checks the
  representative negative integral handover produces nonzero normal forward
  output, ramp30->31, I reset near0, then subsequent I buildup rather than reset
  every tick. Both signs of inherited I clear; peak state/deadline cannot leak
  into forward mode. Other forward targets remain exactly equal to V11.
- Prior centre/in-place/rolling/acquisition/stable-exit sequences, optical and
  angle/path/time/loss bounds, five-second boundary and controller/PWM peak
  expiry/no-retrigger/overspeed tests pass. Phase9 now expects its own reason.
- Pico W Release, SDK2.2.0, Arm GCC14.2.1; application warnings treated as errors.
  ELF text65944/data0/BSS12232; binary end0x1000f19c below log sector0x101ff000.
  USBstdio init present, UARTstdio init absent. Static record/trace budget checks pass.
- UF2 magic/end markers,242-block sequence/count,256-byte payloads,
  RP2040 family0xe48bff56 and flash ranges verified; padded end0x1000f200.
- No build-host hardware actuation. Physical V8 result pending; regression success
  does not establish sufficient physical alignment speed/torque.

Flash V8,h verify valid1/side_valid1,3,k,h confirm mode3. Retain calibration when
mounting/height are unchanged. Same placement/start sequence and sufficient
outgoing track. Expected1,2,4,9,6,7, then test complete after5s in phase6.
Wait3s after stop/save before switching sources separately; p prints fresh log.
