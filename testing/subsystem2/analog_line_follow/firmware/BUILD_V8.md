# Analog line-follow V8 - 21 October 2026

Software-verified; left-curve physical validation pending.

## Separate profile5

Normal V6 edge following; both-white invokes left0/right45mm/s target rather
than30/60 shallow arc. Left output coasts atPWM0 (no active brake/reverse).
Recovery bound1200ms OR30mm average absolute wheelpath; decision10 during recovery,
AL_LOST4 and zero outputs on expiration. Normal edge returns on optical contact.
Assumes missing strip is LEFT; does not solve all-white direction ambiguity.
Both-dark/crossing and motion guards retained. Total600mm/12s, samples150ms.
Other profiles1..4 remain unchanged, including V7 experimental black-straight.
The V6 evidence is saved under ../logs/v6_left_bend_loss_2026-10-21.txt.

## Checks passed

- Host C11 strict warnings:650ms no longer prematurely stops ONLY profile5;
 1200ms/30mm boundaries, contact timer reset, both-dark stop, invalid-cal refusal,
  left-zero PI/integral/kick reset, profile5 start gate; prior profile4/edge/
  response/PI/kick/rearm/button/guard tests.
- Pico W Release SDK2.2.0 Arm GCC14.2, application -Wall -Wextra -Werror.
- ELF text61272/data0/BSS12068; USBstdio enabled/UART disabled.
- UF2 114688 bytes; valid RP2040 family/headers/sequence, payload below reserved
  flash0x101ff000. Stored Trace/SavedLog structure/version unchanged.
- Source comparison: V6 edge, V7 black-straight, diagnostic, PI, breakaway unchanged.
  New profile5 only dispatched when selected5; overall duration/distance and
  active/generation/button/USB/encoder/no-motion gates verified present.
- Saved selected-profile range extends to4, recorded byte5; earlier data loads.
  Select1 then k before downgrade to firmware that rejects profile5 records.
- No actual robot at build host; no claim of successful or smooth curve following.

SHA256 buddy23_analog_line_follow_v8.uf2:

```
f9a40caa547a8aa50a3a1d23f1d742fec7cb73f5e28101f7f64e0838790a3d1b
```

Finish/save the ongoing V7 profile4 result first. Then h confirm V8/valid1;
5 then k; left edge450..550/rightwhite, just before a gentle LEFT bend with mat
throughout route. ComputerUSB OFF, bank ON; wait5s,BOTH3s,release,wait1s,GP21tap/
release,2s countdown. Either button stops. After stop wait3s,bank OFF,USB ON,p.
Send full log and observation/video of line recovery.
