# Integration V4: offline arming diagnostics

20 October 2026. Hardware validation pending.

V3 physical evidence: USB stable >60 s and h responsive; raised straight and
left/right steering worked; GP21 and GP20 stopped drive. Offline attempts did
not create a new saved run. The exact cause is NOT established.

V4 changes:
- One retryable offline arming state machine, with both buttons debounced20 ms.
- Both held3 s, full release, separate GP21 tap/release,2 s countdown.
- Cancellation/rejection does not permanently consume the arming attempt.
- Saved button/state/USB/rejection history on accepted runs and idle failures.
- One diagnostic save per boot after5 s idle following button activity, or30 s
  if none seen. Successful motion later replaces the diagnostic. Flash format2.
- Original steering/control/IRQ paths and hardware limits retained.

Checks PASS:
- Host: full offline hold/release/tap/countdown, both release orders,
  cancellation/retry, no repeated start, USB/busy refusal, timer wrap;
  real S3 target scaling, stale timing, flash size/CRC.
- Full real RTOS build, application -Wall -Wextra -Werror.
- verify_build.py: strong real MC/nav/S3 interfaces, larger idle/tmp stacks,
  raw cutoff without kernel/FP calls, SRAM flash writer, unchanged upstream
  nav/S3, adapter/build agreement, UF2 below reserved log sector.
- ELF text76904, data0, BSS30288. UF2 154112 bytes.
- Known RWX linker warning from executable SRAM flash routine remains.

SHA256 buddy23_line_motion_v4.uf2:

```
1170162c2979f13addc320934711090f17a4d287e838ab9d065794e429723477
```

First test: reboot on power bank ONLY with wheels raised, perform the sequence.
If no movement, release both and wait10 s before switching bank off / computer
USB on and pressing p. Never overlap supplies. See README for ARM field meanings.
