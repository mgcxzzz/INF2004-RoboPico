# Analog line-follow V2: floor breakaway adjustment

21 October2026. Built/software-checked, physical improvement NOT confirmed.

User V1 floor log: offline launch and CRC restore successful; only2.22/2.21mm
wheel travel; no-movement stop798ms; invalid0/0. Obstruction check clear.
Earlier V6 straight floor runs at90..100mm/s logged roughly18..20% steady PWM.
Fixed equal PWM would remove the steering/speed feedback needed on curves.

V2 changes only breakaway assistance, plus a startup banner:
-250permille (25%) instead of180, inside existing global25% cap.
-120ms pulse deadline instead of80ms; ends on speed>=20mm/s or wrong direction.
-100ms cooldown before another pulse; demand/ramp>=20 and speed[-8,8) required.
-Zero target cancels immediately. Steady speed uses normal PI PWM.
-Deadline observed on control updates (normally10ms); independent100ms late-loop
  guard still cuts drive. No-movement500ms counter continues across pulses.
-All profiles/speed targets/loss/encoder/button/flash guards unchanged.
-Flash record magic/version/layout unchanged; valid V1 calibration is reusable.

Checks PASS:
-Full Pico W SDK2.2.0/Arm GCC14.2.1 build, application -Wall -Wextra -Werror.
-Host tests: ramp gating,250 cap,120ms deadline, speed exit,cooldown,zero-demand
  cancellation, wrong-direction refusal, unmodified steady PI and irregular
  control intervals; existing analog calibration/steering/recovery/guard checks.
-UF2 RP2040 blocks/headers/addresses below reserved flash-log sector.
-Source: PWM cap assertion, stall threshold and active/generation commit retained.
-ELF text54456/data0/BSS12044. UF2 100864 bytes.

SHA256 buddy23_analog_line_follow_v2.uf2:

```
350f76fd29557c2cbbabd787eb198b938716bd6dd7343890ebd7977ed0fa0c85
```

First physical retry: verify h valid1/profile1, normal-height left edge level500
and right-white, bank-only deliberate start, short150mm/3s profile. Same p log
retrieval after3s stop settling and non-overlapping power-source switch.
Repeated stall requires examining load/power/drive response rather than raising
PWM beyond the existing cap. Physical logs, not this build, determine success.
