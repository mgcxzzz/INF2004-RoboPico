# Analog line-follow V9 - 21 October 2026

Compiled and software-verified. New symmetric-placement hardware test pending.

## Profile6

For physical left/right spots spanning the strip, BOTH BLACK is normal. Error
is normalizedL-normalizedR; strict+/-80 deadband, correction15*error/1000mm/s.
Targets45-correction/45+correction: more left black turns LEFT, more right turns
RIGHT. Bounds30..60mm/s. Balanced near1000/1000 gives45/45. No centre500 target,
no both-dark crossing stop. Saturated equality cannot prove exact strip centre
or detect junctions. No derivative term added.

Both-white<150: only clear asymmetry (difference>=200 and one level>=400)
within500ms may select recovery. Latched recent side drives30/60or60/30 for at
most650ms OR25mm average absolute wheelpath. No direction/stale direction stops
immediately; optical return resets loss state. Start requires one level>=650
plus valid calibration and normal mode/button/rearm gates. White start refused.
Profile6 bounds5s/200mm;75ms traces keep the full run plus settling within100rows.

PI,25% PWM cap, kick and all motion guards retained. Old profiles1..5 retained
for history but incompatible with the new physical placement. Calibration shared;
old calibration may load, verify actual home surface and mounting before motion.
Evidence from the user's stationary test is saved under ../logs/.

## Verification passed

- Host C11 -Wall -Wextra -Werror: both-black remains straight beyond120ms;
  left/right sign and mirror symmetry, noise deadband, actual user-calibrated
  samples, recent/stale/unknown recovery direction, exact650ms/25mm stop bounds,
  contact reset/direction change, weak-signal memory refusal, invalid calibration,
  symmetric start/offset/refusal gates. Prior profiles/PI/kick/safety checks pass.
- Pico W SDK2.2.0, Arm GCC14.2, Release, application strict warnings.
- ELF text63760/data0/BSS12092. UART disabled, USBstdio enabled.
- UF2 119808 bytes; RP2040 family/headers/sequence and flash payload bounds
  below reserved0x101ff000 verified. Trace/SavedLog structure/version unchanged.
- Old optical/response/kick functions and wheel_control.h unchanged in source.
  New controller selected only for6, reset on every run; preflight and actual
  start gates retained; independent duration and main distance limit checked.
- No hardware attached on build host; no claim of smooth driving or curve success.

Saved selected-profile range now includes5, recorded-profile byte6. V9 reads
previous analog records. Older versions may reject selected OR recorded profile6;
selecting another profile alone does not clear a saved trace. Record calibration
values before downgrading; recapture on old firmware if required.

SHA256 buddy23_analog_line_follow_v9.uf2:

```
28f6b29407af08264dfa0d8df8bd79f3aac9069eb64823e52f0a12813440466b
```

First test: short centred straight only, BOTH spots black. h verify V9/valid1;
6 then k; normal load/height; one power source. Offline BOTH3s/release/wait1s/
GP21tap-release/2s countdown, either button stops. Wait3s before switching power,
p to save full log. Observe steering separately from startup stutter. Later
small RIGHT/LEFT offsets on same straight test LEFT/RIGHT correction respectively;
these cannot establish continuous-curve following.
