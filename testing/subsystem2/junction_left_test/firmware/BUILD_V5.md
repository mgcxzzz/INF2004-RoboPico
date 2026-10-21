# Junction left V5 — 22 October 2026

Artifact buddy23_junction_left_v5.uf2:118784bytes,232blocks.

SHA256:

```
9a63e24dc309a882c7959be698f0f3b0000ac91d88b187345d7513f060f2fce4
```

User accepted a rolling corner after V4 left reverse stalled. Select MODE3, not2.
Left motor OFF/coast, right forward. Axle turn-start goal = measured stripe
midpoint+92mm-55mm; 55mm is half provisional110mm wheel spacing. Ideal path
ends55mm along the outgoing left branch at90deg; right arc172.8mm. Not an
axle-centred turn. Front127mm ahead means outgoing black needs>=220mm from cross.

Mode3 flows from position phase2 to turn phase4 without a full stop. Right speed
ramp is preserved; previous forward integral is cleared when higher loaded turn
feedforward activates, preventing double compensation. Left0 resets controller
state. Same V4 effort caps35% sustained/one40% nominal150ms peak, only on right;
peak skipped if moving>=25mm/s, expiry enforced in commit and1ms ISR. Forward
phases still<=25%. No power increase in V5.

Right target45 then30mm/s near65degrees. Need to leave old line, then both front
levels>=650 for30ms at75..110degrees. Inner drift<=12mm (motor off is not locked),
outer arc<=240mm, turn<=9s for longer path. Same500ms no-motion-under-demand,
button/direction/invalid/late-loop kills and25s/900mm whole-run. Mode1/2 retain
their earlier geometry and6s/120mm limits. Settle on branch then V11 follow20mm.

Verification passed:

- Strict C11 host suite (-Wall -Wextra -Werror -O2) including prior in-place,
  centre-only, guard, loaded PWM/pulse/no-retrigger/overspeed/forward equivalence tests.
- Mode3 ideal sequence:55mm early goal within10ms sampling tolerance, no pre-turn
  full stop, left request/output0, successful branch+exit and missing-branch stop.
- Deliberate unequal arcs permitted in rolling mode; inner drift>12mm and9s
  timeout stop. Ideal90deg rolling kinematics verified from110mm wheel spacing.
- Moving-right transition retains30->31 ramp, clears old integral80, and skips
  forced peak at measured30mm/s; no duplicate forward+turn feedforward compensation.
- Actual V4 Junction/SavedLog typedefs from commit0857e76 compared with V5:
  sizeof3792/row offset192/Trace36 unchanged. New rolling flag fits padding.
  Nonzero legacy padding tested with CRC preserved and normalized to false before
  access; front/side calibration and old targets retained, new mode3/revision5 stored.
- Pico W Release SDK2.2.0, Arm GCC14.2.1, application -Wall -Wextra -Werror.
- ELF text63304/data0/BSS12232; end0x1000e74c below log sector0x101ff000.
- UF2 header/sequence/family0xe48bff56/payload lengths/flash addresses verified
  below log sector. USBstdio present; UART init absent. No hardware actuation here.

Flash,h verify V5 and valid1/side_valid1, **3,k,h confirm mode3**. Same mounting,
calibration and bank buttons. Extend outgoing line>=220mm before attempting.
Starting the corner before axle reaches cross is intended. After stop/save, wait
3s and switch power sources separately, p; fresh run must show V5/test3/rolling.
Old restored trace keeps old version/mode; V5 physical result remains pending.
