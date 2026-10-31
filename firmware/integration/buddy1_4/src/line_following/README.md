# Line following — Buddy 3

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Inputs and decisions

The two front analog sensors are GP26 and GP28. Stored white/black endpoints convert each reading to a clamped 0–1000 reflectance level. These levels describe reflectance, not physical lateral distance. A both-black plateau cannot identify a unique centre. Front calibration is independent of barcode GP27 acquisition.

The optical tracker in `symmetric_logic.h` uses the signed level difference, a straight dead zone and piecewise interpolation. Representative inner/outer targets are 45/45 at absolute error80, 40/50 at250, 25/50 at500, and0/45 at1000. This avoids jumping directly between two PWM values. `circuit_recovery.h` retains direction memory and explicit line-loss states.

`continuous_follow.h` runs the original optical/barcode state machines once per control tick. `continuous_steering.h` then adapts ordinary visible-line wheel targets:

1. On entry, seed mean speed from measured travel and steering difference from zero.
2. Begin with70% steering gain; sustained requests to the same side progressively restore full gain by350ms, retaining bend authority.
3. When optical error is returning toward centre, use bounded encoder-derived yaw damping to reduce overshoot. Damping fades as that evidence disappears.
4. Slew mean speed and steering difference, then emit non-negative per-wheel targets. This smooths correction transitions without locking the heading after a bend.
5. Yield the adapter during explicit stops, lost-line recovery and manoeuvres. Profile2 retains its legacy ramp experiment; profile1 is the normal test profile. Profile3 can earn up to135mm/s after the original centred-stability test, rather than always driving at135.

Barcode activity caps ordinary scan travel at45mm/s. A short both-white interval retains the original150ms bridge; persistent loss has a2s confirmation path before bounded sweep search. Sweep time/distance budgets are independent; transient flashes cannot renew them indefinitely. See `line_search.h` and `circuit_recovery.h` for the exact stages.

## Interfaces and ramp interaction

The application adapter submits Buddy3 wheel targets through `nav_submit_navigation`. The coordinator forwards them only in AUTO; a Buddy5 lease cannot be overwritten. Motion owns PI/PWM, not this feature. During an approved ramp hold the motion controller may raise a low inner-wheel target toward35mm/s; that deliberately reduces steering authority. The dashboard shows the requested and constrained targets separately.

Calibration, tracker decisions and Continuous V2 steering constants were preserved from the combined baseline. Mounting changes can change behavior even when this source is unchanged. See the calibration guide before using `c`: it restores the old built-in low-height preset, not the teammate's later tilted-front capture.

## Read and test

Start with `line_logic.h`, `symmetric_logic.h`, `circuit_recovery.h`, then `continuous_steering.h` and `continuous_follow.h`. Tests cover9,000 routing/PI parity ticks, bends, both-white recovery, profiles and adapter damping. Historical successful straight-line and earlier wheel-response logs are context; new physical flat/bend acceptance remains pending.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
