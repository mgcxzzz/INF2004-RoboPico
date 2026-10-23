# Circuit test observations — 23 October 2026

## V1 profile 1: reaches first cross, turns, loses outgoing curve during alignment

Full user log: `logs/v1_profile1_curve_after_left_2026-10-23.txt`.
CRC restored; 118 rows, none overwritten; no invalid encoder transitions.
Elapsed 28.422 s; distance 997.9 mm; average encoder speed 35.11 mm/s.

- Phase 10 follows approximately 803 mm over the first 20.3 s.
- First logged phase 2 / side black is 20.563 s, distance 814.4 mm.
- First logged phase 4 is 22.317 s; last pre-turn row at 22.067 s.
  Left remains approximately 944.5 mm while the right advances through the turn.
- First logged phase 9 is 25.830 s: front levels 938/2, targets 21/34.
- At 27.334 s levels 478/467 briefly satisfy the instantaneous balance condition.
  The 250 ms sampling cannot establish whether it held for the required 80 ms.
  No turn-completion event means that continuous condition did not qualify.
- 27.584 s levels 0/526, 28.335 s levels 0/15, cutoff 28.421 s phase 8,
  `line not recovered`. The separate phase-9 both-white guard is 150 ms;
  the six-second alignment deadline had not expired.
- `completed_lefts=0` means alignment did not qualify, not that no physical
  left rotation occurred. A precise turn-origin angle is absent from the saved
  rows; do not present an exact externally measured 90-degree result.

User video `video_2026-10-23_13-43-43.mp4` (11.58 s clip) shows the rolling
left, then the front sensor pair reaching the bottom loop's curve before the
robot stops. The clip begins partway through the run; its seconds are not the
log's elapsed timestamps. This supports the interpretation that V1's slow
straight-alignment mode is unsuitable for this short outgoing straight. The
video does not independently measure sensor levels, turn angle or slip.

V2 therefore uses the usual full adaptive curve follower immediately after
the existing optical branch confirmation. A continuously visible branch for
150 ms completes handover; exact two-sensor balance is no longer required.
The standard line-loss policy remains bounded (650 ms / 25 mm with recent
direction memory). There is no general six-second both-white carry in this test.

## V1 profile 2: repeated opposite corrections and line loss before any cross

User supplied a complete pasted log: 37 rows, none overwritten, phase 10
throughout until fault. Side raw stays 0; ignored edges 0; turn summaries 0.
Elapsed 8.036 s; distance 367.1 mm; average encoder speed 45.67 mm/s;
invalid encoder transitions 0/0. Selected exact samples are preserved below.

| Elapsed ms | Front levels L/R | Requested L/R mm/s | Observation |
|---:|---|---|---|
| 0 | 1000/319 | 30/96 | Starts with an asymmetric front observation |
| 502 | 1000/57 | 4/90 | Strong left correction |
| 753 | 138/885 | 94/24 | Opposite right correction |
| 1506 | 598/0 | 40/98 | Left correction again |
| 1756 | 96/472 | 100/66 | Right correction again |
| 2258 | 229/282 | 90/90 | Low but balanced readings still triggered V1 fast speed |
| 2508 | 25/0 | 0/45 | Both-white recovery |
| 3008 | 867/0 | 12/92 | Reacquired asymmetric line |
| 3259 | 0/747 | 94/24 | Opposite correction again |
| 7270 | 217/86 | 86/94 | Weak signal before final loss |
| 7520 | 0/0 | 45/0 | Final both-white recovery |
| 8036 | 0/0 | 0/0 | `line not recovered` cutoff |

At error -747, profile 1 requests 47/12, whereas V1 profile 2 requested 94/24.
V1 scaled both wheel targets, preserving a speed ratio but increasing the yaw
rate and forward travel before the controller could correct. Wheel response
also lags the rapidly changing requests; for example the 4.011 s sample has
measured 92.4/40.0 mm/s while its new request is 34/96. This supports the user's
reported left/right swerving; it does not prove that speed scaling is the only
physical contributor.

V2 uses profile 1 correcting targets at every error for all profiles, with a
profile 1 forward controller and 30% correction cap. Higher cruise speed needs
both levels >=400 and difference <=80 for 300 ms. Losing that qualification
removes the cruise boost and clears cruise compensation. Actual slowing still
depends on inertia, load, grip and the encoder PI response.

## V2 validation status

Host simulations and UF2 build/structure checks passed. Physical V2 results
are pending. First repeat the profile 1 junction-and-loop test to check branch
handover; then repeat profile 2 to check steering and whether stable sections
earn fast cruise. Request the complete new saved log after each comparison.

## V2 profile 1: optical handover succeeds, right wheel then stalls

User supplied 54 rows, one turn summary, no overwrites. First cross at about
6.77 s; left turn / capture completes at 11.846 s, 390.5 mm, estimated 89.1
degrees, handover 160 ms. By 12.036 s both encoder positions stop advancing:
L312.4 / R470.3 mm. At 12.286 s targets 1/45 mm/s, actual0/0, PWM43/300,
levels974/2. Cutoff12.421 s is `no encoder movement under drive`.
The left wheel's low speed is intentional. The commanded outer right wheel
is the feedback concern. This is not a both-white line-loss timeout.

## V2 profile 1: moving through weak right-only readings, then immediate line loss

Another complete log: 56 rows, one turn; 12.805 s, 420.3 mm, average32.82 mm/s,
no encoder errors. Turn completes10.952 s, 362.1 mm, estimated91.1 degrees,
handover159 ms. Both wheels keep moving after capture; user confirms right
wheel rotation. At11.780 s levels160/559;12.031 s14/287;12.281 s0/305;
12.532 s0/309;12.782 s0/234;12.805 s0/120 and `line not recovered`.

V2 only refreshes recovery direction with difference>=200 and a level>=400,
and retains it for500 ms. The sampled weak interval is longer than that.
The last active sample is only23 ms before cutoff, so a full650 ms white
recovery attempt is incompatible with these rows. The source and sample
pattern support stale recovery memory causing an immediate no-direction stop.
Exact intermediate raw readings are not available. The regression test feeds
this strong-to-weak-to-white pattern to both old and new policies: V2 stops,
V3 requests right recovery45/0.

## V2 profile 2: full left black, outer right wheel advances in bursts then stops

Complete pasted log:48 rows, one turn,10.837 s,308.7 mm,average28.48 mm/s,
invalid transitions0/0. Captures branch9.914 s,306.9 mm,estimated88.1 degrees.
At9.780 s targets13/47, measured1.3/34.3, PWM42/107;10.030 s5/46 with
measured0/0.1, PWM53/202. Right position389.5->392.3->392.9 mm, then unchanged
from10.533 through10.837 s. At cutoff left level935, right0; targets3/45
immediately beforehand, rightPWM299. STOP is encoder movement, not optical loss.

User saw jerking and wheels moving. This does not establish that the right
wheel rotated continuously during the final flat encoder interval. V2's guard
also discards each10 ms interval with fewer than two encoder transitions, so
sustained slow/bursty movement can be misclassified as a stall.

## V3 response and pending physical validation

- Directed recovery2 s/50 mm; memory1.5 s. Weak asymmetric level>=200,
  difference>=150 must remain same sign30 ms to refresh it. White never
  chooses a direction or renews the deadline. Stop reasons distinguish expiry,
  distance and absent recent direction.
- For up to3 s/45 mm after branch sighting, visible near-pivot corrections
  retain turn feedforward only on the outer wheel,42% cap, no new48% pulse.
  Targets/ordinary correction thresholds stay the same. Inner effort is normal.
- Progress monitoring accumulates four FORWARD ticks (~0.32 mm) instead of
  discarding small individual samples.750 ms with no such progress stops.
  Tests accept slow/bursty advancement and reject true zero progress/jitter.
- CSV adds assist0/1/2 (none/left/right) without changing flash record size.

V3 software/build tests pass; physical results pending. A real wheel rotating
steadily while logged distance is flat still warrants checking encoder signal,
wiring and coupling. Relaxing a guard cannot restore missing encoder pulses.
