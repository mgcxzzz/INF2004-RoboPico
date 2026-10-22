# Physical evidence and next measurements

## Reported V9 small-hump failure, 2026-10-22

- Only the robot's right drive wheel was on the hump; the approach was straight.
- User saw it climb about halfway up the small black fixture, then stop.
- Small black height: 10 mm. Orange height: 23 mm. Each reported horizontal
  uphill run is about 4 inches (101.6 mm). Actual course dimensions may vary;
  these measurements describe the fixtures and do not configure the control.
- Front IR housings scrape the mat in the current working mounting. The user
  previously confirmed the front support ball rolls freely.

Selected rows from the complete user-supplied V9 log:

| ms | L/R distance mm | L/R speed mm/s | L/R PWM permille | L/R target mm/s | L/R normalized IR |
|---|---|---|---|---|---|
| 1503 | 32.96 / 34.33 | 35.5 / 35.9 | 131 / 182 | 45 / 45 | 922 / 927 |
| 3506 | 96.35 / 97.61 | 19.0 / 17.1 | 224 / 202 | 49 / 21 | 158 / 722 |
| 5008 | 124.64 / 125.93 | 10.8 / 20.8 | 250 / 211 | 50 / 27 | 1 / 472 |
| 5509 | 127.81 / 129.65 | 0.0 / 0.2 | 250 / 250 | 50 / 30 | 1 / 428 |
| 5778 | 127.81 / 129.65 | 0.0 / 0.0 | 0 / 0 | 0 / 0 | 1 / 399 |

Stop: `no encoder movement under drive`, run 5777 ms, invalid transitions 0/0.
All rows were phase 1 (approach); no junction was detected or executed.

Meaning: at least one wheel commanded to move produced fewer than two encoder
ticks per control update continuously for about 500 ms, so the existing
feedback guard removed drive. Both recorded wheel positions froze here.
250 means 25% PWM duty in this firmware, not full output. The physical report
and zero invalid-edge counts support a real load/stall interpretation, but
do not establish available supply current, motor current, traction, or the
single mechanical cause. No voltage/current readings were supplied.

The left normalized IR also fell from black to nearly white while the right
was intermediate. A one-wheel hump does not guarantee unchanged front sensor
height/contact or line position. Increased PWM alone may not fix that.

## Hump V1 experiment

Hold line targets unchanged and compare baseline 25% against an optional
bounded per-wheel load response capped at 35%. Log extra effort, actual
speeds, targets, final PWM, both ADCs, normalized levels, and stop reason.
First verify a flat straight. Then test the small black hump, noting:

1. Does the right wheel continue rotating, or spin without moving the chassis?
2. Does the left wheel remain moving? Does either front housing catch the mat?
3. Do extra effort and PWM rise before the stall cutoff?
4. Do front readings still provide useful steering across the climb/crest?
5. Does speed recover and effort reduce after returning to the flat?

Software compilation and simulated checks do not answer these physical
questions. No hump V1 physical result has been received yet.
