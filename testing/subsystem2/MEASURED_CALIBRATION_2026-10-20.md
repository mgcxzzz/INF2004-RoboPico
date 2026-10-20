# Buddy 2 measurements supplied in the chat, 20 October 2026

These are user-operated bench observations, not hardware tests run by Codex.
Tables transcribe supplied snapshots/logs. Keep original PuTTY logs as primary
evidence; a missing trial must not be counted as a pass.

## Geometry and encoder counts

Each encoder trial was reported as five manual output-wheel revolutions, starting
after a reset. Decoder counts all four transitions of each quadrature cycle.

| Wheel | Forward 1 net | Forward 2 net | Backward 1 net | Backward 2 net | Mean absolute counts/revolution |
|---|---:|---:|---:|---:|---:|
| Left | -12632 | -12618 | +12606 | +12625 | 2524.05 |
| Right | +12733 | +12566 | -12641 | -12626 | 2528.30 |

All eight supplied snapshots had invalid=0. Right-wheel five-turn totals span
about 1.3%; these means are provisional measurement-based parameters, not exact
manufacturer encoder specifications. Do not silently round to an assumed gearbox
ratio. Small manual reversals, mark alignment and capture errors need calibration
against external travel later; invalid=0 does not prove no missed pulses.

- Left rolling circumference: 200 mm reported twice.
- Right rolling circumference: 200 mm reported for one revolution.
- Wheel contact-centre spacing: 110 mm reported with ruler.
- No measurement uncertainty or ruler photo supplied yet.
- Supply label previously reported: 5 V/2.1 A. Actual motor model/current/supply
  rating and encoder electrical ratings are still not established by documentation.
- User confirms the Robo Pico and Pico remain powered by the power bank when USB
  is unplugged.

Forward polarity: left uses A-side PWM (`f` in v3), raw encoder count negative;
right uses B-side PWM (`r` in v3), raw encoder count positive. Reverse is opposite.
V4 normalises both wheels to positive=chassis-forward in its reported movement.

## Raised-wheel drive results

Three-second runs unless noted. Cutoff count is signed raw movement while drive
was applied; settled movement also includes coast. Speeds are approximate selected
steady windows, not precisely identified system-response models.

| Wheel / physical direction | PWM | Raw ticks at cutoff | Net movement including settling | Approx steady counts/s magnitude |
|---|---:|---:|---:|---:|
| Left forward trial 1 | 25% | -6841 | -6927 | 2322-2334 |
| Left forward trial 2 | 25% | -6916 | -6999 | 2349-2362 |
| Left backward | 25% | +6703 | +6790 | about 2280 |
| Right forward earlier outlier | 25% | +4848 | +4916 | uneven acceleration; cause unknown |
| Right forward repeat | 25% | +6576 | +6646 | 2230-2238 |
| Right forward during no-retrigger check | 25% | +6568 | +6639 | about 2230 |
| Right backward | 25% | -6542 | -6612 | about 2222 |
| Left forward | 15% | -3722 | -3761 | about 1270 |
| Left backward | 15% | +3581 | +3619 | about 1190-1240 |
| Right forward trial 1 (nonzero baseline) | 15% | +3492 | +3522 | about 1190 |
| Right forward trial 2 | 15% | +3462 | +3492 | about 1175 |
| Right backward | 15% | -3570 | -3600 | 1207-1211 |

Supplied rows report invalid=0. Third planned left-forward repeat at 25% was
explicitly **skipped/not run**. Earlier right 40% runs exist in a supplied log,
but were not developed into a rated operating envelope and are not required by v4.
All tests above were raised-wheel tests; they do not establish floor-load speeds,
traction, rated current, stall current, or achieved robot distance/angle.

## Manual safeguard evidence

- Left v2 GP21 release: STOP at run_ms=526.
- Left v2 GP20 stop: STOP at run_ms=265, GP21 still held.
- Right v3 GP20 stop: STOP at run_ms=413, cutoff ticks=+796, invalid delta=0.
- Right v3 GP21 release: STOP at run_ms=310, cutoff ticks=+567, invalid delta=0.
- Right v3 serial s: STOP at run_ms=416, cutoff ticks=+810, settled net=+880.
- Right v3 no-hold command: rejected; user confirmed no movement.
- Right v3 same-hold retrigger: three-second run ends, GP21 remains held,
  second command rejected, counts settle at +6639.

Run durations are NOT measured button-to-drive-off response times. Firmware
messages are software evidence; external timing has not been measured. These
results do not validate v4's new controller, offline mode or the final RTOS path.
