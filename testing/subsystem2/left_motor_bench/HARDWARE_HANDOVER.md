# Buddy 2 hardware measurements and handover

Purpose: collect the measurements that require the physical robot before it is
returned. Firmware development can continue later, but closed-loop tuning and
full robot acceptance still require access to the robot.

## Evidence already supplied by the user

These observations come from serial logs pasted in the chat, not hardware
operated by Codex. They are limited to the wired LEFT motor on M1.

| Check | Evidence / result |
|---|---|
| USB firmware and console | User opened Pico USB serial COM13 in PuTTY. COM number may change on another computer. |
| Encoder wiring | Initial GP0 connection was wrong; after correction both channels counted. |
| Manual encoder rotation | A/B each reached 2397 changes; invalid stayed 0; net count changed in opposite directions. |
| Powered direction A (`f`) | Version 1 drive=1; net -2360 to -3522, including settling. Physical chassis direction still needs recording. |
| Powered direction B (`r`) | Version 1 drive=2; net -3522 to -2370, including settling. Physical chassis direction still needs recording. |
| Original pulse timeout | Version 1 returned drive off while GP21 remained held; exact electrical latency not measured. |
| GP21 release | Version 2 reported `STOP: GP21 released`, run_ms=526, drive=0. |
| GP20 stop | Version 2 reported `STOP: GP20 pressed`, run_ms=265, drive=0, while GP21 remained held. |
| Invalid transitions in the supplied powered samples | 0. This does not establish capture reliability at all speeds. |

## Highest priority remaining hardware work

1. **Record the hardware and wiring.** Photograph all six motor connections,
   M1/M2 labels, encoder GPIO connections, supply arrangement and any motor model
   markings. Record which physical wheel is under test. Confirm electrical supply
   and encoder output ratings from documentation when available; successful
   motion alone is not an electrical-rating check.
2. **Counts per wheel revolution.** With drive off, mark the wheel and a fixed
   reference. Send `z`, turn the wheel gently for exactly five complete turns in
   one direction, then stop and record the settled `net`, A/B edge totals and
   invalid count. Repeat twice in that direction, then twice in the other
   direction. Reset before each trial. Do not force the gearbox. Divide the
   absolute count by five to get x4 counts per output-wheel revolution.
3. **Physical direction convention.** With wheels raised, record whether `f`
   would propel the chassis forward or backward and the sign of its encoder
   change. Repeat for `r`. Motor polarity and encoder polarity are separate
   software calibration settings. Use chassis travel direction, not clockwise
   viewed from an unspecified side.
4. **Right wheel hardware.** Repeat encoder, direction and stop checks on the
   right wheel. Final intended wiring is M2 GP10/GP11 and encoder GP4/GP5. The
   current v3 firmware can select M1/GP0/GP1 with `L` or M2/GP4/GP5 with `R`.
   It drives one motor at a time. If one motor cable is
   reused with the M1 test, label the result as right motor on M1; that still
   does not verify the M2 driver or final right encoder GPIO path.
5. **Geometry.** Measure effective left/right wheel rolling circumference with
   the robot's normal load (a straight rollout over several full revolutions,
   if feasible), and the lateral distance between the centres of the two tyre
   contact patches. Record units in mm, method and measurement uncertainty.
6. **Bounded drive behaviour.** At explicitly selected duty levels within the
   established motor/supply ratings, collect startup, steady rotation, both
   directions, invalid counts and coast-down data for both motors. Check for
   supply resets and loose connections. No intentional stall tests. Current
   v3 firmware supports 15%, 25% and 40% for at most three seconds; use higher
   duty only after rating/supply checks. Dual-wheel tests require another
   firmware revision and simultaneous wiring.

## Safeguards to check on v3 for each wheel (wheels raised)

- No GP21 held: send `f`; expect rejection and no movement.
- Hold GP21, send `f`, leave GP20 released and keep holding for three seconds:
  expect timeout at about 3000 ms. Keep holding GP21 and send `f` again: expect
  rejection. Release GP21 before another run.
- During a run send `s`; expect a serial-stop message and drive off.
- During a run close PuTTY: firmware should remove drive when USB serial
  disconnection is detected. If that is not detected by the terminal/USB stack,
  the independent pulse deadline must still end the run. Reopen PuTTY, inspect
  the last stop reason, and record what happened. A terminal-close check is not
  identical to physically unplugging USB or proving a particular stop latency.
- Do not use breakpoints or pause the CPU while motor drive is active. The
  hardware PWM can continue while software stop handling is paused.

## Tables to fill with real measurements

### Counts calibration

| Physical wheel | Trial | Direction | Exact turns | Settled net | A_edges | B_edges | invalid | abs(net)/turns |
|---|---|---|---:|---:|---:|---:|---:|---:|
| Left | 1 | forward | 5 | | | | | |
| Left | 2 | forward | 5 | | | | | |
| Left | 3 | reverse | 5 | | | | | |
| Left | 4 | reverse | 5 | | | | | |
| Right | 1 | forward | 5 | | | | | |
| Right | 2 | forward | 5 | | | | | |
| Right | 3 | reverse | 5 | | | | | |
| Right | 4 | reverse | 5 | | | | | |

### Geometry and conventions

| Parameter | Measured value / evidence |
|---|---|
| Left rolling circumference, mm | |
| Right rolling circumference, mm | |
| Wheel contact-centre spacing, mm | |
| Left `f` chassis direction / encoder sign | |
| Right electrical-positive chassis direction / encoder sign | |
| Motor model and supply rating | |
| Encoder supply and output ratings | |
| Final wiring photographs | |

## What still requires robot access after the measurements

Closed-loop speed tuning, straight-distance accuracy, turning-angle calibration,
floor traction/slip, performance on the actual course, dual-motor power behaviour,
fault recovery and the integrated micro T-Kernel timing/ownership/safety path.
Encoder-only tests cannot prove that the chassis travelled the same distance as
the wheel rotation predicts. Software/simulated tests can exercise state changes,
packing, queues and arithmetic without the robot, but cannot replace these
physical checks.

Numeric representation must be sized from the required operating envelope and
resolution, informed by measurements. The largest number seen in a short bench
test is not automatically the maximum legal value for production firmware.
