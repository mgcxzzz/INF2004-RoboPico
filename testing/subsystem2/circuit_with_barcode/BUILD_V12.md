# Circuit with barcode V12: faster U-turn and stronger delayed recovery

Built 2026-10-25 for Pico W/RP2040, Pico SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, C11 with -Wall -Wextra -Werror.

- Artifact: firmware/buddy23_circuit_with_barcode_v12.uf2,241152 bytes,
  471 sequential RP2040 UF2 blocks.
- SHA256: c47fe618365ec17640b5ba3b10d803b38f7c8949a885483170872ab8337f66d6
- Padded end0x1001d700; linker end0x1001d69c.
- ELF text124568/BSS215376; BSS end0x20035c90, stack limit0x20040000;
  gap between those symbols41840 bytes.
- Main record remains64848 bytes with the V11 extension and historical
  64840-byte prefix; versions1-12 load, unknown13 rejects. Raw format/banks
  and trace56/summary40 layouts are unchanged.
- UF2 sequence/family/payload/addresses/trailer/markers and package/build
  equality pass verification. Published V3-V11 hashes are unchanged.

## Behaviour

Z uses forward90/55/35mm/s with angular taper at60/82 degrees, reverse
80/45/30 with taper at150/170, and120ms measured-still pauses(both speeds<5).
Its two-quarter geometry,165..195 return-line window, prior departure and
optical confirmation remain. Ordinary A/F turns retain their prior targets
and controller. Visible-line smoothing, calibration, cruise and decoding
remain unchanged. Profile2's existing ramp experiment is retained separately.

maneuver_power.h applies only to Z turning or an active search wheel. It uses
physical600 sustained/750 start PWM, no20% re-scaling, Kp2.2/Ki6, bounded I,
300mm/s^2 acceleration and700mm/s^2 reference deceleration. The start pulse
ends at180ms or measured along speed min(50mm/s,80% of goal). It cannot
continually retrigger during one stalled powered stage. Zero immediately
clears output/state. Overspeed coasts and positive I unwinds. The control
commit and independent1ms ISR share the same signed750/600 envelope. Search
levels disallow negative drive; only Z retains signed output mapping.

Both-white recovery retains the short150ms bridge, then coasts during a
2s continuous-white qualification. Below150 starts the timer,150..199 holds
its classification, and either sensor>=200 resets the pre-search timer.
Confirmed return uses the existing40/80/120ms confidence bands. Once active,
the separate search budget cannot be reset by flashes. The five alternating
rolling stages retain350/700/1050/1400/1750ms timing but request90mm/s,
45 while confirming; outer-wheel power uses the new maneuver controller.
The first side is last reliable direction, or left with no history.
Search ends on confirmed return,5.25s or300mm. Outgoing alignment allows
9s/360mm so a full wait/search can fit. Non-barcode builds without
CIRCUIT_SWEEP_SEARCH retain their original6s/180mm alignment behaviour.

j adds RECOVERY_WAIT/SWEEP/FOUND/FAILED with compact timing/direction/reason
evidence. Pending route commands are preserved and junction acceptance is
blocked while lost. Expected white during planned turning still uses turn
geometry/angle guards rather than ordinary search. Existing manual/USB/
late-control/direction/invalid-encoder checks remain.

## Executed validation

- test_maneuver_power: both wheels/signs;750 start;180ms ISR expiry;600
  sustained cap; no repeatedly renewed stalled kick; early cutoff with motion;
  overspeed coast/zero-off; signed mapping; search reverse prohibition;
  normal-controller state/output parity when inactive.
- test_line_search: exact2s boundary;150ms bridge then coast; short return
  resets pre-search confirmation; hysteresis; five mirrored stages; independent
  post-start time/travel bounds; weak flashes; opposite-side acquisition;
  pending command retained; no cross while lost; compact recovery events;
  outgoing alignment permits search beyond its previous180mm cutoff.
- test_uturn: complete Z at141/282mm in both directions, ignored90-degree
  branch, reverse quarter and rejoin, next crossing after reverse, missing
  return/time/drift/settle guards and record versions1-12. Ideal fixture
  turn-plus-settle duration5320ms, pose x45.7/y0.7mm and177.9deg at confirmation.
  A fixture with finite wheel response reports5560ms, x48.8/y2.1mm,178.3deg.
  These are software models, not measured robot performance or torque claims.
- test_barcode: captured A replays, decoder/queue/CRC and12 alternating A/F
  turns per profile. Also compiles/runs without CIRCUIT_SWEEP_SEARCH to check
  the shared-header fallback and historical behaviour.
- test_analog, test_start, test_front_calibration, test_barcode_raw,
  test_barcode_sensitivity, test_smooth_follow and test_ramp_profile pass;
  six Python raw-dump parser cases pass.
- Pico W build and packaged UF2 verification pass. Optional WiFi/BLE SDK
  modules remain unused by the standalone USB target.

First physical test: profile1, same mounting/calibration, scan Z at a clear
cross then send j. Recovery can then be checked on clear floor; expect the
coasting wait followed by stronger alternating arcs and return to following.
Physical fast-turn/recovery success has not yet been claimed.
