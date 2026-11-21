# Buddy 5 — obstacle detection, width and bypass

- `test_b5_straight_gate` checks heading qualification and route-turn handling.
  `test_b5_mission` checks detection, stopped ranging, lease ownership, width
  probing, bypass and rejoin. It includes the v0.4.3 borderline range replay.
- Hardware procedures are T2-06 and T2-07 in the [case table](../TRIBUNAL_2.md).
  The [team circuit run](../evidence/tribunal2/run-4/README.md) captures
  detection, a width estimate and return to the original line. The
  [v0.4.3 record](../evidence/tribunal2/v0.4.3/README.md) records the replay
  separately from the physical run.
