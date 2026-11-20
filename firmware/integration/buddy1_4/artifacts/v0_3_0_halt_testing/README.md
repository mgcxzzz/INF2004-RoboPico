# RoboPico v0.3.0 halt testing

[Download the v0.3.0 UF2](robopico_buddy1_5_rtos_v0_3_0.uf2) and use the
[v0.2.13 dashboard](../v0_2_13_combined/). Confirm the car's firmware badge
reports **v0.3.0** after flashing. The older bundle also contains its own UF2,
so use the v0.3.0 download on this page for this test.

## What to test

1. Use **Baseline · 45 mm/s** and the same obstacle placement as before.
   Set the car far enough back to follow 200 mm of straight line before the
   sonar reaches the 330 mm confirmation range.
2. On a qualified straight, three fresh echoes within 330 mm qualify the
   obstacle. The first qualifying echo at 295 mm or closer requests a stop.
   The probe then checks the range at rest. On a bend, three close echoes at
   300 mm or less still cause a hold without starting a bypass.
3. Check whether it gets through the first angled scan and eventually rejoins
   the line. If it halts, press **Stop** promptly. Leave power on, wait for the
   coordinator to show Off, and use **Download run ZIP**. The ZIP captures the
   stopped width-policy range and the probe's reference and angled echoes.

The simulations passed both bypass directions. The staged configuration
stopped at a 293 mm reference in the two opposite-side obstacle simulations.
The ARM build, flash and memory checks, C standard checker, and pinned
formatter passed. The stopped range and width probe still need physical tests.

## Build and check

Build from the repository root in Git Bash on the project Windows build host.

```sh
bash tools/rtos_migration/build_release.sh APP=rtos_buddy1_5
```

Run the focused host suite from the repository root.

```sh
python tools/rtos_migration/run_tests.py --only test_b5_mission test_b5_straight_gate test_connected_bridge test_release_exports --skip-python
```

- [Confirmation and stop logic](../../../../../app_program/obstacle_navigation/b5_mission.c)
- [Straight-track qualification](../../../../../app_program/obstacle_navigation/b5_straight.c)
- [Width probe](../../extensions/buddy5_obstacles/width_probe.c)
- [UF2 SHA-256](SHA256SUMS.txt)
