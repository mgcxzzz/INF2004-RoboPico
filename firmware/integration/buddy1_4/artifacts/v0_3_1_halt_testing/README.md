# RoboPico v0.3.1 halt testing

[Download the v0.3.1 UF2](robopico_buddy1_5_rtos_v0_3_1.uf2) and use the
[v0.2.13 dashboard](../v0_2_13_combined/). Confirm the car reports firmware
**v0.3.1** after flashing.

## What to test

1. Use **Baseline · 45 mm/s** and the same obstacle placement as in v0.3.0.
   Allow at least 200 mm of straight travel before the sonar reaches the
   330 mm confirmation range.
2. Watch the angled width scan and whether the robot completes the bypass
   and rejoins the line. An eligible rejected scan gets one stationary
   retry. A replacement that fails the original checks causes HOLD.
3. After the run, leave the robot powered on and use **Download run ZIP**
   before Reset. If the robot holds, press **Stop** and wait for the
   coordinator to show Off before downloading. The D4 trace records the
   first triplet when the retry runs.

Both logged v0.3.0 triplets qualified for a retry in host tests. The
simulation continued after a clean retry and held after a second rejected
triplet. The ARM build, UF2 validation, memory and stack checks, and the
project's mechanical C standard and formatting checks passed. Hardware
verification is pending.

## Build and check

From the repository root on the project Windows build host, build in Git Bash.

```sh
bash tools/rtos_migration/build_release.sh APP=rtos_buddy1_5
```

Run the focused host suite with the configured host compiler.

```sh
python tools/rtos_migration/run_tests.py --only test_b5_mission test_b5_straight_gate test_connected_bridge test_release_exports --skip-python
```

- [RTOS retry policy](../../../../../app_program/obstacle_navigation/width_retry_policy.h)
- [Width probe](../../extensions/buddy5_obstacles/width_probe.c)
- [UF2 SHA-256](SHA256SUMS.txt)
