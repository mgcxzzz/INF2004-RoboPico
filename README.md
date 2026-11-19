# RoboPico Buddy 1–5 RTOS integration

`barr-c-combined` contains the **v0.2.13 integration candidate**. It pairs the
tested Buddy 1–4 v0.2.12 baseline with the Buddy 5 obstacle mission on
micro T-Kernel SMP. The combined build passed software validation. A complete
obstacle run on the robot is still pending.

- [Firmware and dashboard downloads](firmware/integration/buddy1_4/artifacts/v0_2_13_combined/)
- [Setup, powerbank operation and logs](docs/setup.md)
- [First hardware tests and expected results](docs/buddy1_5_v0_2_13.md)
- [Dashboard overview](dashboard/README.md)
- [Build and offline checks](docs/build.md)

## What the combined firmware does

| Buddy | Responsibility |
| --- | --- |
| 1 | Wi-Fi/MQTT, dashboard commands, telemetry and wireless log downloads |
| 2 | Wheel encoders, speed control, motor output and motion safety |
| 3 | Line following and recovery, barcode decoding, mapped junction turns |
| 4 | IMU, hump height measurement, ramp assistance and pickup stop |
| 5 | Sonar detection, width probing, direction selection, obstacle bypass and line rejoin |

Buddy 5 takes a coordinator lease when sonar confirms an obstacle on a
qualified straight. Its finite moves use the shared motion executor. Buddy 3
resumes line following after the coordinator accepts the hand-back. The
dashboard shows obstacle progress in **Obstacle & avoidance** and detailed
evidence in **Diagnostics → Buddy 5 obstacle mission**.

For a first run, use **Baseline · 45 mm/s**. Start from the dashboard after
the car is online and ready, then use **Download run ZIP** after Stop to save
the dashboard capture and stored rows.

## Design and evidence

- [System architecture](docs/architecture.md) and [shared APIs](docs/interfaces.md)
- [Buddy 5 mission stages and telemetry](app_program/obstacle_navigation/README.md)
- [Validation status](firmware/integration/buddy1_4/STATUS.md)
- [RoboPico C Standard](CODING_STANDARD.md) and [documented exceptions](docs/coding_exceptions.md)
- [Release archive and rollback image](firmware/integration/buddy1_4/artifacts/README.md)
- [Historical SDK reference](firmware/integration/buddy1_4/SDK_REFERENCE.md)
