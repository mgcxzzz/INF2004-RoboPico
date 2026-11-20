# RoboPico Buddy 1–5 RTOS integration

`halt-testing` contains the **v0.3.0 stopping-distance candidate** for Buddy 5.
It confirms an obstacle from 330 mm on a qualified straight and requests a
stop at 295 mm. The 300 mm close-obstacle hold still applies on bends.
The staged stop passed host simulation. The stopped range and first angled
width scan still need a run on the robot.

- [v0.3.0 UF2, test steps, and source pointers](firmware/integration/buddy1_4/artifacts/v0_3_0_halt_testing/)
- [Compatible v0.2.13 dashboard bundle](firmware/integration/buddy1_4/artifacts/v0_2_13_combined/)
- [Dashboard setup, powerbank operation and logs](docs/setup.md)
- [Dashboard overview](dashboard/README.md)
- [Build profile and focused checks](firmware/integration/buddy1_4/artifacts/v0_3_0_halt_testing/README.md#build-and-check)

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
