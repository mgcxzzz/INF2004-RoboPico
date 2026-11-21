# RoboPico Buddy 1–5 RTOS

The team selected **v0.4.3** for Tribunal 2. It runs on the Pico W under
micro T-Kernel SMP with Demo Console v0.2.15. The release applies a warning
and continues for a corroborated borderline obstacle range after a turn.

- [v0.4.3 firmware and full dashboard bundle](firmware/integration/buddy1_4/artifacts/v0_4_3/)
- [Tribunal 2 requirements, cases and results](testing/)
- [Build instructions](docs/build.md)
- [Dashboard setup, powerbank operation and logs](docs/setup.md)
- [Dashboard overview](dashboard/README.md)

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
- [Release verification and hardware evidence](testing/)
- [RoboPico C Standard](CODING_STANDARD.md) and [documented exceptions](docs/coding_exceptions.md)
- [Release archive and rollback image](firmware/integration/buddy1_4/artifacts/README.md)
- [Historical SDK reference](firmware/integration/buddy1_4/SDK_REFERENCE.md)
