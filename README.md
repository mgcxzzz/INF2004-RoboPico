# RoboPico

**Team IS28 · Pico W · micro T-Kernel SMP**

RoboPico is an autonomous course robot that follows tape, reads barcodes to
choose junction turns, estimates hump height with an IMU, and uses sonar to
bypass obstacles and rejoin the line. Its web dashboard displays live motion
and sensor data over MQTT and collects run logs.

**Firmware v0.4.3 · Demo Console v0.2.15**

[Download the full bundle (ZIP)](firmware/releases/v0.4.3/robopico-v0.4.3-buddy1-5-bundle.zip) ·
[Download the firmware (UF2)](firmware/releases/v0.4.3/robopico-v0.4.3-buddy1-5.uf2) ·
[Release notes and checksums](firmware/releases/v0.4.3/)

## Explore the project

| What you’re looking for | Where to find it |
| --- | --- |
| Demo and results | [Demo videos](videos/) · [Circuit test](testing/system/logs/2026-11-21_run_4/README.md) · [Test cases, results and logs](testing/) |
| Setup and build | [Setup and operation](docs/setup.md) · [Dashboard guide](dashboard/README.md) · [Build guide](docs/build.md) |
| Design and interfaces | [Project requirements](docs/requirements.md) · [System architecture](docs/architecture.md) · [Shared APIs](docs/interfaces.md) |
| Source and verification | [Firmware layout](firmware/) · [v0.4.3 verification](testing/integration/logs/2026-11-21_v0.4.3/README.md) · [C coding standard](CODING_STANDARD.md) · [Documented exceptions](docs/coding_exceptions.md) |
| Team | [Team roster, components and ownership](TEAM.md) |

## The five subsystems

| Buddy | Responsibility |
| --- | --- |
| 1 · [Networking](app_program/network_bridge/) | Wi-Fi and MQTT, dashboard commands, live telemetry and wireless log downloads |
| 2 · [Motion and safety](app_program/rtos_bringup/) | Wheel encoders, speed control, motor output and motion safety |
| 3 · [Line and barcode navigation](app_program/line_navigation/) | Tape following and recovery, [barcode decoding](app_program/barcode_navigation/) and mapped junction turns |
| 4 · [IMU and ramp assistance](app_program/imu_navigation/) | Hump height estimation, ramp assistance and pickup stop |
| 5 · [Obstacle navigation](app_program/obstacle_navigation/README.md) | Sonar detection, width estimation, bypass side selection and line rejoin |

The shared motion coordinator arbitrates requests. One task writes normal
motor output, and the safety path can cut drive independently.

[Archived releases and rollback image](firmware/integration/buddy1_4/artifacts/README.md) ·
[Historical SDK reference](firmware/integration/buddy1_4/SDK_REFERENCE.md)
