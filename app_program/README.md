# RoboPico application components

The [v0.4.3 release](../firmware/releases/v0.4.3/) builds with `APP=rtos_buddy1_5`. These adapters connect the [shared firmware logic](../firmware/integration/buddy1_4/) to micro T-Kernel tasks, board services and the dashboard.

| Component | Responsibility |
| --- | --- |
| [Network bridge](network_bridge/) | Copied MQTT commands, state, telemetry and stopped downloads |
| [RTOS bring-up](rtos_bringup/) | Startup, motor and encoder ownership, safety gating |
| [Line navigation](line_navigation/) | Front ADC, calibration, tracking and stationary recovery |
| [Barcode navigation](barcode_navigation/) | Decoding, mapped actions and junction qualification |
| [IMU navigation](imu_navigation/) | FIFO acquisition, hump estimation and pickup stop |
| [Obstacle navigation](obstacle_navigation/) | Sonar coverage, width scan, bypass and tape rejoin |

The numbered `subsystem1/` through `subsystem5/` directories and the root `app_main.c` are the course example profile. The [active build and ownership map](../docs/architecture.md) describes the release.
