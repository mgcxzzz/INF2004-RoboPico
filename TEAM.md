# Team and subsystems

The current release is [RoboPico Buddy 1–5 v0.4.3](firmware/releases/v0.4.3/), built with `APP=rtos_buddy1_5` on micro T-Kernel SMP.

## 1. Members and subsystems

| Buddy | Subsystem | Student ID | GitHub | Code | Tests |
| --- | --- | --- | --- | --- | --- |
| 1 | Wi-Fi, MQTT and dashboard | 2500399, 2500339 | [mgcxzzz](https://github.com/mgcxzzz), [N1sh0](https://github.com/N1sh0) | [Network bridge](app_program/network_bridge/) · [Dashboard](dashboard/) | [Subsystem 1](testing/subsystem1/TEST_LOG.md) |
| 2 | Motor control, encoders and odometry | 2500399 | [mgcxzzz](https://github.com/mgcxzzz) | [Motor and safety tasks](app_program/rtos_bringup/) · [Shared motion](firmware/integration/buddy1_4/src/motion/) | [Subsystem 2](testing/subsystem2/TEST_LOG.md) |
| 3 | Line following, barcode decoding and junction routing | 2500399 | [mgcxzzz](https://github.com/mgcxzzz) | [Line navigation](app_program/line_navigation/) · [Barcode navigation](app_program/barcode_navigation/) | [Subsystem 3](testing/subsystem3/TEST_LOG.md) |
| 4 | IMU, hump measurement, ramp help and pickup stop | 2500399 | [mgcxzzz](https://github.com/mgcxzzz) | [IMU navigation](app_program/imu_navigation/) | [Subsystem 4](testing/subsystem4/TEST_LOG.md) |
| 5 | Obstacle detection, width estimate, bypass and line rejoin | 2500339 | [N1sh0](https://github.com/N1sh0) | [Obstacle navigation](app_program/obstacle_navigation/) | [Subsystem 5](testing/subsystem5/TEST_LOG.md) |

## 2. Pins

| Pins | Owner | Use |
| --- | --- | --- |
| GP0/1 and GP4/5 | Buddy 2 | Left and right encoder inputs |
| GP8–11 | Buddy 2 | Motor outputs |
| GP20/21 | Safety and motion | Physical stop/start controls |
| GP26/27/28 | Buddy 2/3 motion and ADC task | Left line IR, barcode IR and right line IR |
| GP2/3 | Buddy 4 | I2C1 for the IMU |
| GP12, GP16/17 | Buddy 5 | Servo PWM, sonar trigger and echo |
| GP23–25 and GP29 | Kernel | Pico W wireless hardware |

## 3. Peripherals

Core 1 owns motor PWM, encoder draining and front ADC sampling. Buddy 4 owns I2C1 on core 0. Buddy 5 owns the sonar and servo, and the core 1 GPIO IRQ timestamps echo edges. The kernel Wi-Fi task on core 0 owns CYW43, lwIP and MQTT.

## 4. Task priorities

See the [active RTOS task table](docs/architecture.md) for core assignments and priorities. Smaller priority numbers run first.

## 5. Interfaces between subsystems

The [interface table](docs/interfaces.md) identifies callers, ownership and data. The coordinator arbitrates drive requests. The motion task commits normal PWM output. Safety can cut output. Cross-core exchanges use bounded copied buffers or short protected snapshots.

## 6. Kernel object budget

The [v0.4.3 verification record](testing/integration/logs/2026-11-21_v0.4.3/README.md) records the linked tasks and the static kernel pool estimate for the release build.
