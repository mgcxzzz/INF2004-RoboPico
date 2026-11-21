# RoboPico requirements

These requirements apply to the [Buddy 1–5 v0.4.3 release](../firmware/releases/v0.4.3/). The [test ledgers](../testing/README.md) track the evidence for each ID. Verification targets are assessed in their corresponding measured tests.

## Functional requirements

| ID | Requirement |
| --- | --- |
| FR01 | **Autonomous operation.** Navigate the marked course onboard after supervisory Start without manual steering or turn commands. |
| FR02 | **Supervisory control.** Validate and acknowledge MQTT Start, Pause, Resume, Stop, Reset, Emergency Stop and configuration. Accepted Start begins a nominal 2 s countdown and requires readiness through the countdown. |
| FR03 | **Network and telemetry.** Connect to Wi-Fi and MQTT and publish heartbeat, operating state, wheel, line, barcode, hump, obstacle and fault data. Stop on loss of the supervised link. Reconnect restores reporting and motion requires a new Start. |
| FR04 | **Motion and odometry.** Allow one normal motion owner through the shared coordinator. Use encoder feedback for wheel speed, distance and heading and report completion or failure of bounded manoeuvres. |
| FR05 | **Line tracking and recovery.** Follow the tape with calibrated front IR sensors, adjust speed on bends and search in place when it loses the line. Stop when the bounded search fails. Allow bounded load assistance in Baseline mode when a driven wheel slows sharply in a correction. |
| FR06 | **Barcode detection and decoding.** Detect and decode a course barcode and report the letter. Queue at most one mapped instruction per traversal. Invalid and unmapped reads queue no action. |
| FR07 | **Junction instructions.** Execute Straight, Left, Right or U-turn at the next qualified crossing. Qualify a 7–30 mm stripe with at least 4 mm of white before and after. Reacquire the outgoing line or stop. |
| FR08 | **Hump measurement.** Detect a hump under one drive wheel, estimate peak height and retain the highest accepted peak. Keep the other drive wheel on level mat for measurement. Allow bounded ramp assistance. |
| FR09 | **Safety and end of run.** Remove drive on E-stop, a critical fault or a qualified IMU lift or overturn. Keep the pickup stop latched until eligible Reset and require explicit Start to move again. |
| FR10 | **Stopped configuration and logs.** Support stopped speed and mapping changes, staged front-sensor calibration with RAM or saved application, and one ZIP of dashboard history and available stored run data. |
| FR11 | **Obstacle detection and width.** Use ultrasonic ranging to detect an obstacle ahead and stop before contact. Protect active left, right and U-turn actions from obstacle takeover. Report a width estimate in millimetres when evidence qualifies, label an assumed width, and indicate when a measurement is unavailable. |
| FR12 | **Bypass and line rejoin.** Use a coordinator lease for obstacle probing and bypass. Select a bypass with sufficient clearance, avoid contact, reacquire the original line and return motion ownership to line following. Stop and report the reason if a safe bypass or rejoin cannot be completed. |

## Non-functional requirements

| ID | Requirement or verification target |
| --- | --- |
| NFR01 | **RTOS execution.** Run on Pico W with micro T-Kernel SMP and support wireless operation on a power bank. Nominal motor control is 100 Hz, IMU FIFO service is 50 Hz, the accelerometer FIFO samples at 400 Hz, and the safety supervisor waits 1 ms between checks. |
| NFR02 | **Stop response.** Remove drive within 100 ms of accepted E-stop or a qualified critical fault or IMU event. Start requires valid sensor and clock evidence, a usable link, released buttons and cleared latches. |
| NFR03 | **Connectivity targets.** Over five minutes, target at least 95% telemetry delivery and a maximum 2 s heartbeat gap. The heartbeat period is nominally 1 s. Supervise the link on the car. |
| NFR04 | **Motion targets.** Baseline speed target 45 mm/s with stopped selections of 60 and 75 mm/s. The bounded-motion target is ±5% of distance and ±5° of turn over ten documented trials. |
| NFR05 | **Line and barcode targets.** Target 30 s of unaided tracking under each of three documented lighting conditions and at least 19 correct decodes in 20 traversals at nominal speed. |
| NFR06 | **Hump measurement quality.** Compare accepted peak readings against a measured one-wheel reference hump across repeated trials. The error target is 10%. Keep rejected or live readings distinct from accepted peaks. |
| NFR07 | **Obstacle verification targets.** Keep ultrasonic triggers at least 60 ms apart. Target 20/20 detections across three materials, width error within 10% for 50–200 mm targets at 200–400 mm, and 9/10 contact-free rejoins within 10 s of clearing. Verify width against a measured target and record whether the displayed value is an estimate or an assumption. |
| NFR08 | **Compact representation.** Match field widths to documented ranges and encode bytes, bits, sign and reserved fields explicitly. Protect shared updates and compare RAM and payload sizes with an equivalent baseline. |
| NFR09 | **Isolation and code standard.** Use testable subsystem interfaces, single-owner drivers, bounded queues or copied snapshots, and I/O outside shared-state locks. Follow the [project C standard](../CODING_STANDARD.md) and [documented exceptions](coding_exceptions.md). |
