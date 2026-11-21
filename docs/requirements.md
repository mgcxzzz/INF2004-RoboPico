# RoboPico requirements

The [test ledgers](../testing/README.md) use these IDs for traceability. Values
identified as targets are evaluated in the corresponding measured tests.

## Functional requirements

| ID | Requirement |
| --- | --- |
| FR01 | Navigate the marked course onboard after supervisory Start without manual steering or turn commands. |
| FR02 | Validate and acknowledge MQTT Start, Pause, Resume, Stop, Reset, Emergency Stop and configuration. Accepted Start begins a nominal 2 s countdown and requires readiness through the countdown. |
| FR03 | Connect to Wi-Fi and MQTT and publish heartbeat, state, motion, line, barcode, hump and fault data. Stop on supervisory link loss. Reconnect restores reporting, and motion requires a new Start. |
| FR04 | Allow one normal motion owner through the shared coordinator. Use encoder feedback for wheel speed, distance and heading and report completion or failure of bounded manoeuvres. |
| FR05 | Follow the tape with calibrated front IR sensors, adjust speed on bends and search in place when it loses the line. Stop when the bounded search fails. Allow bounded load assistance in Baseline mode when a driven wheel slows sharply in a correction. |
| FR06 | Detect and decode a course barcode. Queue at most one mapped instruction per traversal. Invalid and unmapped reads queue no action. |
| FR07 | Execute Straight, Left, Right or U-turn at the next qualified crossing. Qualify a 7–30 mm stripe with at least 4 mm of white before and after. Reacquire the outgoing line or stop. |
| FR08 | Detect a hump under one drive wheel, estimate peak height and retain the highest accepted peak. Keep the other drive wheel on level mat for measurement. Allow bounded ramp assistance. |
| FR09 | Remove drive on E-stop, a critical fault or a qualified IMU lift or overturn. Keep the pickup stop latched until eligible Reset and require explicit Start to move again. |
| FR10 | Support stopped speed and mapping changes, staged front-sensor calibration with RAM or saved application, and one ZIP of dashboard history and available stored run data. |
| FR11 | Detect an obstacle ahead, stop before contact, and report a width when the measurement qualifies. Report whether an estimate is unavailable or assumed. |
| FR12 | Select a safe bypass, avoid contact, reacquire the original line and return ownership to line following. Stop and report a blocked route when there is no safe bypass. |

## Non-functional requirements

| ID | Requirement or verification target |
| --- | --- |
| NFR01 | Run on Pico W with micro T-Kernel SMP and support wireless operation on a power bank. Nominal motor control is 100 Hz, IMU FIFO service is 50 Hz, the accelerometer FIFO samples at 400 Hz, and the safety supervisor waits 1 ms between checks. |
| NFR02 | Remove drive within 100 ms of accepted E-stop or a qualified critical fault or IMU event. Start requires valid sensor and clock evidence, a usable link, released buttons and cleared latches. |
| NFR03 | Over five minutes, target at least 95% telemetry delivery and a maximum 2 s heartbeat gap. The heartbeat period is nominally 1 s. Supervise the link on the car. |
| NFR04 | Baseline speed target 45 mm/s with stopped selections of 60 and 75 mm/s. The bounded-motion target is ±5% of distance and ±5° of turn over ten documented trials. |
| NFR05 | Target 30 s of unaided tracking under each of three documented lighting conditions and at least 19 correct decodes in 20 traversals at nominal speed. |
| NFR06 | Compare accepted peak readings against a measured one-wheel reference hump across repeated trials. The error target is 10%. Keep rejected or live readings distinct from accepted peaks. |
| NFR07 | Track 20/20 detections across three materials, width error within 10% for 50–200 mm targets at 200–400 mm, at least 60 ms between triggers, and 9/10 contact-free rejoins within 10 s of clearing. The deployed width display uses a bounded estimate or labelled fallback, so width accuracy is evaluated against the actual target. |
| NFR08 | Match field widths to documented ranges and encode bytes, bits, sign and reserved fields explicitly. Protect shared updates and compare RAM and payload sizes with an equivalent baseline. |
| NFR09 | Use testable subsystem interfaces, single-owner drivers, bounded queues or copied snapshots, and I/O outside shared-state locks. Follow the [project C standard](../CODING_STANDARD.md) and [documented exceptions](coding_exceptions.md). |
