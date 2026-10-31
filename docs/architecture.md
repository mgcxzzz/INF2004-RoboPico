# Provisional Buddy1–4 architecture

Public requests -> nav coordinator (ownership, bounds, expiry, audit) -> motion implementation -> one generation-checked PWM commit. Buddy3 owns ordinary AUTO; Buddy4 requests a temporary modifier; Buddy5 is not integrated.

Read the detailed feature logic: [navigation](../firmware/integration/buddy1_4/src/navigation/README.md), [motion](../firmware/integration/buddy1_4/src/motion/README.md), [line following](../firmware/integration/buddy1_4/src/line_following/README.md), [barcode](../firmware/integration/buddy1_4/src/barcode/README.md), [IMU](../firmware/integration/buddy1_4/src/imu/README.md), [network](../firmware/integration/buddy1_4/src/network/README.md) and [telemetry](../firmware/integration/buddy1_4/src/telemetry/README.md).

Core0 schedules sensing/control; core1 owns SDK Wi-Fi/lwIP/MQTT through copied bounded queues. No RTOS migration is performed. Application-private adapters retain the established state-machine scheduling; they are not public subsystem APIs. Pure tracker/controller headers remain testable numeric modules. Compiled coordinator/executor/IMU/network files are separate translation units.
