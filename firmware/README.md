# RoboPico firmware

## Current release

**v0.4.3 · Buddy 1–5 · Pico W · micro T-Kernel SMP**

- [Flash-ready UF2](releases/v0.4.3/robopico-v0.4.3-buddy1-5.uf2)
- [Full ZIP with firmware and dashboard](releases/v0.4.3/robopico-v0.4.3-buddy1-5-bundle.zip)
- [Release notes, build manifest and checksums](releases/v0.4.3/)
- [Setup](../docs/setup.md) and [rebuild instructions](../docs/build.md)

## Source and release layout

| Location or build target | Purpose |
| --- | --- |
| `firmware/releases/vX.Y.Z/` | Published firmware, dashboard bundle and provenance. Current download filenames use the release number and Buddy range. |
| [`firmware/integration/buddy1_4/`](integration/buddy1_4/) | Shared controller, navigation, sensors, telemetry and interfaces. The source directory keeps its original name from the Buddy 1–4 integration. Both RTOS profiles compile this code. |
| [`app_program/`](../app_program/) | RTOS application tasks, including [network](../app_program/network_bridge/), [motion and safety](../app_program/rtos_bringup/), [line](../app_program/line_navigation/), [barcode](../app_program/barcode_navigation/), [IMU](../app_program/imu_navigation/) and [obstacle navigation](../app_program/obstacle_navigation/). Numbered subsystem folders are standalone applications. |
| `APP=rtos_buddy1_5` | Current combined build profile. |
| `APP=rtos_buddy1_4` | Four-Buddy build profile retained for comparison and rollback. |
| [Archived packages](integration/buddy1_4/artifacts/) and [migration checkpoints](migration/micro_tkernel/) | Versioned development, RTOS bring-up and test history. Paths and file names identify the original builds. |
