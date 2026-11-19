# RoboPico RTOS integration

This directory contains the shared feature modules for Buddy 1–5
**v0.2.13** on `barr-c-combined`. The combined build uses
`APP=rtos_buddy1_5`. The existing directory name is used by the build and its
shared API includes.

- [Download the combined UF2 and dashboard](artifacts/v0_2_13_combined/)
- [Setup and powerbank operation](../../../docs/setup.md)
- [Combined test procedure](../../../docs/buddy1_5_v0_2_13.md) and [validation status](STATUS.md)
- [Build instructions](../../../docs/build.md) and [offline tests](../../../docs/testing.md)
- [Architecture](../../../docs/architecture.md) and [public interfaces](include/interfaces/)
- [Buddy 5 mission](../../../app_program/obstacle_navigation/README.md)
- [Release archive](artifacts/README.md) and [SDK reference](SDK_REFERENCE.md)

Active task adapters are under `app_program/`. The shared motion executor
and coordinator are under `src/`. Buddy 5's reviewed width, direction and
bypass algorithms are under
[`extensions/buddy5_obstacles/`](extensions/buddy5_obstacles/). The
application uses micro T-Kernel scheduling and the kernel Wi-Fi service. Pico
SDK components provide the low-level USB, radio and lwIP libraries.

**Baseline · 45 mm/s** is the default. The dashboard configures a barcode map
and can change cruise mode while stopped. Saved Wi-Fi and front IR calibration
remain on the Pico during a normal firmware update.

After Stop, **Download run ZIP** collects the dashboard capture with motion,
controller diagnostics, raw GP27 readings and navigation records. Buddy 5
status and probe telemetry are captured during the dashboard session. Export
before a new Start, Reset or power cycle.
