# Shared RoboPico integration source

Both Buddy 1–4 and Buddy 1–5 builds use this source tree. The current combined release is [v0.4.3](../../releases/v0.4.3/).

The combined build selects `APP=rtos_buddy1_5`. Build files and shared API includes reference the existing source path.

- [Current v0.4.3 UF2 and dashboard](../../releases/v0.4.3/)
- [Setup and powerbank operation](../../../docs/setup.md)
- [Current test ledgers](../../../testing/README.md) and [v0.2.13 validation history](STATUS.md)
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
