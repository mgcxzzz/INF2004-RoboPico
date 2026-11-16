# Buddy 1–4 RTOS integration v0.2.6

**PROVISIONAL. Buddy 5 obstacle avoidance is NOT integrated.**

This is main's active micro T-Kernel SMP release, promoted from tested Step 5
v0.1.5 (75325f2). The shared feature modules in src/ are retained; active task
adapters are under app_program/. The Pico SDK supplies low-level libraries,
while micro T-Kernel owns task scheduling and the kernel Wi-Fi service.

- [Setup, powerbank and downloads](../../../docs/setup.md)
- [Release status / hardware evidence](STATUS.md)
- [Build](../../../docs/build.md) and [focused checks](../../../docs/testing.md)
- [Architecture](../../../docs/architecture.md) and include/interfaces/
- [Current UF2 + full bundle](artifacts/v0_2_6_rtos/)
- [Historical SDK reference](SDK_REFERENCE.md)

Profile 1 keeps 45 mm/s nominal straight travel. v0.2.2 adds qualified bend
slowing/authority, bounded in-place search and a 7 mm junction minimum; junction turns have improved in user testing. Bend acceptance and recovery
diagnostics remain pending. Runtime A–Z routing, barcode acceptance,
110 mm GP27-to-axle geometry, saved calibration and ramp assistance are retained. Short odometry gaps use the tested bounded qualification policy.
Powerbank operation requires the Pico's fresh broker connection; closing a
USB terminal no longer stops this release. GP20/GP21 and dashboard stops remain.

Stopped stored datasets: motion48-byte frames, controller56-byte rows,
raw GP27 ten-byte rows and nav-coordinator31-byte records. The dashboard
produces CSV/binary/metadata and the package includes passive wireless capture.
These histories are RAM-only and bounded; export before another Start/reboot.

The existing CMake SDK target remains a legacy comparison build. Use the
RTOS make profile in docs/build.md for this release. Public images contain
no Wi-Fi password. The manifest pins the tested baseline and build inputs.

The [v0.2.3 IMU safety addition](../../../docs/imu_end_run_v0_2_3.md) replaces
collision classification with a latched lift/overturn stop. It does not tune the
line/barcode/ramp controllers. Use the complete new dashboard bundle.

v0.2.4 corrects C9 diagnostic freshness to use the same hardware-timer epoch as
its samples. It retains v0.2.3 detection/stop/reset behavior; see
[the status correction guide](../../../docs/imu_status_clock_v0_2_4.md).

[v0.2.5 Demo console and drive modes](../../../docs/demo_console_v0_2_5.md) adds
stopped-only Baseline45/Normal60/Faster-trial75 cruise settings and bounded
per-wheel load reserve. It also fixes protected IMU freshness sampling, changes
tilt qualification to45 degrees/100 ms and exposes retained peak diagnostics.
Use the complete new dashboard bundle. Calibration, barcode decoders, junction
geometry/7mm gate, bend/recovery logic and RTOS ownership remain. Physical
acceptance of the new speed/gesture settings is pending.

[v0.2.6 ramp timing correction](../../../docs/ramp_timing_v0_2_6.md) reads the
hardware clock after copying the IMU snapshot. It retains the real stale
deadlines and all controller/measurement thresholds. No state or packet layout
is expanded. The v0.2.5 dashboard remains compatible.
