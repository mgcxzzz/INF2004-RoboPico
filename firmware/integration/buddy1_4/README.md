# Buddy 1–4 RTOS integration v0.2.0

**PROVISIONAL. Buddy 5 obstacle avoidance is NOT integrated.**

This is main's active micro T-Kernel SMP release, promoted from tested Step 5
v0.1.5 (75325f2). The shared feature modules in src/ are retained; active task
adapters are under app_program/. The Pico SDK supplies low-level libraries,
while micro T-Kernel owns task scheduling and the kernel Wi-Fi service.

- [Setup, powerbank and downloads](../../../docs/setup.md)
- [Release status / hardware evidence](STATUS.md)
- [Build](../../../docs/build.md) and [focused checks](../../../docs/testing.md)
- [Architecture](../../../docs/architecture.md) and include/interfaces/
- [Current UF2 + full bundle](artifacts/v0_2_0_rtos/)
- [Historical SDK reference](SDK_REFERENCE.md)

Profile 1 stays fixed at 45 mm/s. Runtime A–Z routing, barcode/line/junction
logic,110 mm GP27-to-axle offset, current calibration and ramp assistance are
retained. Short odometry gaps use the tested bounded qualification policy.
Powerbank operation requires the Pico's fresh broker connection; closing a
USB terminal no longer stops this release. GP20/GP21 and dashboard stops remain.

Stopped stored datasets: motion48-byte frames, controller56-byte rows,
raw GP27 ten-byte rows and nav-coordinator31-byte records. The dashboard
produces CSV/binary/metadata and the package includes passive wireless capture.
These histories are RAM-only and bounded; export before another Start/reboot.

The existing CMake SDK target remains a legacy comparison build. Use the
RTOS make profile in docs/build.md for this release. Public images contain
no Wi-Fi password. The manifest pins the tested baseline and build inputs.
