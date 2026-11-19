# RoboPico — provisional Buddy 1–4 RTOS integration

**This branch is `barr-c-cleanup`: v0.2.11 caster-assist diagnostic candidate; hardware
acceptance pending.** [New firmware and complete bundle](firmware/integration/buddy1_4/artifacts/v0_2_11_caster_assist/)
 · [Setup and test instructions](docs/caster_assist_v0_2_11.md)
 · [Application coding standard](CODING_STANDARD.md).
The earlier [v0.2.7 cleanup candidate](docs/coding_cleanup_v0_2_7.md) remains archived.
The tested main baseline described below remains available for rollback.

**Current main release: v0.2.6, micro T-Kernel SMP. Buddy 5 is NOT integrated.**

[Download firmware and full bundle](firmware/integration/buddy1_4/artifacts/v0_2_6_rtos/)
 · [Setup / powerbank / log downloads](docs/setup.md)
 · [Validation and pending circuit checks](firmware/integration/buddy1_4/STATUS.md)
 · [Architecture and interfaces](docs/architecture.md)

This builds on the hardware-tested Step 5 v0.1.5 baseline and main v0.2.1.
It retains Continuous V2 Profile 1 at 45 mm/s, both barcode decoders, live A–Z mappings,
junction actions, motion/encoders and Buddy 4 IMU/ramp assistance. The new production
profile supports powerbank operation with Wi-Fi/MQTT supervision and stopped
wireless history downloads. Motors stay OFF on boot; Start is explicit.

The RTOS baseline passed flat/Pause/Resume/Stop and one nominal 10 mm one-sided ramp:
0 flat humps; 1 accepted ramp/13 mm maximum; assistance active; 50 Hz IMU/100 Hz control;
no new motion-window acquisition/timing/logging faults. This is one ramp success,
not repeatability certification. v0.2.0 powerbank driving and wireless downloads
were subsequently user-tested. v0.2.1 diagnostics were used to identify weak
barcode peaks and a borderline junction width. v0.2.2 changes junction qualification,
bend response and line recovery; physical acceptance of these changes is pending.
Sharp bends and complete left/right line reacquisition need sufficient mats.

Run [the complete package](firmware/integration/buddy1_4/README.md). Python and an
installed Mosquitto broker are needed on the laptop; the ARM compiler is needed
only for rebuilding. Saved front calibration/Wi-Fi are preserved. Buddy 1 owns
network messaging; Buddy 3/Buddy 4 requests use the shared nav coordinator and one
motor writer. All application networking goes through copied bounded buffers to
the kernel Wi-Fi service. New fields use compact representations and protected
snapshots; see [data representation](docs/data_representation.md).

The [SDK v0.1 reference](firmware/integration/buddy1_4/SDK_REFERENCE.md) and its
artifacts remain available for comparison. Migration checkpoints under
firmware/migration/ are historical validation stages. Main's active build is
`APP=rtos_buddy1_4`, not the SDK superloop or the template example.


[v0.2.2 setup, focused driving tests and downloads](docs/drive_refinements_v0_2_2.md):
7 mm junction minimum, selective bend assistance and bounded in-place line search.
The working barcode decoders and calibration priority are unchanged. This release
retains fault diagnostics and does not claim to cure delivery outages or all weak
barcode signals. Buddy 5 remains outside this integration.

[v0.2.3 IMU end-of-run setup and checks](docs/imu_end_run_v0_2_3.md): lifted/overturned
detection now latches all motors off through the safety/coordinator path; Reset
requires eligible IMU evidence, and Start remains explicit. Production collision
classification is removed. The user reports improved v0.2.2 junction detection;
bends remain untested and recovery awaits diagnostic logs.

[v0.2.4 status correction](docs/imu_status_clock_v0_2_4.md) uses a single hardware
clock inside C9 reporting, preventing false IMU-not-ready labels from kernel/
hardware uptime offsets. Actual safety gates and controllers are unchanged.

[v0.2.5 Demo console and drive modes](docs/demo_console_v0_2_5.md) adds the compact
rubric overview, stopped-only 45/60/75 cruise selection, bounded per-wheel load
reserve, 45-degree/100 ms pickup qualification and retained IMU peak diagnostics.
The default remains Baseline 45. Faster modes and the revised pickup gesture need
physical acceptance; Buddy 5 remains pending.

[v0.2.6 ramp timing correction](docs/ramp_timing_v0_2_6.md) fixes false stale
IMU-snapshot ages in the ramp adapter. Existing RTOS ownership, wire formats,
speed modes and all detection/acceptance thresholds remain. Hardware acceptance
of this timing correction is pending.
