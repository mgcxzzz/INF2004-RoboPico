# RoboPico — provisional Buddy 1–4 RTOS integration

**Current main release: v0.2.1, micro T-Kernel SMP. Buddy 5 is NOT integrated.**

[Download firmware and full bundle](firmware/integration/buddy1_4/artifacts/v0_2_1_rtos/)
 · [Setup / powerbank / log downloads](docs/setup.md)
 · [Validation and pending circuit checks](firmware/integration/buddy1_4/STATUS.md)
 · [Architecture and interfaces](docs/architecture.md)

This promotes the hardware-tested Step 5 v0.1.5 baseline on the user's instruction.
It retains Continuous V2 Profile 1 at 45 mm/s, both barcode decoders, live A–Z mappings,
junction actions, motion/encoders and Buddy 4 IMU/ramp assistance. The new production
profile supports powerbank operation with Wi-Fi/MQTT supervision and stopped
wireless history downloads. Motors stay OFF on boot; Start is explicit.

The RTOS baseline passed flat/Pause/Resume/Stop and one nominal 10 mm one-sided ramp:
0 flat humps; 1 accepted ramp/13 mm maximum; assistance active; 50 Hz IMU/100 Hz control;
no new motion-window acquisition/timing/logging faults. This is one ramp success,
not repeatability certification. v0.2.0 powerbank driving and wireless downloads
were subsequently user-tested. v0.2.1 adds diagnostic snapshots, whose first
physical comparison remains pending. The full circuit test is still outstanding.
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


[v0.2.1 diagnostic test and required downloads](docs/diagnostics_v0_2_1.md):
latest barcode attempt raw samples, latched encoder trip and latched network-stop
reason/ages. This build preserves the guards and does not claim to fix missing
barcode peaks, signal errors or delivery outages before their cause is verified.
