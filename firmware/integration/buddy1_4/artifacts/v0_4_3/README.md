# RoboPico v0.4.3

- [Firmware UF2](robopico_buddy1_5_rtos_v0_4_3.uf2)
- [Full firmware and Demo Console v0.2.15 bundle](robopico_buddy1_5_rtos_v0_4_3_test_bundle.zip)
- [Build manifest and source commit](BUILD_MANIFEST.json)
- [SHA-256 checksums](SHA256SUMS.txt)

Flash the UF2 in BOOTSEL mode. Use the bundled dashboard for the new range
warning. The bundle includes the Windows launchers and macOS instructions.
Confirm the dashboard reports v0.4.3 and the car is ready before starting.

The release accepts a corroborated slightly short echo after straight settling.
The robot completes its stationary scan and continues the bypass when the
range, tape and clearance checks pass. The event timeline displays
**accepted with range warning**.

[Rebuild instructions](../../../../../docs/build.md) ·
[Tests and evidence](../../../../../testing/README.md)
