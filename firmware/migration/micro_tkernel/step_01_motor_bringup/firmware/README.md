# Step1 firmware download — experimental, hardware acceptance pending

- `robopico_rtos_motor_bringup_v0_1.uf2`: flash this bench image through BOOTSEL.
- `robopico_rtos_motor_bringup_v0_1_bundle.zip`: complete project sources and
  test helper; unzip and open `START_HERE.md`. A convenient second copy of the
  same UF2 is at the bundle's `firmware/` root.
- `BUILD_MANIFEST.json`: source revision, compiler/SDK, profile and verification.
- `SOURCE_SHA256.json`: checksums of the798 packaged source/documentation files.
- `SHA256SUMS.txt`: checksums of the public UF2 and ZIP.

This is a micro T-Kernel USB motor/encoder/stop checkpoint, not the circuit
application. Motors remain off until clock validation and an explicit bounded
bench request. Run the read-only timing capture before any powered wheel test.
Wi-Fi/dashboard, line/barcode, IMU and Buddy5 are not active in this image.

Follow the [checkpoint instructions](../README.md). Hardware behaviour still
requires the user's testing. Saved calibration/settings sectors are outside
this UF2; this checkpoint performs no flash writes.
