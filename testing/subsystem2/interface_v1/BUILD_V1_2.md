# Interface V1.2 test build — 2026-10-27

V1.2 changes ordinary forward effort and cruising qualification to address the user's straight-line stuttering reports. It retains V1.1's confirmed startup fix, V1 API, ownership rules and external finite-move policies. Read SMOOTH_DRIVE_V1_2.md for evidence, scope and physical testing instructions. It is not an exact V12.2 motor-output baseline. BUILD_V1.md / BUILD_V1_1.md and matching metrics archive the earlier releases.

## Executed verification

- **23 host executables passed** via run_tests.py, Zig C11 `-Wall -Wextra -Werror`. These include twelve retained V12.2 tests; coordinator tests packed/unpacked; clock-order regression packed/unpacked; independent flash-log tests; the Buddy5 workflow; baseline routing parity; and four new drive/cruise/replay/pipeline tests.
- Clock regressions retain14,000 realistic request/snapshot-order ticks plus stale heartbeat, settling, takeover and resume boundaries in each representation.
- The baseline9,000-tick routing test still verifies the unchanged provider/coordinator forwarding path for identical inputs. It deliberately does not claim the new V1.2 cruise/PWM adapter is unchanged; those outputs are intentionally different and separately tested.
- Ordinary-drive tests cover gradual positive output, caps300/420, blocked and changing-load conditions, immediate zero/inner-wheel coast, and exact disabled-path parity. Twelve simplified stiction models reach the requested speed; some have higher cumulative PWM variation with adaptive trim, so they do not establish improvement in every metric or actual robot behavior.
- Captured20ms motion inputs: positive PWM jumps>80 counts fall22->0 and near-steady-target jumps18->0 in a counterfactual replay. Recorded velocity is held as input; this is not an exact10ms replay or a prediction of new physical speed.
- The new cruise qualifier is tested independently of ADC levels, with unequal brightness, actual turning and steering hunting. The composed optical pipeline qualifies real normalized850/500 steady tracking, preserves an urgent bend response, and decodes and dispatches A/F/Z frames in both directions at a subsequent cross.
- Default, unpacked-reference and opt-in Buddy5 example ARM Pico W Release builds passed `-Wall -Wextra -Werror`. External API maneuvers and dedicated ramp/search/turn controllers bypass the new ordinary-drive wrapper.
- verify_firmware.py checks frozen snapshot hashes/four explicit hardware/record patches, RP2040 UF2 blocks, embedded version/diagnostic markers, current build equality, flash bounds and preserved V12.2/V1/V1.1 artifacts.

Toolchain: Pico SDK2.2.0, ARM GCC14.2.1, local CMake/Ninja, USB serial. Optional missing Wi-Fi/Bluetooth SDK submodules are not used by this standalone build.

## Measured ARM sizes

| Build | text | data | BSS |
|---|---:|---:|---:|
| V12.2 original |127128|0|215408|
| V1.1 packed |150832|0|222064|
| V1.2 same-feature unpacked reference |154176|0|227072|
| V1.2 published packed |155432|0|222112|

Like-for-like shared-field packing still saves4960B BSS; V1.2 adds48B BSS over V1.1 for two12-byte DriveSmooth states and one24-byte TrackingCruise state. Runtime float calculations remain; this is partial Tribunal compliance, not whole-firmware completion. No execution-time or physical smoothness measurement on the Pico is claimed.

`__flash_binary_end=0x10024f2c`; padded UF2 end `0x10025000`. `__bss_end__=0x20037830`, `__StackLimit=0x20040000`:34768B between those symbols, not a measured worst-case free-stack/heap figure. Logs remain at coordinator0x1af000/raw0x1b0000/motion0x1d0000. The image is below all log banks.

## Artifact

- `firmware/buddy23_interface_v1_2.uf2`
- 303104 bytes; 592 RP2040 UF2 blocks with contiguous256-byte payloads.
- SHA256 `e8afd34c97ceb0963969e571330989693528d23939f3462a97c68c81e52f7bdf`.
- V1.1 comparison file remains `42a81aeca01fffba960a5e508f3b1c6f3e00d5a03cf53fff0a2c5d445b73f775`.
- V12.2 original remains `76941390c3779295ff750596a76d84e575a92c2b26bc78a3203771efa052641c`.

Motion record20 uses the same64848-byte storage layout, but earlier binaries cannot import20. Note CAL references before downgrading and recalibrate as needed. Existing calibration imports on upgrade. The CMake target stays buddy23_interface_v1 for integration compatibility; the new download has the distinct v1_2 filename.

## Required next evidence

Test profile1 on the same mat/straight first, then a bend and barcode. Compare actual jerk, wheel tracking and logs. Only then try profile3. Physical V1.2 following/scanning, different surfaces, power/load variation, and ultrasonic-integrated control timing remain unverified. This firmware changes the suspected effort oscillation and qualification policy; it does not promise perfectly straight motion or scans on every mat.
