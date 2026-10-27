# Interface Continuous v2 build evidence — 2026-10-28

Base: Continuous v1 at`577652daa73afd5f293d59fef7136188a6274974`. Separate folder/artifact; no Buddy5 or IMU policy is added. Default extension source remains empty.

## Executed checks

- **26 host executables passed** with Zig C11`-Wall -Wextra -Werror`. Existing API/clock/stop/ownership, codecs/CRC, generic A/F/Z reader and dispatch, junction/search and continuous motor-model tests are retained.
- New steering checks cover70% entry gain,350ms same-sign turn recovery, reversal and straight-dead-zone reset, mirrored noise, normalized fall-rate confidence, encoder damping on return versus zero yaw, restored full authority for a constant bend request, bounded outputs, profile/scan transitions, finite inputs and immediate bypass/reset.
- Gap-handover regression uses the recorded-style right-wheel252PWM/94physical-I state and positive35mm/s request. The continuous controller retains running state and respects its output rate bound instead of switching to the legacy driver. Explicit both-zero/coast stays immediately off. Search19..23, unknown decisions, external owners, planned-turn phases and profile2 remain excluded. Tests hold inputs/models fixed; they are not predictions of the new trajectory.
- The wrapper regression compares unmodified optical requests and navigation state across all profiles, exact profile2 legacy adapter behaviour, scan begin/end, loss/stop/turn bypass and A/F/Z dispatch. Generic coordinator host mocks still exercise stop/takeover/resume and stale-commit rejection; default Buddy5 hooks are empty.
- Packed diagnostics now exercise **464bits/58bytes**, added3-bit GAIN and2-bit DAMP extrema/clipping, format4 CRC/save/restore, old-format rejection and the scheduled3s window. Motion record25 validates1..25 and rejects26. Runtime save/restore on the actual robot still needs the new run.
- Default and unpacked-reference **Pico W Release builds passed** strict warnings using existing Pico SDK2.2.0, ARM GCC14.2.1, CMake/Ninja. No dependencies added. Optional Buddy5 source is unchanged and not linked into the delivered UF2.

| ARM build | text | data | BSS |
|---|---:|---:|---:|
| Continuous v1 default |170320|0|237472|
| Continuous v2 default |171232|0|237508|
| Continuous v2 unpacked shared state |170176|0|242468|

V2 adds36B BSS over V1. Like-for-like shared-field packing still saves4960B; public packed interfaces do not change. The two diagnostic fields consume five existing padding bits without increasing row/RAM/flash payload size. This is partial representation evidence, not whole-firmware compliance.

Default symbols:`__flash_binary_end=0x10028ce4`,`__bss_end__=0x2003b474`,`__StackLimit=0x20040000`. The19340B symbol gap is not a measured worst-case stack/heap margin. Padded UF2 end`0x10028d00` remains below diagnostic bank0x101ab000. Diagnostic/nav/raw/motion offsets stay0x1ab000/0x1af000/0x1b0000/0x1d0000.

## Artifact

`firmware/buddy23_interface_continuous_v2.uf2`: **334336bytes**, **653RP2040 UF2 blocks**. SHA256:`591fa3c93e3e3cc33b900ec43b71f738964d87c17747b8db5df0f824cdc0e4a0`.

`CONTINUOUS_BASELINE.json` records V1 C/H source hashes. `CONTINUOUS_PATCHES.json` lists every V2 C/H change (historical key name`handover_sha256` now refers to this immediate V1 baseline). The motor controller, API/coordinator/executor/codecs/hooks, optical/calibration core, decoder and manoeuvre/search headers are unchanged. Historical manifests/build evidence copied with the snapshot describe their earlier versions, not this artifact.

`run_tests.py` runs the26 host executables;`verify_firmware.py` checks source scope, UF2 block integrity/checksum, preserved old artifacts, tracked publication file, default empty extension and flash/RAM bounds. CMake target remains`buddy23_interface_v1` for compatibility; package name/banner identify v2.

Today: profile1 on a straight, same mounting/calibration, normal startup; stop/wait5s and collect p+d. The full circuit harness remains enabled. **Physical reduction of weaving and actual left/right bend performance are not yet validated.** A later bend test is required despite restored turn authority in synthetic steady-input tests.
