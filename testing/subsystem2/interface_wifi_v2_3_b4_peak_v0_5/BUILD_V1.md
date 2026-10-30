# Interface V1 build and verification — 2026-10-26

Baseline: V12.2 commit `9df1b95` on `codex/buddy2-left-motor-test`. Only this new `interface_v1` folder is added. `V12_2_BASELINE.json` records 57 copied source/test/document files; `V12_2_PATCHES.json` records four intentional changes in that snapshot: hardware-main routing/hooks/log integration, record-version acceptance, and the two corresponding record-version tests. `scan_smooth.h`, decoder/navigation/search/turn/calibration/motor headers are unchanged from V12.2.

## Executed verification

- `C:/Python310/python.exe testing/subsystem2/interface_v1/run_tests.py`: **17 host executables passed**, compiled with Zig C11 `-Wall -Wextra -Werror`. Twelve retained V12.2 tests, coordinator/codec tests in packed and unpacked modes, coordinator log storage, Buddy5 dispatcher workflow, and routing parity.
- Routing parity compared **9,000 optical/PI ticks** across profiles 1/2/3, both steering directions, weak/strong readings and loss/recovery. Coordinator AUTO forwarding retained the same wheel targets and ordinary PI output for identical simulated inputs. Existing A/F/Z, cross/turn/search and signed-power fixtures also pass. This is not a physical road test.
- New API tests cover ownership and Buddy5 pre-emption, measured stopping, command IDs/results, stale leases/commits, reverse/rolling/in-place execution models, interrupted-junction handback, cancellation, faults and timeouts, terrain telemetry/unsupported assistance, 0.1 mm codec roundtrips and bounded audit overwrite.
- Log tests reject erased/torn headers, data/header corruption and invalid event padding, and accept full/empty banks.
- Pico W Release ARM builds passed for the default weak-hook firmware, identical-feature unpacked measurement reference, and opt-in example extension. Compiler checks use `-Wall -Wextra -Werror`. The example linked a strong `buddy_extension_poll`; its sensor/planner inputs remain weak no-op defaults.
- `verify_firmware.py` passed source manifest/patch hashes, original V12.2 artifact hash, RP2040 UF2 family/block/address structure, embedded interface/legacy markers, packaged-vs-build equality, and image bounds below the independent log sector.

Toolchain: Pico SDK 2.2.0, ARM GCC 14.2.1, local CMake/Ninja, Pico W target, USB serial only. Optional uninitialized Bluetooth/Wi-Fi SDK submodules warn at configure time; neither is linked or used by this standalone firmware.

## ARM size comparison

| Build | text (arm-size) | data | BSS |
|---|---:|---:|---:|
| Original V12.2 | 127128 | 0 | 215408 |
| Interface V1, same features with unpacked state | 149224 | 0 | 227024 |
| Interface V1, published packed state | 150424 | 0 | 222064 |

Packing saves **4960 B of BSS** against the same interface with unpacked fields. The new interface adds **6656 B of BSS** over V12.2 because it introduces coordinator state/results/audit buffers. Code size also increases; compact-field access costs some instructions. These are measured build sizes, not proof of whole-firmware minimum-bit compliance or faster execution.

`__bss_end__=0x20037800`, `__StackLimit=0x20040000`: 34,816 bytes between those symbols remain for the usual heap/main-stack region; this is not a measured free-heap or worst-case stack claim. Buddy5's additional code/buffers/stacks must fit and be checked again. Encoder capture and the existing 10 ms control/100 ms lateness protection remain, but on-target timing overhead has not been measured.

## Published artifact

- File: `firmware/buddy23_interface_v1.uf2`
- Bytes: 292864; 572 contiguous 512-byte UF2 blocks with 256-byte payloads.
- RP2040 family: `0xe48bff56`; base `0x10000000`.
- Linker image end: `0x10023b9c`; padded UF2 end: `0x10023c00`.
- Coordinator sector: `0x1af000..0x1affff`; raw log begins `0x1b0000`; normal motion log begins `0x1d0000` in the standard 2 MiB flash layout. Image is below all log banks.
- SHA256: `abef641045462f2e452265e352e32de996fd2b3d63c70551f0c3e0186b1b52dc`.
- Original V12.2 UF2 remains unchanged: `76941390c3779295ff750596a76d84e575a92c2b26bc78a3203771efa052641c`.

To reproduce the RAM comparison, configure a second build with `-DINTERFACE_UNPACKED_REFERENCE=ON` and otherwise identical source/toolchain/options. Do not use that measurement build as the handoff firmware. The public artifact uses packed fields and default weak buddy hooks.

## Remaining physical verification

Run the default baseline on a straight/bend/barcode/junction first. Then integrate and test an obstacle stop, one reverse, one finite turn, and explicit resume after clearance and line reacquisition. Validate actual turn angle/slip/clearance and capture `z` with `p`. Generic avoidance maneuvers, actual flash persistence on the user's device, and control timing with the ultrasonic/servo driver have not been physically tested here. IMU speed assistance is deliberately unsupported in V1; no ramp behavior is silently enabled.
