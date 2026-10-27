# V1.1 DIAG1 build evidence — 2026-10-27

This is an observation-only fork of InterfaceV1.1 commit72e11ce. It does not incorporate V1.2/V1.3 controllers. Physical timing overhead remains to be measured by this firmware; no improved physical tracking is claimed.

## Executed checks

- 21 host executables passed, including all retained V1.1 API/clock/stop/ownership/storage/Buddy5 and V12.2 navigation/barcode/turn/search tests.
- 12,000 optical updates compare instrumented steering directly with the exact frozen V1.1 reference: identical requests, calibrated levels, decision, phase and smoothing state across all3 profiles,7164 filtered,3471 urgent-bypass and1365 other bypass updates. New taps only copy observations.
- The diagnostic codec tests every field's extrema, sign, saturation and clipping flag.458bits are stored in58bytes. Trigger/ring tests retain127 pre-trigger,1trigger,128post-trigger samples; exercise no-trigger latest-window fallback and early stop. Timing metadata updates after freeze. Header/data corruption, missing/partial header, reboot restoration and matching metadata are checked.
- All original motor/control headers and API/coordinator sources are byte-identical to V1.1. The only production changes are the outer diagnostic wrapper, hardware observation/storage/dump calls, optical observation taps, and saved-record acceptance/version. The source manifests and DIAG_PATCHES.json identify all other support/documentation changes.
- Pico W Release builds passed warnings-as-errors for default, unpacked shared-state reference and opt-in Buddy5 example. Toolchain: Pico SDK2.2.0, ARM GCC14.2.1, CMake/Ninja, USB CDC.

| ARM build | text | data | BSS |
|---|---:|---:|---:|
| V1.1 original |150832|0|222064|
| V1.1 DIAG1 packed |163112|0|237372|
| Same diagnostic features, unpacked shared fields |161936|0|242332|
| Buddy5 example + diagnostics |165832|0|237404|

Diagnostics add15308B BSS overV1.1, including14848B row payload and observation/metadata state. Like-for-like shared-field packing still saves4960B. The new diagnostic payload is itself explicitly bit-packed. This remains partial minimum-bit compliance; legacy fields are not migrated by this release.

`__flash_binary_end=0x10026d2c`, padded UF2 end `0x10026e00`, below the new diagnostic bank0x1ab000. `__bss_end__=0x2003b3ec`, `__StackLimit=0x20040000`:19476B between symbols. This is not a measured worst-case free heap/stack margin. Check runtime timing and any Buddy5 extension's additional memory before integration.

Diagnostic header256B+payload14848B fits the independent16KiB bank0x1ab000..0x1af000. CRC-checked header is written last, after stopped-only data programming. Existing nav/raw/motion banks are untouched. Motion record22 imports1..21; downgrade calibration checks remain necessary.

Artifact `firmware/buddy23_interface_v1_1_diag_v1.uf2`: 318464bytes, 622 contiguous RP2040 UF2 blocks. SHA256 `e4891f5568d8a54e61be458368e04fee1630f9361abe29d3c6a8794ae82f1c3a`. Original V1.1 artifact remains unchanged in ../interface_v1/firmware.

The control-cost observation includes normal body work and snapshot/packing, excluding preceding ADC work and the final timing-bit update. max_pack_us covers the row-push/packing block only. Neither metric alone is an exact logger-overhead benchmark. No in-run diagnostic UART output or flash operation occurs.
