# V1.1 HANDOVER1 build evidence — 2026-10-27

This test starts from V1.1 DIAG1 at949d4b7 and changes only eligibility-gated ordinary cruise-exit PI-state handling. Existing algorithms/gains/caps remain. Physical smoothness and actual timing overhead require the next run; neither is established by host tests.

## Executed checks

- 22 host executables passed: all21 DIAG1 API/clock/stop/ownership/storage/Buddy5, optical/Code39/turn/search and observation parity checks, plus the handover regression/model executable.
- 12,000 optical updates still match exact V1.1 requests, levels, decisions, phase and filter state:7164 filtered,3471 urgent,1365 bypass. Original motor/API/coordinator source headers are unchanged. Only the guarded call at the old cruise exit uses the new adapter.
- Captured2.522461s inputs reproduce old103/110PWM and yield H1=168/181 while I remains53.44/58.80. Captured3.052145s inputs reproduce old108/140 and yield155/166 with I38.67/21.54. Speeds/targets are held as recorded for these checks; these are computed transitions, not predictions of the new physical trajectory.
- 1076 unsupported-context combinations preserve exact legacy handover state. Explicit boundary cases reject large overspeed, invalid speed, low target and turning/reverse state. Existing zero, low-inner-wheel coast and substantial overspeed behavior pass.12 simple varying-load wheel cases settle55->50mm/s without restarting after eligible handover; real stiction/slip/optics may differ.
- Diagnostic format2 tests field extrema/sign/clipping, new preserved-handover trigger, ring chronology, whole-run timing after freeze, CRC corruption/torn-header rejection and reboot restoration. A CRC-valid old format1 header is deliberately rejected. Normal motion record23 imports earlier calibration/records1..22.
- Default Pico W, unpacked shared-state reference and Buddy5 example Release builds passed `-Wall -Wextra -Werror`. Pico SDK2.2.0, ARM GCC14.2.1, local CMake/Ninja/USB CDC.

| ARM build | text | data | BSS |
|---|---:|---:|---:|
| V1.1 original |150832|0|222064|
| V1.1 DIAG1 |163112|0|237372|
| HANDOVER1 packed |163696|0|237372|
| HANDOVER1 unpacked shared fields |162520|0|242332|
| HANDOVER1 Buddy5 example |166416|0|237404|

The handover adds no persistent RAM over DIAG1. Diagnostic payload remains14848B; total BSS overhead versus originalV1.1 remains15308B. Existing shared-field packing still saves4960B like-for-like. Diagnostic row459bits/58bytes,5padding bits. This is partial data-representation evidence, not whole-firmware compliance.

`__flash_binary_end=0x10026f74`, padded UF2 end `0x10027000`, below the diagnostic bank0x1ab000. `__bss_end__=0x2003b3ec`, `__StackLimit=0x20040000`:19476B between symbols, not a measured worst-case stack/heap margin. Diagnostic/nav/raw/motion banks remain0x1ab000/0x1af000/0x1b0000/0x1d0000. No extra in-run UART or flash writes.

Artifact `firmware/buddy23_interface_v1_1_handover_v1.uf2`: 319488bytes, 624 contiguous RP2040 UF2 blocks. SHA256 `ca15cf9d0f680e00b313d7d6189d3b540098ec54c47030bc47f64b4dd709524e`. DIAG1 and V1.1 artifacts remain unchanged in their original folders. Public APIs/start/save/dump behavior is retained; h shows HANDOVER1, p identifies normal record23, d shows capture2 and the preserved-cruise selection bit.

Source hashes in DIAG1_BASELINE.json/HANDOVER_PATCHES.json and V1_1_BASELINE.json/DIAG_PATCHES.json record the exact inherited/control/diagnostic changes. Run verify_firmware.py for packaged identity, source checks and flash bounds.

First physical test: same mat, mounting, calibration and profile1 as the diagnostic run. Run normally through the straight and bend, then stop/wait5s and send p+d. Do not simultaneously change speed, calibration or sensor mounting. d now also triggers on an eligible preserved handover, even if its PWM change is smooth, so that this transition remains observable. No repeat of the fixed-speed probe is required.
