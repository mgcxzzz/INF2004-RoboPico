# Wi-Fi V2.3 validation - 2026-10-28

Base: working SDK V2.2 at `adcfb6c91326f1636aecd90d7b64e03c810e1411`. This is not another UTK build. The user and teammate report successful physical testing of that V2.2 base.

| Check | Result and scope |
| --- | --- |
| Host suite | PASS: 35 C test executables, 10 JavaScript schema tests, 20 Python helper tests; assertions enabled. |
| Calibration | PASS: exact preset, legacy 26/27 override, own saved 28 accepted, invalid CRC rejected, immutable restore source. ARM build compiles stopped-only `c` command. |
| Controller/transport | PASS: existing steering, motor, API/nav, barcode, coordinator and transport tests; C-to-dashboard packet vectors. Radio and motors are not exercised by host tests. |
| Source preservation | PASS: Continuous V2 headers, all network/API/nav files, optical/maneuver algorithms and dashboard match V2.2. Only the approved calibration/record/banner/output-name changes alter firmware sources. |
| ARM network ON Release | PASS with warnings-as-errors, no embedded credentials, no default Buddy 5 extension. |
| Real SDK lwIP allocator | PASS: original core-only pool reproduces exact exhaustion; fixed 7+4 slots pass 100 application callback reschedules and 1000 add/remove cycles. |
| Real ELF stack/heap | PASS: separate 8 KiB stacks, heap below both, driving call edges and 4424-byte known frame chain verified. Not a complete worst-case stack bound. |
| Release integrity | Verify with `python verify_firmware.py`: source hashes, approved diff, UF2, flash/RAM bounds, package. |
| V2.3 real heartbeat/Start/driving | PENDING: use SETUP.md and save the actual results. No robot was flashed from this development workspace. |

Evidence is in `logs/host_tests.txt`, `logs/lwip_timeouts.txt`, `logs/stack_layout.txt`, `logs/arm_compile_tail.txt` and `logs/arm_stack_usage.txt`. The compiler and SDK versions, memory totals and artifact checksum are in BUILD_METRICS.json. Historical baseline documents copied into this folder are not fresh V2.3 test evidence.
