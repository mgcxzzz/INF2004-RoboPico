# v0.4.3 release verification

Verified on 21 November 2026. Firmware source commit
`8aa5f2cd876dea356fa6829caf8969a0232c40ec`.

| Check | Result |
| --- | --- |
| Build and identity | Clean `APP=rtos_buddy1_5`, `SMP=1` ARM build reproduced the published UF2 byte for byte. SHA-256 `3d03cc8bc2cf45c9bbc6032b06af95e17abb5cf0d56b8b2de0884c6644655a2e`. The linker reports the existing RWX LOAD-segment warning. |
| Application C | Mechanical checker and clang-format 18.1.8 passed on all 145 in-scope files. Review used the team's adapted Barr-C:2018 standard and its documented exceptions. The ported probe invokes the RTOS policy under `APP_RTOS_BUDDY5`. |
| RTOS and motion | The ELF links `tk_cre_tsk`, `tk_sta_tsk`, `tk_dly_tsk`, the motion and safety tasks, IMU and network tasks, and the Buddy 5 service. Buddy 5 requests motion through the coordinator lease. The motion task applies outputs through the shared implementation. The boot allocation model leaves 2,024 bytes in the kernel pool. |
| Compact data and transport | Compact signed values, native layout bounds, wire vectors and C-to-dashboard decode passed. The range warning adds an outcome value within the existing event frame. |
| Host and dashboard | 55 production C programs, Python helper checks and 28 dashboard tests passed. The local 64-bit host compiler downgraded its pointer-to-integer warning for the upstream 32-bit `libtm_printf` path. The ARM build used its normal settings. |
| Obstacle replay | The C fixture replays the exact borderline sonar batch. The simulated car performs the width scan, bypasses, and hands back to Buddy 3 without simulated contact. The real C-packed telemetry decodes as **accepted with range warning**. |

The physical circuit recording used [v0.4.2](../run-4/README.md). The exact
v0.4.3 Pico W run remains pending. The memory result is a static budget, and
physical timing and stop latency are separate measurements.

[Application C standard](../../../../CODING_STANDARD.md) ·
[Coding exceptions](../../../../docs/coding_exceptions.md) ·
[Build and test procedures](../../../TRIBUNAL_2.md)
