# V2.2 stack layout correction

User evidence: `logs/start_heartbeat_loss_2026-10-28.txt`. Telemetry-off heartbeats remain Ready through 17:36:59. Start is accepted in 48 ms at 17:37:00; Running heartbeat arrives at 17:37:02; LWT arrives at 17:37:10. This establishes broker-observed loss of the car connection after driving begins, not just a stale UI label. The log does not contain the underlying panic or CPU state. The user reports continued driving, no recovery after physical stop, and that simultaneous power-bank/USB power is unsuitable for their wiring.

## Compiled defect

The exact V2.1 Release application was compiled with `-fstack-usage` using the same compiler flags. Direct `bl` edges were verified in the linked ARM ELF:

| Simultaneously active call frame | Bytes |
| --- | ---: |
| main | 2344 |
| control_step | 1176 |
| interface_navigation_step | 472 |
| barcode_circuit_step.constprop | 336 |
| Sum, before further callees or interrupt context | **4328** |

V2.1 core0 SP starts at `0x20042000`. Subtracting these frames reaches `0x20040f18`, inside the live core1 stack `[0x20040800,0x20041000)`. The main function alone exceeds its nominal 2 KiB default allocation. This is a concrete cross-core stack-collision path, consistent with the network failure. Hardware corruption at that address was not captured directly; motor power/noise or another fault is not excluded by the message log.

`logs/stack_layout_baseline_v2_1.txt` checks the old ELF, compiler frames and real call edges. The old stack-usage report is retained in `logs/arm_stack_usage_v2_1.txt`.

## Corrected layout

V2.2 uses the pinned SDK's memory-region override symbols, without copying or modifying the SDK linker scripts. Main SRAM is shortened by 16 KiB for BSS/heap, and the SDK stack-region aliases are assigned to that reserved area. Target definitions reserve 8192 bytes per stack in the SDK's normal stack sections.

| Region | Half-open address interval |
| --- | --- |
| BSS / heap RAM region | `[0x20000000,0x2003c000)` |
| Core1 stack | `[0x2003c000,0x2003e000)` |
| Core0 stack | `[0x2003e000,0x20040000)` |

`__HeapLimit` and `__StackLimit` are `0x2003c000`, so SDK allocation cannot grow into either stack. `core1_stack` starts at its linker-assigned bottom and is 8192 bytes. The boot vector's initial core0 SP is `0x20040000`. Physical scratch SRAM is no longer used for these stacks. Flash offsets and saved calibration/Wi-Fi/run record layouts are unchanged.

`verify_stack_layout.py --build BUILD_DIRECTORY` checks actual symbols, the real core0 driving call edges and compiler frames, two 8 KiB stack allocations, heap/BSS separation and more than 1 KiB margin above the measured chain lower bound. This is not a complete call-graph worst-case stack bound or a runtime watermark. Run it against an ARM Release build, not host binaries. `verify_firmware.py` additionally checks the packaged boot SP and recorded stack bounds.

No controller, motor output, MQTT retry or packet format logic changed for V2.2. The timer-pool fix remains present. Required hardware result: continuous heartbeats before, during and after Start/driving/physical Stop, first telemetry-off then telemetry-on. Physical V2.2 result: **PENDING**.
