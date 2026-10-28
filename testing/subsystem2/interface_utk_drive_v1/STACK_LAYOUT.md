# Kernel task stacks and the V2.2 SDK correction

The teammate's `interface_wifi_v2` commits 24638a7/adcfb6c reserve separate SDK core stacks to correct the old SDK stack-overlap path. Those branch changes are retained. This kernel build does not use the SDK multicore launcher or its core-stack regions.

- `app_main.c`: one driving task, TP_PRC2 / physical core1, stack 12288 bytes allocated by T-Kernel.
- `cyw43_utk_kernel.c`: network service TP_PRC1 / physical core0, stack 8192 bytes.
- USB service: separate kernel task on physical core0, stack 4096 bytes.
- The kernel allocator gives tasks distinct live stack allocations; no unreserved scratch-SRAM core1 stack is used. Allocation failure leaves motors off and reports initialization failure.

Using the actual ARM GCC 14.2 Release flags and `-fstack-usage`, the compiler reports these frames for the driving path: utk_drive_main1320, control_step824, interface_navigation_step464, barcode_circuit_step304 bytes, summing to 2912 bytes before further callees. This is not a complete worst-case call graph or runtime stack watermark. IRQ handling has the kernel's separate exception-stack provision.

The INTERNAL+4 lwIP timer allowance and compile-time guard remain enabled. All networking APIs run in the kernel Wi-Fi service, and drive task communication uses bounded message buffers with TMO_POL. The first hardware check must verify continuing heartbeats before, during and after a profile1 run with telemetry enabled. A real driving result is not yet available for this image.
