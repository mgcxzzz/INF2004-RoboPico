# Height V1 ARM stack layout

Inherited from V2.3 and verified against the real Release ELF. SDK commit98a542c1a62fb549ffb5d66a3e5892b06276b670; GCC14.2.Rel1.

| Region | Half-open interval |
| --- | --- |
| BSS / heap RAM | [0x20000000,0x2003c000) |
| Core1 network stack | [0x2003c000,0x2003e000) |
| Core0 controller stack | [0x2003e000,0x20040000) |

Known compiler frames on the verified chain total4424 bytes: main2440, control_step1176, interface_navigation_step472, barcode_circuit_step336. This excludes deeper calls and interrupts, and is not a complete worst-case stack bound or runtime watermark. Logs contain the compiler .su report and ELF checks. Hardware performance remains pending.
