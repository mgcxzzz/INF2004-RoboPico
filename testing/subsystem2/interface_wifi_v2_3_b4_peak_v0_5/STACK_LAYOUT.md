# V2.3 stack layout

The V2.2 stack fix is retained byte-for-byte in CMake except the output filename. The SDK is pinned to its V2.2 commit, `98a542c1a62fb549ffb5d66a3e5892b06276b670`.

| Region | Half-open address interval |
| --- | --- |
| BSS / heap RAM | `[0x20000000,0x2003c000)` |
| Core 1 network stack | `[0x2003c000,0x2003e000)` |
| Core 0 controller stack | `[0x2003e000,0x20040000)` |

`verify_stack_layout.py` inspects the actual ARM ELF, compiler frame report and real call edges. `logs/stack_layout.txt` records the V2.3 result. The known driving chain uses 4424 bytes with GCC 14.2; this excludes deeper callees and interrupt frames. Each stack is 8192 bytes; heap allocation stops below both. The initial boot SP is `0x20040000`.

This is static evidence, not a complete runtime worst-case bound. V2.2's physical success was reported by the user; V2.3's physical test remains pending.
