# Circuit Left V3 build evidence

Built 2026-10-23, Pico W/RP2040, SDK2.2.0, Arm GNU, CMake/Ninja Release
C11, warnings as errors (`-Wall -Wextra -Werror`).

Artifact: `firmware/buddy23_circuit_left_v3.uf2`

- Size148,480 bytes;290 sequential RP2040 UF2 blocks.
- SHA-256 `27866927b310234a0d6f7825c1334181a39f03539a10c051334ec8cd5b64be24`.
- Padded image ends `0x10012200`, linker end `0x100121c4`.
- All payloads below the reserved record region starting `0x101e0000`.
- ELF text78,272 bytes, BSS130,024 bytes; BSS end `0x20020dd8`, stack
  limit `0x20040000`. USB init/write symbols verified.
- V1 and V2 UF2 files remain byte-identical to their recorded hashes.

## Changes

Circuit-local recovery uses2 seconds/50 mm with1.5-second direction memory.
Strong direction observations retain their original immediate rule. A weaker
asymmetric observation (max level>=200, difference>=150) needs a consistent
sign for30 ms before it refreshes direction. Existing optical target mapping,
centred cruise qualification and speed profiles remain unchanged. Shared
historical junction, line-following and hump sources are not modified.

Outer-wheel effort during a visible near-pivot capture correction can retain
turn feedforward, capped42%, no additional48% start pulse. Scope is3 seconds/
45 mm after branch confirmation, ending on sustained two-wheel rolling.
Inner-wheel targets/effort remain ordinary. No extra effort during blind loss.

Encoder feedback now requires four accumulated forward transitions (~0.32 mm)
to renew a750 ms progress window. Small samples accumulate; backtracking/jitter
does not automatically reset the window. The direction, invalid-transition,
late-control, buttons and bounded turn/line guards remain.

Flash record structure is unchanged; versions1–3 read correctly. New rows pack
capture-assist flags into previously unused high decision bits. Dump prints
the original decision and an appended assist column separately. Previous
records decode assist0 and retain their original version/feedback wording.

## Executed checks

```powershell
gcc -std=c11 -Wall -Wextra -Werror test_circuit.c -lm -o test_circuit.exe
./test_circuit.exe
python verify_firmware.py 1
python verify_firmware.py 2
python verify_firmware.py 3
```

Host result:

```text
PASS:12 turns/profile; steering parity/cruise; observed weak-signal regression;2s/50mm bounds; scoped capture effort/no new peak; cumulative feedback/stall; V1-V3 records/CRC/ring.
```

Tests cover the observed weak-right-signal failure against V2 and V3; recovery
timeout/distance/unknown/stale direction; weak sign dwell and reacquisition;
capture time/path/visibility/inner-target/rolling bounds; outer effort with no
new peak; slow/bursty progress versus no progress/jitter; existing steering
error sweep, cruise qualification, repeated turns, gate/cap/CRC/ring checks.

Arm build and delivered UF2 validation pass. Simulations use idealized sensor
and encoder inputs; physical curve capture and motor loading still need lab
validation. Begin with profile1, then profile2 only after the first loop works.
