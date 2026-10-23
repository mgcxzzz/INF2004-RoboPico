# Circuit with barcode V1 build evidence

Built 2026-10-23 for Pico W / RP2040 using Pico SDK 2.2.0, Arm GNU,
CMake/Ninja Release, C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_with_barcode_v1.uf2`

- Size: 184,320 bytes; 360 sequential RP2040 UF2 blocks.
- SHA-256: `ee1c7fc1991b8a78fce1d42368684e29eb9672e2a2ab989d0b141b2ae56c5fcf`.
- Padded end: `0x10016800`; linker end: `0x100167cc`.
- ELF text: 96,200 bytes; BSS: 144,096 bytes.
- BSS end: `0x20024580`; stack limit: `0x20040000`.
- USB init/write symbols verified. Firmware is below the new barcode record
  bank at `0x101d0000`. Existing circuit and tail record banks are preserved.
- Saved record: 64,840 bytes within a 65,536-byte bank; trace 56 bytes,
  summary 40 bytes, barcode event 32 bytes. CRC corruption is rejected.

## Executed checks

Host `test_barcode.c` compiles with C11 warnings as errors and passes:

- Independently measured runs from all four supplied sample PDF pages:
  A/A/Z/Z at the two print sizes, forward and reverse.
- Standard A/F patterns at narrow widths 1.5 and 3 mm, wide:narrow 3:1,
  both scan directions, element jitter and varying timing/speed.
- Malformed stop sentinel, partial-frame expiry, non-barcode stripe rejection.
- D0 debounce, first-candidate position/timestamp, queue wrap and overflow.
- A/F pending replacement; unsupported Z preserves a prior valid command.
- Separate cross width recognition; no-command straight crossing; gate and
  post-turn clearance rejection retain pending; accepted cross consumes once.
- Twelve alternating A/F turns per profile (six left, six right), both sizes,
  reverse scans and barcode-to-cross gaps of 0.5, 30, 100 and 300 mm.
- Mirrored right motor demands and real-world follower handover afterward.
- Record ABI and CRC validation.

Arm firmware build and `verify_firmware.py` pass. The verifier checks UF2
headers/family, sequential addresses, payload boundaries, version/event
markers, hash and equality with the built artifact.

The PDF specification and all four sample pages were visually inspected.
The sample sheets are A/Z; actual routing remains A/F as requested. The
Code 39 table was checked against ZXing's primary implementation linked in
the README. `fixtures/pdf_runs.h` holds measured widths, not copied PDF pages.

The supplied latest Circuit Left V6 log is truncated and lacks its complete
ending, so it is not evidence of a fault-free full course. V6 is retained as
the follower baseline without editing its historical source or artifact.

Software checks do not prove physical barcode readability, right-turn grip
or course completion. Start with separate A and F runs in profile 1 and
inspect the saved barcode events before a continuous circuit.

## Reproduce

From this folder, with the toolchains on PATH:

```powershell
gcc -std=c11 -Wall -Wextra -Werror test_barcode.c -lm -o test_barcode.exe
./test_barcode.exe
cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release
cmake --build build
python verify_firmware.py
```

Set `PICO_SDK_PATH`, Ninja and `picotool_DIR` to the locally installed SDK
tools as required. The verifier checks the delivered firmware copy; after a
new build copy the UF2 from `build/` to `firmware/` before verifying it.
