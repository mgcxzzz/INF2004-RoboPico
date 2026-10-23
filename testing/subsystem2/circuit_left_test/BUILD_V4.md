# Circuit Left V4 build evidence

Built 2026-10-23 for Pico W / RP2040, Pico SDK 2.2.0, Arm GNU,
CMake/Ninja Release. C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_left_v4.uf2`

- 149,504 bytes; 292 sequential RP2040 UF2 blocks.
- SHA-256 `d39871db8fab26eb52a86028eaa616022196f90a33f27deb2fad0a1284bfed94`.
- Padded image ends at `0x10012400`; linker end `0x1001236c`.
- Payload is below the reserved log region starting at `0x101e0000`.
- ELF text 78,696 bytes; BSS 130,060 bytes. BSS end `0x20020dec`,
  stack limit `0x20040000`. USB init/write symbols verified.
- Previous V1–V3 UF2 hashes remain unchanged.

## Change and scope

The latest V3 profile 3 log stopped before any junction, with right level
1000 and targets 45/1 mm/s. The outer left wheel lacked accumulated forward
progress at the normal 30% correction cap. Post-turn assistance was ineligible.

V4 qualifies ordinary visible near-pivot underspeed for 120 ms and applies
existing turn feedforward only to the outer wheel, capped at 42%, without a
new 48% start pulse. It operates in all profiles. Inner target <=12 enters,
<=20 retains; outer target >=30 and at least one level >=400 are required.
Qualification requires outer measured speed <15 mm/s. Each episode lasts at
most 2 seconds /30 mm, followed by a 300 ms cooldown. Direction, phase,
visibility and stop changes also end assistance. Post-turn capture takes
priority. Steering targets, fast-cruise conditions, optical recovery and the
750 ms accumulated encoder-progress guard retain V3 behavior.

CSV assist values 3/4 identify ordinary-curve left/right outer assistance;
1/2 retain post-turn left/right meanings. Flash structure is unchanged and
accepts versions 1–4. Restored records retain their original version labels.
Shared junction, hump and historical line-following sources are unchanged.

## Executed checks

```powershell
gcc -std=c11 -Wall -Wextra -Werror test_circuit.c -lm -o test_circuit.exe
./test_circuit.exe
python verify_firmware.py 1
python verify_firmware.py 2
python verify_firmware.py 3
python verify_firmware.py 4
```

All pass, along with the Arm build. Tests retain twelve successive junction
turns per profile, complete steering-error parity, cruise qualification,
weak-signal recovery, capture effort, feedback, record CRC and ring checks.
New checks reproduce the V3 left45/right1 load case, qualification boundary,
both wheel directions, visibility/phase/stop/target/speed gates, hysteresis,
cooldown, time/path limits, priority and outer effort without an extra peak.

Physical validation is pending. Test profile 1 first, then repeat higher
profiles on the same section. This does not resolve a mechanical snag or
missing encoder feedback if either is the underlying cause.
