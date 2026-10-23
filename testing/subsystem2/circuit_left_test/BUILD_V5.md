# Circuit Left V5 build evidence

Built 2026-10-23 for Pico W / RP2040, SDK 2.2.0, Arm GNU,
CMake/Ninja Release; C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_left_v5.uf2`

- 143,872 bytes; 281 sequential RP2040 UF2 blocks.
- SHA-256 `d00ccd841d5d23786e7ee6c8ba6461c082ffd81ee3f439c858e7c3a8e3db021a`.
- Padded end `0x10011900`; linker end `0x10011824`.
- All payloads below the record region starting `0x101e0000`.
- ELF text 75,808 bytes; BSS 130,164 bytes; BSS end `0x20020e64`,
  stack limit `0x20040000`. USB init/write symbols verified.
- V1–V4 artifacts retain their previous hashes.

## Changes

Recovery direction changes require opposite strong asymmetry (level >=400,
difference >=200) sustained for 80 ms. Weak observations may seed or refresh
the same direction after 30 ms but cannot reverse existing memory. Ordinary
optical steering remains immediate and preserves the full target table.

Both levels <150 enter a loss episode. Its direction and deadline stay latched
through weak flashes. Return requires level >=400 for 40 ms, or >=300 for
80 ms. A reliable curve command may bridge the first 150 ms at reduced speed
before the normal 0/45 search. Directed recovery remains bounded at 2 s/50 mm.
A separately confirmed, recent centred observation can bridge straight at
35/35 for at most 500 ms/15 mm. Unknown position does not invent a turn.
Left and right cases use the same logic without course coordinates.

At the user's explicit request, the 750 ms accumulated encoder-progress
detector is now diagnostic only. It does not set the stop reason. Warning
bits remain visible and saved. Manual stop, USB changes, invalid transitions,
direction, control timing and bounded optical/junction checks remain. A real
jam can therefore require manual stopping.

V4 outer-wheel assistance is retained, with no assistance during an
unconfirmed optical recovery episode. Cruise profiles and explicit rolling
left-turn speeds/limits remain unchanged.

Saved rows append memory, recovery, reacquisition and encoder-warning fields
using previously unused phase/side bits, preserving the 48-byte trace and
64 KiB record ABI. Raw printed phase/side are masked correctly. V1–V5 records
read correctly. State transitions add rows between the usual 250 ms samples;
the latest 1,200 rows can cover less than five minutes when events are frequent.
Whole-run totals and the latest 64 turn summaries remain.

## Executed checks

- Host C11 suite, warnings as errors: pass.
- Twelve successive junction turns per profile; steering-error sweep from
  -1000 to +1000; cruise qualification and immediate steering exit: pass.
- V4 right-bend → loss → weak opposite flash → loss and its exact left-bend
  mirror, including weak and brief strong opposite flashes: pass.
- Genuine return, genuine 80 ms direction reversal, 40/80 ms reacquisition,
  interrupted dwell and400-threshold flicker, preserved deadline, new episode after confirmed return,
  curve bridge and straight-gap time/path bounds: pass.
- Existing weak-signal, capture/loaded curve effort, no additional peak,
  cumulative slow/bursty versus absent encoder progress, CRC/import/ring,
  and packed diagnostic round-trip checks: pass.
- Pico W Arm build, memory boundaries, USB symbols and delivered UF2 identity:
  pass. `verify_firmware.py` passes for revisions 1 through 5.

Shared historical junction, hump and analog follower sources are unchanged.
Physical full-course completion is not established by software tests. Start
with profile 1 on both bend directions and the junctions, then compare faster
profiles only after that run is consistent.
