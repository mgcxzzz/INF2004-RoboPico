# Circuit Left V6 build evidence

Built 2026-10-23 for Pico W / RP2040, SDK2.2.0, Arm GNU,
CMake/Ninja Release; C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_circuit_left_v6.uf2`

- Size148,992 bytes;291 sequential RP2040 UF2 blocks.
- SHA-256 `8012143e7ca3c3509a3a42acd99f21576c3cbfead96d6e295f7900a2992429d0`.
- Padded end `0x10012300`; linker end `0x100122dc`.
- All payloads below the record region starting `0x101e0000`.
- ELF text78,552 bytes; BSS130,212 bytes; BSS end `0x20020e94`,
  stack limit `0x20040000`. USB init/write symbols verified.
- V1–V5 artifacts remain unchanged.

## Evidence and response

The V5 user log ends in a left search,2 s expiry, zero completed junctions.
At10.873 s the right level is243 but V5 continues0/45 targets and does not
begin return confirmation. Earlier9.608 s requested46/8 is measured21.5/34.6
with PWM104/216: actual wheel response still yields left rotation while the
optical controller is requesting right. The user confirms the same bend.
These observations establish a strict recovery gate and response lag; they
do not prove a particular hardware failure or how long the243 reading lasted.

V6 addresses both:

- Asymmetric faint return (stronger>=200,difference>=150) can guide a slow
  relative-balance probe after30 ms with a consistent sign. Faint confirmation
  and memory reversal require120 ms of consistent evidence; stronger return
  still confirms at>=300/80 ms or>=400/40 ms. Probe failure does not restart the
  original loss deadline. Alternating faint signs cannot accumulate probe dwell.
- Visible faint steering also uses `(L-R)/(L+R)` and the existing steering
  shape scaled to outer35 mm/s. Thus0/243 gives35/0 instead of a shallow50/40.
  Strong-signal steering and all cruise profile speeds retain their targets.
- Ordinary correction requested-speed ramps decelerate at400 instead of100
  mm/s²; acceleration remains100. Positive integral clears while decelerating
  above target+5. An inner wheel requested1–15 mm/s coasts at PWM0 while
  measured speed exceeds target+5, then resumes regulation near target. No
  reverse or active brake is added. Explicit loaded junction-turn controller
  output is unchanged in regression comparisons.
- Ordinary loaded-curve assistance qualifies below70% of target for120 ms,
  retaining42% sustained cap,2 s/30 mm maximum and300 ms cooldown. It ends
  when measured outer speed exceeds target+5. This catches21.5 vs46 which
  V5's fixed15 threshold misses. No extra pulse or higher effort limit is added.

V5's warning-only encoder progress is retained. Directed recovery remains
2 s/50 mm, recently centred gap500 ms/15 mm, and existing manual/control/USB/
direction/invalid/junction limits remain. All changes are mirrored left/right.
Historical shared junction, analog and hump files are unchanged.

## Diagnostics and compatibility

Decisions17/18 mark left/right probes; their transitions create trace rows.
The existing phase/side diagnostics and48-byte trace ABI remain. V1–V6 saved
records read correctly and retain their version labels. New V6 runs save as6.
Calibration and prior firmware payloads are preserved. Progress feedback is
still warning-only and never sets the V6 stop reason.

## Executed checks

Host C11 tests pass, including twelve successive junction turns per profile,
strong steering parity, cruise gating, historical weak-signal cases, memory/
reacquisition timing, bounded time/path, gap bridges, record CRC/import/ring,
and effort limits. New mirrored V5 regressions cover faint243 return,30 ms
probe,120 ms confirmation/reversal, failed probe budget, alternating-sign
rejection, inner34.6/target8 coasting,400 mm/s² ramp, relative underspeed and
overspeed exit. Explicit turn output is compared against the existing loaded
junction controller for both wheels. Arm build, USB symbols, memory bounds
and UF2 validation pass. `verify_firmware.py` supports revisions1–6.

Software tests do not establish physical course completion. Start with
profile1 and compare the same bend, then continue through the junctions.
