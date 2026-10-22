# Hump line V3 build, 2026-10-22

Artifact: `firmware/buddy23_hump_line_v3.uf2`

- SHA-256: `d811c358f7c0e7c5ff40503492106f83f42fbd0df4a4fd4cc4e7e34c75f24305`
- 109056 bytes;213 sequential RP2040 UF2 blocks,256-byte payloads.
- Payload `[0x10000000,0x1000d500)`; ELF end `0x1000d484`.
- No image writes in hump-record0x101fe000 or junction-record0x101ff000 sectors.
- Arm size: text58496,data0,BSS12104 bytes. USB stdio present;UART stdio absent.
- Pico SDK2.2.0, Arm GCC14.2.1, Pico W Release,C11,warnings as errors.

V3 multiplies final V2 effort1.20x, rounded to nearest permille. This covers
normal forward drive, steering, breakaway, recovery and load assistance.
Physical ordinary ceiling300, assisted420, maximum extra180 permille. Speed
targets and all guards remain unchanged. The PI/anti-windup and assistance
state use V2 effort units internally; the boundary adapter and logs use physical
scaled PWM. Zero stays off. Feedback can lower duty as speed recovers.

Passed host checks: expected scaled V11 baseline output across both profiles
and wheels; intended slowdown; sustained per-wheel underspeed; stalled20 and
32->20 regressions; caps and bridge polarity; overspeed/recovery/zero removal;
V1/V2/V3 record validation; V9/V10 junction front-calibration import; optical,
button and late-loop guards. UF2 verification checks family, sequence, addresses,
banner, equality to compiled image, and hash.

V2 small black hump passed by user report; orange failed without an attached
failure log. V3 physical tests are pending. No climbability guarantee is inferred.
Prior V2 artifact hash remains
`fd26ed434630a4b0ac03f6b6f1c1f556bc7083af834af0c0e098342f2b78b94b`.
