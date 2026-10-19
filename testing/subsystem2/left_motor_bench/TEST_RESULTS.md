# Left M1 hardware test record

Hardware results: **NOT RUN by Codex. Complete on the robot.**

- Tester / date:
- Firmware SHA-256:
- Board / motor model:
- Encoder supply and output-voltage compatibility verified from:
- Power arrangement:
- Serial log filename:

| Check | Observation | Pass / fail / not run |
|---|---|---|
| Boot does not drive either motor | | |
| GP20/GP21 displayed correctly | | |
| Manual wheel turn: both A/B edge counts increase | | |
| Opposite hand rotation reverses net count direction | | |
| Slow hand turn: invalid count | | |
| Without GP21 held: f/r rejected | | |
| f pulse: actual direction, counts, current not measured | | |
| r pulse: actual direction, counts | | |
| GP21 release removes drive | | |
| GP20 removes drive | | |
| 3000 ms timeout removes drive while GP21 held | | |
| Same held press cannot trigger another pulse | | |
| Right motor remains off (currently unwired) | | |

Notes / issues / next change:

Wheel counts per revolution and mm/s are **not calibrated** by this test.
