# Junction left V2 — 21 October 2026

Artifact: buddy23_junction_left_v2.uf2,112640bytes,220 UF2 blocks.

SHA256:

```
d481f9771c1c20c7b2e1ca9dde6a525d85b5b5ed8580f91dfe31d4083858d3a9
```

Change from V1: during settled in-place LEFT turn only, use220permille as the
minimum feedforward estimate, with the same signed PI, ramp, breakaway and250
output cap. Measured overspeed can reduce actual output BELOW220; zero request
clears PI/kick and outputs0. Forward controller is unchanged. Turn direction
remains left backward/right forward. State-machine geometry and all limits
remain unchanged, including500ms no-motion cutoff and6s/110deg turn bounds.

Verification passed:

- Strict C11 host suite (-Wall -Wextra -Werror -O2) including existing staged
  junction/branch rejection/settle/guard tests and signed bridge output sweep.
- New stationary-under-load turn test: sustained250 after startup pulses end,
  both signed directions; overspeed reduces output, zero resets state.
- Forward phase output-by-output comparison against V11 PI+breakaway across
  stop/start/speed changes on each wheel: exact equality.
- V1 record typedefs taken from actual commit73f2b6e compared with V2 definitions:
  identical3792byte record/36byte trace, row/Junction offsets, CRC coverage;
  zero-filled legacy padding revision0 loads as V1, new revision2 is stored.
  Front/side calibration and old trace survive the record reinterpretation.
- Pico W Release SDK2.2.0, Arm GCC14.2.1; app -Wall -Wextra -Werror.
- ELF text60400/data0/BSS12184, firmware end0x1000dbf4 below reserved0x101ff000.
- UF2 RP2040 family0xe48bff56, headers/sequence/payload lengths and all flash
  addresses verified. USB stdio linked; UART init absent.
- No V11 source/controller/artifact, shared production source or teammate branch
  modified for this version. V1 UF2 remains available.

Physical V2 turn remains pending. No local robot connected or actuated. Reflash,
h verify v2, valid1/side_valid1;2,k,h confirm mode2; same bank sequence and mat.
Do not recalibrate if mounting unchanged. Wait3s after stop, switch sources
separately, p. A restored V1 trace is historical; only fresh run prints V2.
