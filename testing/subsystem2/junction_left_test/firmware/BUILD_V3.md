# Junction left V3 — 21 October 2026

Artifact buddy23_junction_left_v3.uf2:115200bytes,225blocks.

SHA256:

```
d1d67eb46ad9d59151b6292047706aba54e4cf9bfacf9b3456b7f9184022df5d
```

V2 physically stalled after3.13degrees with both outputs at250permille. V3 uses
phase4-only loaded feedforward270 and PWM ceiling300 (27/30%). All forward
phases keep250 and match the V11 controller. Left still reverses, right forward.
No changes to sensor calibration,92mm geometry, optical branch gates, speed
targets,25s/900mm whole-run,6s/110degree turn or500ms stalled-feedback cutoff.

The shared line_breakaway and dual_levels both clamp250. V3 explicitly preserves
larger turn PI output past the former and uses a local, gated30% bridge mapping
instead of the latter, ONLY when phase4 has left-negative/right-positive demand.
The final output commit still checks active flag/generation with IRQs masked.

Checks passed:

- Strict host C11 -Wall -Wextra -Werror -O2 sequence/rejection/guard suite.
- Stationary turn demand reaches300 in actual bridge levels after startup pulse
  duration; no silent250 clamp. Overspeed lowers output; zero clears PI/kick.
- Both wheels' signed turn outputs bounded300. Bridge sweep -1000..1000 validates
  direction/channel exclusion, turn<=300, forward<=250, same forward mapping.
- Forward PI/boost/output sequences exactly match V11 across changes and zeros.
- Existing calibration/record layout unchanged; V1/V2 records retain version
  labelling, new runs record3. Compile-time offsets and3792byte sector-fit retained.
- Pico W Release SDK2.2.0, Arm GCC14.2.1; application warnings treated as errors.
- ELF text61520/data0/BSS12184; firmware end0x1000e054 below log sector0x101ff000.
- UF2 family0xe48bff56, headers/block sequence/payload sizes/flash address ranges
  verified below the reserved sector. USBstdio present, UART init absent.

User video was inspected; no local hardware actuation. Physical V3 turn remains
pending. Reflash,h verify v3, valid1/side_valid1,2,k,h verify mode2. Same bank/mat
sequence. Retrieve fresh p log after stop/save and power-source switch. Existing
V1/V2 UF2s retained; V11 and shared production/teammate code untouched.
