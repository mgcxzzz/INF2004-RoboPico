# Junction left V6 — 22 October 2026

Artifact: `buddy23_junction_left_v6.uf2`, 123904 bytes, 242 UF2 blocks.

SHA256:

```
a515673f250f0f9b79031980095b171ed05b484b63e09ea7ece87b33003493ed
```

## Reason for this revision

Second V5 rolling test reached the outgoing stripe with left level721/right3
near92.01deg of turn-relative rotation. It kept pivoting because both>=650 never
qualified together, stopping at110.01deg with `left branch not confirmed`.
The video shows outgoing track available on the mat. Right wheel was still
moving at cutoff: this was optical handover failure, not a request for more PWM.
Full physical evidence and first V5 short-mat result are in MEASUREMENTS.md.

## Mode3 change

- Preserve cross detection,92mm side offset,55mm early rolling turn-start,
  left OFF/right45 then30mm/s pivot and bounded35% sustained/40% start effort.
- After leaving old line, either normalized level>=400 for30ms at75..<110deg
  starts NEW phase9. Turn deadline9s cannot be bypassed by a late candidate.
- Phase9 uses both motors forward at20..35mm/s, steering from normalized sensor
  difference and V11 direction memory. Straight30/30; strongest difference20/35
  mirrored as needed. Uses normal forward PI,25% output cap, no turn feedforward
  or40% pulse. Inner wheel is deliberately no longer fixed/off.
- Both>=400 and absolute difference<=350 for80ms flows directly to phase6,
  the V11 line follower for20mm, then phase7/test complete. No settle stop in
  mode3 handover. Mode2 retains its original both>=650/settle sequence.
- Acquisition bounds3s/60mm average absolute wheel path/60..115deg relative turn.
  Bothwhite<150 allows at most150ms using recent clear direction; unknown direction
  stops immediately. Whole25s/900mm,500ms feedback stall, wrong-direction/invalid,
  button/USB/control-late/deadline kills remain. No higher PWM or supply change.

## Verification

- Strict host C11 `-Wall -Wextra -Werror -O2`: pass. Covers either sensor first,
  the supplied721/3 observation, stable forward alignment/exit, transient/early
  angle/original-line rejection, missing branch, loss/time/path/angle bounds and
  late candidate rejection. Prior centre-only/in-place/rolling geometry and
  PWM pulse/expiry/no-retrigger/forward-equivalence tests pass.
- Host record probe: V4 padding normalization, CRC/front+side calibration, V5
  rolling record import, newV6/phase9 round trip. No new Junction fields: original
 3792-byte log/row192/36-byte trace ABI retained, also Arm static assertions.
  Acquisition temporarily reuses exit_origin for absolute path, then replaces it
  with mean wheel distance at phase6 entry. Old traces retain original labels.
- Pico W Release, SDK2.2.0, Arm GCC14.2.1. Application warnings treated as errors.
  ELF text65912/data0/BSS12232; firmware end0x1000f17c below reserved0x101ff000.
  USBstdio symbol present; UARTstdio init absent. No local hardware actuation.
- UF2 magic/end markers,256-byte payloads, block sequence/count, RP2040 family
 0xe48bff56 and flash ranges verified. Padded image end0x1000f200, below log sector.
- V11 shared controller files and the unrelated digital test folder unchanged.

Physical V6 test pending. These software checks do not establish successful
line capture or physical wheel speed under load.

## Next user test

Flash V6; h must show V6/valid1/side_valid1. V5 endpoints import without recapture
when mounting and height are unchanged. Select3,k,h confirm mode3. Repeat the
same approach with a flat white surface around the branch and enough outgoing
black (nominal>=220mm; prefer300mm for extra alignment room). Existing bank
sequence or USB hands-free g. Expected phases1,2,4,9,6,7 and STOP test complete.
Wait3s, switch sources separately, p; send fresh V6/test3 log and video. New run
replaces previous record. Older firmware can reject a V6 saved record on downgrade.
