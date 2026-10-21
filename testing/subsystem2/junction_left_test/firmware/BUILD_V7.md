# Junction left V7 - 22 October 2026

Artifact `buddy23_junction_left_v7.uf2`:123904 bytes,242 UF2 blocks.

SHA256:

```
732afd0517dc8fa0ab0e191db4e5a943196b350d94a6d3636d952b4b4945ceed
```

User reports successful V6 handover and requests a longer outgoing follow.
No fresh V6 log accompanied that report. V7 keeps V6 acquisition and completes
phase6 after5,000,000us from phase6 entry, replacing the20mm completion and3s
exit timeout. Mode1 still centre/stop; both modes2/3 use the timed exit. Its five
seconds start after alignment succeeds; approach/turn/alignment time is excluded.

The whole-run deadline becomes30s, leaving more time for the exit. The100-row,
400ms trace still fits its compile-time budget.900mm overall absolute-path abort,
V11 line-loss handling,500ms feedback stall, buttons/serial/USB/control-late and
encoder guards remain. Same35% sustained/one40% bounded turn pulse,25% forward
cap, targets and pins. Saved3792-byte/row192 record layout unchanged; accepts
V1..V6 records/front+side calibration, new traces labelledV7. The generic phase6
label is 'follow exit' so restored V6 logs are not described as five-second runs.

Verification:

- Host strict C11 Wall/Wextra/Werror/O2 suite passed. New assertions show
  continued drive beyond20mm, active at4,999,999us, outputs zero/completion at
 5,000,000us, and early line-loss/900mm abort. Existing V6 gradual handover,
  centre-only/in-place/rolling sequence and controller/pulse/expiry guards pass.
- Pico W Release, SDK2.2.0, Arm GCC14.2.1; application warnings treated as errors.
  ELF text65816/data0/BSS12232. End0x1000f11c below reserved log sector0x101ff000.
  USBstdio init present, UARTstdio init absent. Static flash/trace ABI checks pass.
- Verified UF2 magic/end markers, sequence/count,256-byte payloads,
  RP2040 family0xe48bff56 and flash ranges. Padded image end0x1000f200.
- No hardware actuation on the build host. Physical V7 five-second test pending.

Flash V7,h verify valid1/side_valid1,3,k,h confirm mode3. Same start sequence.
Allow about250mm of outgoing line beyond the sensor reacquisition point
(roughly500mm total branch from cross centre gives turn/alignment room).
Expect1,2,4,9,6,7 and STOP test complete. It can stop earlier on loss/guard
conditions. Wait3s after stop/save before changing power sources, then p.
Existing endpoints need no recapture when mounting/height remain unchanged.
