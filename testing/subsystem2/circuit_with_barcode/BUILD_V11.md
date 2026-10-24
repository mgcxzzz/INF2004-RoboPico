# Circuit with barcode V11: queued Z junction U-turn

Built 2026-10-25 for Pico W/RP2040, Pico SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, C11 with -Wall -Wextra -Werror.

- Artifact: firmware/buddy23_circuit_with_barcode_v11.uf2,229888 bytes,
  449 sequential RP2040 UF2 blocks.
- SHA256: c01b659635489fa5e503c4317f3ecb23fcb781bd7e4af651f883d3b02dfc70c7
- Padded image end0x1001c100; linker end0x1001c05c.
- ELF text118872/BSS215344; BSS end0x20035c70, stack limit0x20040000;
  gap between symbols41872 bytes. All reserved flash banks remain separate.
- Main saved record64848 bytes appends decoded_z/uturns after the unchanged
  64840-byte V1..V10 prefix. Trace56/summary40 offsets remain unchanged.
  Version-aware CRC loads historical records/calibration; unknown12 rejects.
- UF2 family, sequence, payload, addresses, trailer, markers and package/build
  equality pass verification. Published V3-V10 artifact hashes are unchanged.

## Resulting behaviour

Generic complete Code39 decoding and two-adjacent-threshold acceptance remain.
The route mapping now includes Z/U alongside A/L and F/R in both digital
legacy navigation and the current analogue reader. Z waits for the same
separate, qualified later junction and is consumed once on dispatch.

Z uses right-wheel-forward/left-off to90 degrees, stop/settle, then
left-wheel-reverse/right-off toward180, stop/settle, then outgoing following.
It ignores the90-degree branch and requires prior line departure plus a
sustained front return within165..195 degrees. Completion still requires
150ms visible outgoing-line following, with the existing6s alignment window.
Per-quarter/whole-turn/settle limits and stationary-wheel drift bounds apply.
Signed PWM mapping and direction checking are enabled for the Z turn only;
the independent ISR uses the same signed42%/bounded48% envelope.

Distance now accumulates absolute32-bit encoder deltas in integer counters;
the cutoff ISR adds pending deltas without floating-point work. Signed wheel
positions remain the source for yaw/bar widths. Subsequent cross manoeuvres
translate their signed edge position into the cumulative path coordinate.
Legacy restored summaries retain historical distance interpretation.

Profile1 preserves V9 steering and wheel behaviour outside the Z manoeuvre.
A/F turns and all ordinary smoothing/search/header implementations remain.
Profile2's V10 ramp experiment is retained but not needed for this task.

## Executed tests

- test_uturn: complete analogue *Z* at141/282mm widths in both scan directions;
  queue/next-cross dispatch; front sensing geometry in a binary18mm cross;
  ignore a visible90-degree arm; settled reverse quarter; return-line optics;
 150ms confirmation and normal forward following; one completed U event;
  next A junction after reverse travel; missing return/time/drift/settle faults;
  negative-left PWM pin mapping; absolute tick count including INT32_MIN;
  V1..V11 CRC prefixes and new counter corruption rejection.
- Ideal no-slip kinematic fixture uses110mm wheel track,127mm front-sensor
  advance,92mm side advance and +/-5mm front sensing offsets. At acquisition
  completion it reports axle x45.6mm/y0.5mm relative to cross centre and177.7
  degrees of yaw. This is test geometry, not a physical measurement or a
  guarantee under scraping/slip. Continuing line following owns final centring.
- test_barcode: historical PDF/A replays, generic decoding/rejection/queue,
 12 alternating A/F turns per profile through the V9 smoothing adapter,
  saved-record legacy compatibility.
- test_analog/test_barcode_sensitivity: full Z now queues U; A/F remain;
  unsupported Q preserves the previous supported command; missing bars and
  conflicting votes reject; analogue Z and low-amplitude fixtures both ways.
- test_smooth_follow, test_ramp_profile, test_line_search, test_start,
  test_front_calibration and test_barcode_raw pass unchanged-purpose coverage.
  Six raw-dump Python parser tests pass.
- Pico W compilation and UF2 verification pass. Optional SDK WiFi/BLE modules
  remain unused by this standalone USB firmware.

First physical test: select1/k, scan a real Z before a cross, leave clear space
for the forward/reverse rolling manoeuvre and send j. The existing92mm side
geometry remains provisional if the mount has moved. No physical Z/U success
has been claimed yet. The user confirmed V9 smoothing read A/F successfully.
