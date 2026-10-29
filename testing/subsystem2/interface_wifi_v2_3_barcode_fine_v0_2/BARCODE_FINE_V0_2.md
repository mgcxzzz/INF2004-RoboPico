# Barcode fine v0.2 — diagnostics repair

v0.1 physical Z test reported no letter. Its diagnostics were suppressed by a producer/packer contract bug: the harness set reserved flag bit7, and the real packer rejects that flag. The previous host tests did not exercise the actual periodic producer and missed this. This bug explains missing source14 diagnostics, not the failed barcode decode.

## Changes from v0.1

- Remove the reserved diagnostic flag. Continue enforcing all reserved-bit guards.
- Allow the actual new winning bank63 through the C packer and both the isolated candidate and current merged dashboard decoders. Reject bank64 and above. Original63 banks and layouts remain unchanged. The old reference decoder remains untouched; differential tests document only this bounded extension.
- Emit informational build tag0xbc02 (48130) in the periodic source11 STATUS detail. Other coordinator events, fault data and command results retain their existing detail. USB help says BARCODE FINE v0.2.
- Add real harness -> packer regression for preview bank11 and fine bank63, then decode these exact emitted packets with both JS decoders. No synthetic-only reporting test.
- Detection algorithms, calibration, record30, controls, encoders, motor outputs/limits and networking are unchanged from v0.1. v0.1 and all older UF2s are retained separately.

## Validation and limitations

35 C test executables and existing JS/Python suites pass. Merged decoder checks20/20 pass. Captured Z still decodes once and captured Y remains unresolved under ASan/UBSan. Repaired real periodic packets decode correctly. ARM Release build has0 warnings; text511416 and BSS199588 bytes. UF2 RP2040 format validated. This reporting repair is not a claimed fix for the physical failed scan or stuttering.

## Artifact

/Users/skyler/SIT/Y2T1/Embedded/bench_tools/buddy2/buddy123_interface_wifi_v2_3_barcode_fine_v0_2.uf2

SHA256 `b11ca752b2ddad4660e7733e80ecec01a389dd76c78a35c2fd1508dfee7c31db`. Local credential-bearing firmware; do not commit. No flashing or motor commands performed by agent.

## Next hardware step

1. Flash this exact version on Mac USB, then disconnect Mac and use power bank only.
2. Reload9072 once (export existing session first only if wanted), connect and enable telemetry. Do not recalibrate or change bracket/sensor/mapping for this comparison.
3. Run over Z once in the same orientation, then Stop. Report just whether Z decoded and whether motion stuttered. Agent checks fresh source11 build tag0xbc02 and current source14 counters from native capture before interpreting the result. If it fails, preserve/read raw data from this run before repeating or changing code.
