# Integration v2 — startup lifecycle fix, 20 October 2026

Failure evidence: user saw COM port with standalone V6, but no COM port with
integration V1 using the same Pico/computer setup. No enumeration success for
V1 is claimed. The unrelated Topping DFU entry remained when Pico unplugged.

Concrete source bug: kernel/inittask/inittask.c calls shutdown_system(fin)
after usermain returns. V1 returned 0; shutdown disables interrupts. V2 sleeps
forever in the initial task, on success AND failure, so it never requests that
shutdown and USB remains scheduled. Hardware confirmation is still pending.

- Same pinned Buddy3 modules and S2 helper sources as V1 BUILD.md.
- No change to S3 line logic, sensor polarity, motor limits or saved format.
- App compile -Wall -Wextra -Werror, full link and UF2 conversion: PASS.
- prepare.py regression gate checks the real kernel return-to-shutdown path
  and the two non-returning adapter paths: PASS.
- Prior module tests: 58 S3 logic, 55 S3 task, 72 Nav, 36 safety, zero failed.
- ELF text 75,344; data 0; BSS26,168. SRAM executable flash section retained.
- RP2040 UF2; next physical step is USB enumeration and motor-off IR diagnostics.

SHA256 buddy23_line_motion_v2.uf2:

```
39a63e0d3f09b7079e8e3a53622b3ea374e60a930d12587fb06e99e1426aac18
```

V1's successful compile/host tests did not validate its hardware startup.
Retained V1 binary is historical, not the recommended image.
