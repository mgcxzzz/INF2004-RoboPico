# Integration v3 — RTOS stability diagnostic, 20 October 2026

Status: built and software-verified; hardware stability/stop timing NOT validated.
V2 USB enumeration succeeded but its IR stream froze later. Separate SDK probe
streamed continuously. Both front sensors are now calibrated black1/white0.

Changes to copied build only:
- Single-core idle/tmp allocation256 ->2048 bytes; USB task2048 ->4096.
- GPIO initial attach uses priority3; USB shim sets priority3 before enable.
- Timer cutoff priority0, raw TA_ASM handler; no kernel or FP calls.
- Same original straddling S3/real nav tasks, PI output, 25% cap,150mm/5s limits.
- seq, microsecond-clock-derived ms, loop/task/IRQ counters and stack high-water
  in diagnostics. V1 return-to-shutdown bug remains fixed.

Reasoning: the private idle context uses the configured temporary-stack size,
only256 bytes in the old single-core profile. Compiler stack reports include
64 bytes in dcd_rp2040_irq alone, plus wrapper/callee/hardware and potential
nested-handler frames. This identifies limited margin, NOT a measured overflow
or a proven root cause. New idle_stack telemetry will provide physical evidence.
Kernel-aware handlers no longer nest one another. Raw cutoff does not touch
kernel bookkeeping; application shared updates remain short IRQ-masked sections.

Checks:
- Full Arm GCC14.2.1 build with application -Wall -Wextra -Werror: PASS.
- prepare.py lifecycle/IRQ config assertions: PASS.
- S3 logic58, S3 task55, nav72, safety36 host checks: zero failed.
- New host steering/target/stale/log checks: PASS.
- verify_build.py: strong real mc/nav/S3 interfaces; idle/tmp2048-byte symbols;
  cutoff calls only sys_time_us and stop_locked; no kernel/FP calls; SRAM flash
  writer; unchanged upstream nav/S3 sources; matching adapter; UF2 bounds PASS.
- ELF text75636, data0, BSS29764. SDK2.2.0 TinyUSB on real single-core uT-Kernel.
- RWX linker warning remains from executable SRAM flash writer (as documented).
- First hardware test is60 seconds motor-OFF diagnostics plus h response.

SHA256 buddy23_line_motion_v3.uf2:

```
95ca69e0d93b3a8981f0543d5376aa95148e8076ab2c118a28a4b7acd0d7d0bd
```

Rebuild via prepare.py and documented GNU Make single-core profile. Optional
-fstack-usage instrumentation was used to inspect stack frames; it does not
change the intended execution. Debug/motion results must be confirmed physically.
