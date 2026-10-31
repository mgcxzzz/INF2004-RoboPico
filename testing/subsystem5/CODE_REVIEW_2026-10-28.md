# Buddy 5 code + bench review — 2026-10-28

Read-only review of `buddy5` at `1a0b559`. HC-SR04+ (GP16/GP17) and SG90 (GP12) remain integrated. Bench evidence supports distance ranging, not safe bypass clearance or scan coverage.

## Prioritized findings

1. **[High] Coarse scan can miss a narrow obstacle between rays.** `task_obstacle.c:537-569` samples only -55°, -27.5°, 0°, +27.5°, +55°. Refinement starts only after a near sample (`obstacle_profile.c:221-274`). A 100 mm object subtends about 23° at 250 mm, less than the 27.5° gap, so an off-center obstacle may go undetected. Add overlapping/dense scan coverage; treat ambiguous gaps as unknown/blocked. Test offsets across the sweep. This affects safety coverage, apart from width metrology.

2. **[High] A missed echo does not break “consecutive” stop confirmation.** `st_confirm()` continues after one timeout (`task_obstacle.c:481-488`) without clearing `g_ctx.confirm`; below-threshold readings separated by misses can accumulate. Reset confirmation on invalid readings, or specify/test a bounded-window rule. Cover near/far/no-echo interleavings in task tests.

3. **[Medium] An absent/low-confidence profile resumes motion.** `task_obstacle.c:627-634` releases the stop when no obstacle is profiled or confidence is below 50%. With incomplete coverage or invalid samples, uncertainty can become “clear.” Track unknown separately; resume after a forward recheck or stay stopped/rescan. Test low-confidence and partial-coverage scenarios.

## Bench evidence and requirements

- 20 Oct range calibration: means 106/303/498 mm at 100/300/500 mm (errors +6/+3/-2; spread 0/4/9 mm); sensor-to-car-front offset 45 mm. This supports `SOUND_NUM=343`; nominal front clearance at the 300 mm stop threshold is 255 mm. `logs/2026-10-20_S5-C03_range_scale.txt`, `TEST_LOG.md`.
- Standalone width scan: 100 mm target read 299–319 mm; 230 mm read 231–237 mm. Alpha is not a universal fix; keep span/clearance conservative and validate 100–200 mm obstacles at multiple positions/ranges.
- Recorded U09 evidence is from a standalone tool and fails the narrow-box criterion. U08/U09 do not prove production scan, settling, or bypass performance. Host tests are software evidence only; powered-car coverage, clearance, and line reacquisition remain physical checks.

## Implemented after review

Dense settled repeated scans, consecutive-confirm reset, navigation-only conservative profiler, unknown/low-confidence stop policy with forward recheck, mission-stop dispatch guards and missing-pose rejection now address the findings. Added model allowances/evidence flags; see [STATUS.md](STATUS.md) for explicit scope and remaining hardware assumptions. New task scenarios5..8 exercise the actual modified task against mocks, not physical hardware. Original findings above refer to baseline1a0b559 line locations.

### Integration blocker found during final review

This branch contains real legacy Buddy3 line code, not only template files. `subsystem3/s3_ir.c` reads digital GPIOs; `s3_config.h` uses GP6/GP7/GP19. The user reports that every IR now uses analog. The strong `b5_line_detected()` from `task_line.c` therefore is not evidence of valid line reacquisition on the new wiring. Port/verify the teammate analog interface before physical bypass/recovery; do not treat the legacy host line tests as hardware validation. This review did not silently merge or rewrite Buddy3. Buddy2 remains a template and `b5_motion_get_estimate()` defaults false; the new pose guard blocks bypass when real odometry is absent.

Timing note: new dense scan has a coarse ping-only lower bound3.78s, exceeding the team report's full3s confirmation/reacquisition target. It is an explicit unresolved target conflict, not a passed end-to-end requirement.
