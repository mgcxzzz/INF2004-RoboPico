# Buddy 5 span and clearance update — 2026-10-28

## Branch and preserved work

This checkout is `/Users/skyler/SIT/Y2T1/Embedded/INF2004-RoboPico-buddy5`, branch `buddy5`, base `1a0b559`. It is a separate Git worktree because the original `/Users/skyler/SIT/Y2T1/Embedded/INF2004-RoboPico` still holds the unfinished Buddy4/Buddy1 merge and ramp/speed work. Do not switch/reset that original tree to access these changes. No commit/push or hardware flash was performed.

## Requirement interpretation and bench evidence

The supplied lecturer clarification keeps the HC-SR04 and servo and emphasizes an estimate of obstacle span and surrounding clearance for safe bypass of expected 100–200 mm obstacles, followed by line reacquisition. It permits using the maximum expected width with scan evidence. Supplied screenshots are saved under `references/`; the student's broad claim of hardware impossibility and any universal 30-degree cone are not treated as calibrated facts.

The prior standalone bench measured a 100 mm target as299–319 mm and a230 mm target as231–237 mm. A single angular subtraction cannot explain both. Range calibration at100/300/500 mm gave106/303/498 mm. Servo/placement variation was about±4 degrees. These data motivate conservative planning; they do not establish a statistical uncertainty interval, production scan accuracy or physical bypass safety.

## Implemented changes

- Production uses `b5_build_navigation_profile()`. The old corrected-width function remains only for the existing synthetic regression, whose95/95 result is not physical metrology evidence.
- Observed span is not shrunk by a guessed beam angle. Planning uses at least200 mm plus angular/range model allowances and retains larger observed extents. The200 mm prior is not a cap and does not imply every real obstacle is200 mm.
- `span_uncertainty_mm` is the two-edge model allowance derived from the configured4-degree servo allowance, sample spacing and10 mm range allowance. It is not a confidence interval or a guaranteed bound on physical width; the unknown target-dependent acoustic response remains uncalibrated. `planning_width_mm` and four evidence bits are available through the status getter and USB profile report. Source5 binary telemetry remains the existing layout and carries observed span; it does not yet carry these extra fields. Downstream consumers must not interpret the telemetry width as a precision measurement.
- Coarse scan is21 positions at5.5-degree spacing, within the measured±55-degree mechanical limits. Coarse and refinement angles use three pings after servo settling. Fewer than a majority of valid echoes, or a triplet spread over20 mm, marks an angle unknown. The servo40 ms settling value is retained as an unvalidated tuning input; compare40/80 ms on hardware rather than declaring either adequate.
- Consecutive supported angle detection has20 mm threshold hysteresis. Unknowns/gaps cannot establish side clearance. Continuous valid outward evidence must reach beyond the configured bypass corridor; the first-free-ray gap is checked too. A second return limits clearance. The car width plus two margins must fit; edge-clipped sides are unavailable. This is a conservative scan model, not proof that every point in a swept vehicle volume is sensed.
- A missing echo breaks consecutive stop confirmation. Low-confidence/ambiguous profiles stay stopped. An apparently empty sweep releases only after a fully valid sweep and repeated fresh forward clearance. Mission Stop before planning/bypass prevents new plan dispatch. An absent initial odometry estimate prevents bypass instead of seeding the pose from uninitialized stack data.
- The profile expands from7 to8 words (32 bytes), preserving minimum-width fields; sample size remains4 bytes, binary telemetry unchanged. Scan capacity48 accommodates21 coarse plus12 refinement points. Status is still copied under its existing mutex.

## Remaining assumptions and integration limits

The existing200 mm obstacle-depth assumption is independent of the lecturer's100–200 mm width range: verify depth/corridor dimensions before powered bypass. Car155x190 mm and margin50 mm are measured/configured inputs, not guarantees about braking/turning clearance. Motion/line/Safety adapters must be real and tested before physical bypass; this branch's host mocks and template results do not prove that integration.

Background-distance subtraction needs a known-empty reference from the SAME sensor pose and comparable scene; no such paired raw capture was supplied. It is intentionally not fabricated from the obstacle scan. Collect an empty scan then the object scan without moving the car and compare angle-matched medians offline first. A reversal/re-scan already exists only when the recently traversed path and real odometry support it; cross-pose scan fusion is not implemented or claimed.

## Next physical tests (not run)

1. Known100/150/200 mm targets at300 mm sensor-front distance: five repeated scans each. Save raw angle/median/validity, observed span, model allowance, planning width, left/right clearance, evidence flags and selected side. Compare repeatability and unknown/blocked rate; do not grade as exact physical width.
2. Repeat offset left/right and at250/350 mm; add a nearer side wall, a second obstacle, an interrupted scan/echo and an object at an arc limit. Unknown clearance must not cause a release or guessed bypass. Compare40/80 ms settling and record pulse/target mounting.
3. With same-pose empty/object paired captures, investigate background-distance contrast before enabling subtraction in production. Never reuse that reference after moving the car.
4. Only after real Motion Control/Safety/line integration: repeated low-speed bypass runs around100–200 mm obstacles. Record chosen side, contact/no contact, measured minimum body clearance against the50 mm design margin, whether the obstacle was fully passed and line reacquisition. Include blocked-route and stop cases. Safe selection and reacquisition, rather than matching a ruler width, are the application outcome.

Use `testing/subsystem5/logs/` and append result rows. User supplies screenshots only when needed for visual evidence; routine checks use text logs.

### Integration blocker found during final review

This branch contains real legacy Buddy3 line code, not only template files. `subsystem3/s3_ir.c` reads digital GPIOs; `s3_config.h` uses GP6/GP7/GP19. The user reports that every IR now uses analog. The strong `b5_line_detected()` from `task_line.c` therefore is not evidence of valid line reacquisition on the new wiring. Port/verify the teammate analog interface before physical bypass/recovery; do not treat the legacy host line tests as hardware validation. This review did not silently merge or rewrite Buddy3. Buddy2 remains a template and `b5_motion_get_estimate()` defaults false; the new pose guard blocks bypass when real odometry is absent.

### Timing limitation

The dense21-angle coarse sweep alone needs63 pings at>=60ms spacing: at least3.78s before servo motion/settling, refinement, bypass or line recovery. This mode therefore CANNOT satisfy the existing team report's NFR-USO-05 target of confirmation-to-line-reacquired<3s. The supplied official briefing/write-up text asks for reacquisition without that numerical target; the team's submitted design target still needs an explicit timing decision, not silent removal. Width clarification does not itself waive that target. Keep this mode as stopped-car conservative profiling preparation; measure actual scan/drive/recovery durations before choosing a final scan schedule or revising the internal target. No timing PASS is claimed.
