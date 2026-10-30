# Buddy4 powered-ramp acceptance v0.2

This release fixes two estimator rejection paths observed during the verified ramp-assist v0.2 runs. It has not been flashed or hardware validated. The successful crossings from the previous release did not produce accepted height measurements.

## Evidence and cause

- First powered run: the rejection event was detail 5 (`IMU_REJ_NO_ODOMETRY`), but the simultaneous source 2 snapshot had fresh odometry, zero invalid encoder counts, quality TURN, and a 200 ms averaged turn of2.4 degrees/s. `side_terrain` separately differentiated a single20 ms quantised heading increment against the20 degrees/s limit. A five-deci-degree increment exceeded the four-deci-degree per-tick allowance even while average yaw was low. The reason code was overloaded: it did not prove missing encoder data.
- Second run: detail 3 (`IMU_REJ_RANGE`) occurred with source 2 quality ACCEL and210 mm/s², with nearby intervals up to306mm/s². This generic quality path did not prove the13mm peak was outside the allowed height range.
- Native capture has source 2 interval summaries and periodic source 15 data, not every20 ms raw FIFO and odometry input. Tests reproduce these boundary mechanisms and a full synthetic processing profile; they are not an exact replay of the physical runs.

## Changes

1. Once available, use the existing200 ms turn rate against the unchanged20 degrees/s limit, instead of also imposing a quantised20 ms derivative. Keep the instantaneous check during warm-up. Sustained turns still reject.
2. Skip short acceleration-only disturbances using the existing maximum100 ms contiguous /25 percent total excluded-time budget. Excluded samples cannot update peak, prove climb persistence, or establish return to level. Prolonged or excessive exclusion still rejects. Live tilt remains invalid during the disturbance, so ramp assist retains its invalid-data cancellation.
3. Keep stale or unclean odometry, reversal, true turning, shock, gravity-norm failure, FIFO overrun, late sampling, timeout, insufficient travel and Stop rejection paths.
4. Source15 flags bit7 marks the new estimator. The Python decoder reports `estimator=2`; bits4–6 still report `detector=2`. Old images decode as estimator1. This is a separate estimator version, not another boost-detector change.

Motor control, target speed, feedforward assistance(+20percent/cap40), ramp-assist detector v0.2, calibration and MQTT commands are unchanged. SHA256 of main.c and continuous_drive.h match the prior v0.2 manifest. Only the telemetry marker changes the assist harness.

## Tests and build

- New focused suite:24 checks, 0 failures with ASan/UBSan. The archived pre-fix processing fails9 of the same24 checks.
- Includes full b4_tick synthetic10 mm acceptance with encoder heading stair-steps, plus rejection of a sustained real turn; state-machine checks exclude invalid peak values and preserve all failure/coverage guards.
- Existing side-ramp suite:45 passed,0 failed under ASan/UBSan.
- Existing Buddy4 service test passes calibration,10 mm synthetic ramp, unclean odometry rejection, reset recovery and64-frame dashboard decoding.
- Existing v2.3 suite:35 C executables pass. Existing ramp-assist tests pass360849 checks per runner/compiler, including sanitizer and decoder checks.
- Native Mac ARM Release build:0 warnings / 0 errors. UF2 blocks/magic/family/address validation passes.
- Records: `testing/integration/logs/2026-10-30_buddy4_powered_acceptance_v0_2_host_tests.txt` and `_build.txt`.

## Local firmware

`/Users/skyler/SIT/Y2T1/Embedded/bench_tools/buddy4/buddy123_interface_wifi_v2_3_b4_v0_2_ra_v0_2_active.uf2`

SHA256: `f0627657ceae6f28078b5cec2fcc06b1d3b0f8eef7cb9854bd90a82a86cebf50`

Contains local Wi-Fi credentials; never commit the UF2/ELF/config. Existing firmware images are preserved. Source manifest: `BUDDY4_POWERED_ACCEPTANCE_V0_2_MANIFEST.json`. Pre-fix processor/config/CMake/harness copies are in `archive/b4_acceptance_before_v0_2/`.

## Hardware continuation

1. Keep lower bracket, original sensor arrangement, divider and original calibration unchanged. Flash the exact new file with Mac USB only; unplug Mac USB before power bank.
2. Keep still10 seconds. One short flat-line run then Stop. Require fresh source 15 `detector=2`, `estimator=2`, no unwanted assistance, and no false accepted hump.
3. Repeat the same10 mm right-wheel ramp without assistance by hand. Stop once all wheels are flat, then wait at least one second for return-to-level acceptance. If physically stalled for3 seconds, Stop; do not wait for a height event while stalled.
4. Verify actual source 4 `hump peak`, accepted count increment, stored last/highest height. Physical crossing alone does not pass height estimation. Compare accepted height against measured10 mm and report error; do not tune to force10 mm or disable rejection safeguards.
5. If rejected, keep exact event and source 2 quality flags; do not interpret generic detail 3/5 as a proven root cause. The new firmware must not be reported as hardware fixed until tested.

No git commit, checkout, flash or motor command was performed.

## Hardware validation: first accepted powered 10 mm ramp
After manual Reset and confirmed fresh IMU Idle/valid/fault0, user reports complete unaided crossing. Fresh source15 frames confirm detector2/estimator2 (16frames). SDKrun2:353mm/8319ms, manualStop49, recovery0, encoderinvalid0/0, maxcontrolgap13124us. Ramp assist activated04:08:35.726 at5mm, endedonDESCENT after2376ms at1mm (peak10mm), peakaddedPWM36. Source4 humpstart04:08:35.741 and acceptedhumppeak04:08:38.743:height10mm,highest10mm,humps1,rejected0,shocks0,lastFault0. Finalmotors0/0,IMUIdle. This is one observed powered acceptance with estimated10mm matching user-measured10mm ramp; repeatability and broader accuracy remain pending.
Evidence: testing/integration/logs/2026-10-30_b4_estimator_v0_2_ramp10_after_reset_040810_review.json. Native capture preserved. No external sharing required.
Next: one unchanged ramp repeat, then Stop during assistance. Existing intermittent flat line-following issue remains separately open and was absent in this ramp run.

## Stop during active assistance: verified (04:14)
User pressed dashboardStop atcrest. Source15detector2/estimator2: activation04:14:35.460; ended04:14:36.806 reasonSTOPPED after1342ms, alpha0 and addedPWM0/0. First inactive motion frame04:14:36.813 has targets0/0 andPWM0/0; measuredwheel speeds settle0/0 later. ManualStopreason49 at213mm/5111ms, encoderinvalid0/0. Source4 partialprofile rejected detail7=STOPPED, humps0/rejected1, eventheight13mm. Expected aborted profile: nofalseacceptedheight. PASS for commanded-output cancellation; precisephysicalstoppinglatency notindependently measured.
Evidence: testing/integration/logs/2026-10-30_b4_estimator_v0_2_stop_assist_repeat_041347_review.json.

Current new-estimator powered evidence: completed10mm ramp accepted10mm and11mm,2/2unaided; active-assistStop verified; no encodererrors or linerecovery in these two completedramps. Intermittentflatline-following issue remains unresolved and reproduced on priorfirmware. Furtherheights/repeats/fullmission are not claimedvalidated. No firmwarechange required by these latesttests.
