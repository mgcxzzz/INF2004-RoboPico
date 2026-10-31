# Evidence index

Historical evidence is copied byte-for-byte from the pinned commits below. It is not hardware acceptance of the newly restructured build. `current/` is reserved for this build’s own checks and later robot runs.

| File | Evidence | Why retained |
|---|---|---|
| [buddy2/hardware/v4_wheel_response_2026-10-21.txt](buddy2/hardware/v4_wheel_response_2026-10-21.txt) | historical hardware log | Early wheel response and asymmetry |
| [buddy2/hardware/v10_successful_run_2026-10-21.txt](buddy2/hardware/v10_successful_run_2026-10-21.txt) | historical hardware log | Historical successful line-following trace; predates Continuous V2 |
| [buddy2/barcode/a_v1_capture_events.csv](buddy2/barcode/a_v1_capture_events.csv) | historical hardware capture | Original barcode event captures; useful regression context, not current-build acceptance |
| [buddy2/barcode/a_v2_capture_events.csv](buddy2/barcode/a_v2_capture_events.csv) | historical hardware capture | Original barcode event captures; useful regression context, not current-build acceptance |
| [buddy2/v23_baseline/host_tests.txt](buddy2/v23_baseline/host_tests.txt) | software/build check | Original V2.3 validation reference |
| [buddy2/v23_baseline/lwip_timeouts.txt](buddy2/v23_baseline/lwip_timeouts.txt) | software/build check | Original V2.3 validation reference |
| [buddy2/v23_baseline/stack_layout.txt](buddy2/v23_baseline/stack_layout.txt) | software/build check | Original V2.3 validation reference |
| [buddy2/v23_baseline/arm_stack_usage.txt](buddy2/v23_baseline/arm_stack_usage.txt) | software/build check | Original V2.3 validation reference |
| [buddy2/network_history/timeout_pool_panic_2026-10-28.txt](buddy2/network_history/timeout_pool_panic_2026-10-28.txt) | historical diagnosis/regression | Documents why the timer-pool fix is retained |
| [buddy2/network_history/hotspot_stall_2026-10-28.txt](buddy2/network_history/hotspot_stall_2026-10-28.txt) | historical diagnosis/regression | Documents why the timer-pool fix is retained |
| [buddy2/network_history/lwip_timeout_pool_regression.txt](buddy2/network_history/lwip_timeout_pool_regression.txt) | historical diagnosis/regression | Documents why the timer-pool fix is retained |
| [buddy4/standalone/2026-10-20_host_tests.txt](buddy4/standalone/2026-10-20_host_tests.txt) | software test | Original standalone IMU validation; not combined firmware |
| [buddy4/standalone/TEST_LOG.md](buddy4/standalone/TEST_LOG.md) | test ledger | Buddy4 branch test history |
| [combined_baseline/2026-10-31_combined_v0_5_speed_hold_runs_native_excerpt.txt](combined_baseline/2026-10-31_combined_v0_5_speed_hold_runs_native_excerpt.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_combined_v0_4_tilt_floor_runs_native_excerpt.txt](combined_baseline/2026-10-31_combined_v0_4_tilt_floor_runs_native_excerpt.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_front_calibration_readback.txt](combined_baseline/2026-10-31_front_calibration_readback.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_tilt_front_flat_repeat_review.json](combined_baseline/2026-10-31_tilt_front_flat_repeat_review.json) | analysis/replay record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_tilt_front_flat_usb_dump.txt](combined_baseline/2026-10-31_tilt_front_flat_usb_dump.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_tilt_front_white_applied_ram.txt](combined_baseline/2026-10-31_tilt_front_white_applied_ram.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_tilt_front_left_black_applied_ram.txt](combined_baseline/2026-10-31_tilt_front_left_black_applied_ram.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-31_tilt_front_right_black_applied_ram.txt](combined_baseline/2026-10-31_tilt_front_right_black_applied_ram.txt) | hardware record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/2026-10-30_barcode_peak_replay.txt](combined_baseline/2026-10-30_barcode_peak_replay.txt) | analysis/replay record | Working combined baseline, preceding failure, calibration or barcode-replay evidence; predates restructuring |
| [combined_baseline/TEST_RESULTS.md](combined_baseline/TEST_RESULTS.md) | test ledger | Context and limitations for the teammate hardware records |

[PROVENANCE.json](PROVENANCE.json) records original branch, commit, path and SHA-256 for each file. Firmware comparisons must use the recorded build and calibration. Earlier failures are retained deliberately to explain the fixes.
