# Coding-cleanup candidate validation

Baseline: tested main `7b18596675e6198837851305500f6541b81774b6`, v0.2.6.
Candidate: `barr-c-cleanup`, v0.2.7. The generated release manifest pins the exact
source commit, compiler, SDK identity, profile and source/artifact hashes.

## Completed checks

| Check | Result and purpose |
| --- | --- |
| Formatting/braces stage | ARM rebuild produced the exact baseline UF2, SHA-256 `9e69291564ccccb3c70359cf331e5df3f884cce514b2972412105dccc9248208`. |
| Types/variables/declarations/module contracts | ARM rebuild again produced that exact UF2; existing alternate-configuration C regressions were also exercised. |
| Explicit expression grouping / constant-first equality | ARM UF2 matched its preceding representation stage byte-for-byte. |
| Final mechanical gate | 133 application C/header/include files; zero findings for the checker's documented subset. |
| Final formatting gate | clang-format 18.1.8 check passed over the same 133 files. |
| Compact signed ranges | Exhaustive 2/3/9/11/13-bit round trips, high-bit masking, 31-bit limits and invalid-width handling; actual projection/pitch/direction consumers checked. |
| Native layout | Seven frozen baseline size/alignment comparisons compile on host and Arm. Native layouts remain implementation details, not wire formats. |
| Final affected-feature C run | 20 production-C regression programs passed. They cover safety, pickup, controllers, mirrored bend/search, coordinator, barcode replay, runtime mapping, ramp bridge/service and exports. |
| Retained codec fixtures | Drive diagnostics/mode, C9 lift, CA peaks, network-stop and line-row bytes equal their preceding-stage fixtures. The C2 release patch intentionally changes from 6 to 7. |
| Dashboard decoding | Actual C-to-browser release/calibration/line/runtime decoding passed; 19 additional drive, pickup and diagnostic tests passed. |
| ARM release | `APP=rtos_buddy1_4 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0` builds successfully. |
| Memory / flash / stack | Static RAM end `0x2002fe48`, unchanged from v0.2.6; BSS 195584 bytes; 65976 bytes remain before dynamic kernel/task allocations. Text 503328 bytes, 72 bytes above baseline. Existing flash-RAM routine and per-frame stack limits pass. |

The final selected C run was:

```text
test_compact_signed test_bringup_policy test_imu_lift_guard test_imu_lift_packet
test_drive_diagnostics test_network_stop test_continuous_drive
test_continuous_steering test_continuous_bend_assist test_continuous_follow
test_continuous_integration test_line_search test_line_stationary_search
test_release_imu test_release_bridge test_release_exports test_rtos_barcode
test_rtos_ramp_bridge test_imu_service_rtos test_hump_odo_grace
```

Run these with `tools/rtos_migration/run_tests.py --only ... --skip-python`.
The earlier staged runs also covered existing barcode peak/reference, mapping,
protocol/settings, acquisition, powered hump and ramp-assist regression programs.
No unrelated Python capture-suite or full browser-layout rerun was needed:
capture tools are unchanged and the dashboard change is version identification.

## Limits and remaining hardware input

Software checks do not measure physical motor-stop latency, control-task
high-water stack use or wheel/mat behavior. The packet/layout checks preserve
representation, while a short physical smoke test remains necessary for the
explicit signed direction/pitch paths. Follow
[the candidate instructions](coding_cleanup_v0_2_7.md). Main is not promoted yet.

This is an adapted Barr-based cleanup, not a full unmodified Barr-C or MISRA
certificate. The automated gate covers a named mechanical subset. Per-function
documentation depth, casts/promotions throughout retained legacy helpers,
complexity/nesting and whole-program synchronization remain explicit manual
review obligations. Long acquisition/control pipelines were not arbitrarily
split just to reduce their physical line count; their ordering and tested
execution behavior take priority in this pass. Future changes must continue
the adopted standard and record specific deviations rather than assuming a
formatter proves semantic conformance.
