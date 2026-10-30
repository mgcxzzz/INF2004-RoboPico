# Buddy4 add-on v0.1 on interface_wifi_v2_3 — change record

Build name: `buddy123_interface_wifi_v2_3_b4_v0_1` (ELF/BIN/UF2).
Baseline: `testing/subsystem2/interface_wifi_v2_3` at commit **8d7f3aa4935565636bba013c74e563f5e8135bcb**
("Add SDK WiFi V2.3 from working V2.2 with successful calibration preset"), checked out
detached in the separate worktree `INF2004-RoboPico-v23b4`. The RTOS checkout
(`INF2004-RoboPico`, branch buddy4, unfinished merge) was not touched.

## Intent

v2.3 keeps driving the car exactly as before. Buddy4 is added as an **observer**:
it reads the GY-511 and v2.3's own odometry, estimates ramps, and publishes telemetry.
It never writes motor, encoder, coordinator, calibration, startup or network state,
and none of its measurement-quality checks (MAD guard, odometry cleanliness, stale
FIFO, faults) can stop or change motion.

## Baseline files changed (3 of 185 in SOURCE_SHA256.json; 182 identical, 0 missing)

Full diff: `BUDDY4_ADDON_V0_1.patch`.

| File | Baseline sha256 (= SOURCE_SHA256.json) | Change |
|---|---|---|
| `v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c` | ab81c1b4… | +4 lines, nothing removed |
| `wifi_harness.h` | 66e3006c… | +1 prototype line; 1 line extended |
| `CMakeLists.txt` | 629e45fe… | +4 source lines; output name |

`main.c`:
1. After `#include ".../wifi_harness.h"`: `#include ".../buddy4_harness.h"`.
2. In `control_step`, after the velocity EMA update and before `if(!active) return;`:
   `buddy4_odometry_note(e,now,elapsed);` — reads the locally copied, already drained
   encoder snapshot `e[]` (delta ticks + invalid counters). It does not touch `enc[]`,
   `total_ticks`, `velocity` or the guard. It runs every control period, active or not.
3. In `main()` after `wifi_harness_init();`: `buddy4_init();`
4. In the superloop after `wifi_harness_poll(now);`: `buddy4_poll(time_us_64());`

`wifi_harness.h`:
- Prototype `static void buddy4_reset_request(void);` after the `start_block` prototype.
- In the accepted `WP_CMD_RESET` path, `buddy4_reset_request();` before `return 0;`.
  It only sets a flag; the v2.3 reset logic is unchanged and runs first.

`CMakeLists.txt`:
- Sources added: `buddy4/buddy4_service.c`, `buddy4/subsystem4/v23b4_drv_lsm303.c`,
  `buddy4/subsystem4/imu_proc.c`, `buddy4/subsystem4/b4_pack.c`,
  `buddy4/subsystem2/b2_pack.c`, `buddy4/system/sys_tlm.c`.
- `OUTPUT_NAME` `buddy123_interface_wifi_v2_3` → `buddy123_interface_wifi_v2_3_b4_v0_1`.
- Libraries, compile definitions and `-Wall -Wextra -Werror` unchanged.

## Files added

Unchanged copies of the validated Buddy4 v0.9 sources (12 files, hashes in
`buddy4/BUDDY4_UNCHANGED_SHA256.json`): `subsystem4/imu_proc.c/.h`, `imu_iface.h`,
`b4_config.h`, `b4_side_filter.h`, `b4_side_ramp.h`, `b4_diagnostics.h`, `b4_pack.c`,
`drv_lsm303.h`; `subsystem2/b2_pack.c`; `system/sys_tlm.h/.c`.
All calibrated constants (axes, 40 mg lateral/up MAD guard, track 113 mm, thresholds)
are the v0.9 values; none changed.

New glue:

| File | Role |
|---|---|
| `buddy4/v23b4_rp2040_shim.h` | `in_w/out_w/set_w/clr_w`, `sys_time_us()=time_us_32()`, and the RP2040 RESETS/IO_BANK0/PADS register addresses the driver used from the RTOS headers. |
| `buddy4/subsystem4/v23b4_drv_lsm303.c` | `drv_lsm303.c` byte-identical except the include block (shim instead of kernel/sys_hal). Bit-banged/register I2C1 timing assumes clk_sys 125 MHz. |
| `buddy4/buddy4_v23.h`, `buddy4/buddy4_service.c` | Port of `task_imu.c`'s loop body without the RTOS: 20 ms fixed-rate schedule, mission mapping, FIFO read, odometry adapter (same semantics as `b4_odometry_adapter.c`), `b4_tick`, diagnostics, telemetry (status/snapshot every 500 ms or on state change). |
| `buddy4_harness.h` | Included into the v2.3 main TU; reads v2.3 state, schedules the service, builds the odometry view, sends frames. |
| `buddy4/host/*` | Host test (fake FIFO, host clock), runner, frame decoder. |

## Adapted interfaces (only these)

**Hardware / scheduling.** GY-511 on I2C1 GP2/GP3 (free in v2.3), same register setup,
400 Hz ODR, FIFO. One Buddy4 period (20 ms) runs on core 0's superloop. It waits until
≤ 700 µs after a 2 ms barcode-ADC sample so the ~2 ms I2C read falls in the sampling
gap, or runs anyway once 2 ms late. Core 1 (CYW43/lwIP/MQTT) is untouched.
`buddy4_init` disables the add-on (message on USB) if clk_sys ≠ 125 MHz.

**Odometry (v2.3's, consumed read-only).** Delta ticks from each `control_step` drain
are summed into Buddy4's own totals (independent of v2.3's per-run `total_ticks`).
Distance = mean of `wheel_mm()` per wheel (v2.3's calibrated conversion); heading
= (L − R)/`TRACK_MM` in 0.1°; speeds = v2.3 `velocity[]` (the EMA it already computes).
An interval is **clean** only if 0 < elapsed ≤ 50 ms and neither wheel's invalid
counter changed. Unclean or stale (> 50 ms) intervals are given to Buddy4 as
"no motion data", so samples are not used for ramp estimation and the source-2
ODO_VALID flag is cleared. This is Buddy4's quality rule only; v2.3's own encoder
policy (tolerate up to 8 invalid per wheel per run, late > 100 ms stops) is unchanged.

**Run state → Buddy4 mission.** Running = `guard.active && session_active && mode ==
MOTION_AUTO`. Rising edge → START (new `guard.generation`) or RESUME (same generation:
hump/odometry counters kept). Falling edge → STOP. Accepted v2.3 MQTT Reset → RESET
(clears an IMU fault, re-initialises the sensor, recalibrates; keep the car still 2 s).

**Telemetry.** Frames are unchanged from the RTOS build: source 4 (IMU, 16 B payload)
and source 2 (37 B, includes per-axis MAD), published on v2.3's telemetry topic through
`wifi_send` when v2.3 telemetry forwarding is on (default on). Both fit v2.3's 64 B
packet limit. The v2.3 dashboard ignores these sources; read them with the RTOS
decoder (`tools/dashboard/is28_schema.js`) or the capture review tools.

**Removed coupling (RTOS only).** No Safety supervisor or `sys_fault` in v2.3: IMU
faults and shocks are reported in telemetry only and never stop the car.
Magnetometer is not used (as v0.9).

## Verification so far

- Host test `buddy4/host/run_host_test.sh` (gcc -Wall -Wextra -Werror):
  - real `imu_proc` through the service;
  - calibrates;
  - START/STOP/RESUME/RESET mapping;
  - 10 mm side ramp accepted as 10 mm;
  - unclean odometry gated (no hump, ODO_VALID clear);
  - sensor read-failure fault, then Reset recovers;
  - 64 frames all unpack in C and decode with the shipped JS decoder (0 errors).

  Log: `BUDDY4_HOST_TEST_v0_1.txt`.
- ARM compile check with the pinned SDK 98a542c and dummy credentials:
  - Baseline and add-on both build with exit 0 and 0 warnings.
  - Size: +16.0 KB flash, +1.2 KB RAM.
  - Stack frames are well inside the 8 KB core-0 stack.

  Log: `BUDDY4_COMPILE_CHECK_v0_1.txt`. The release UF2 is built locally on the Mac.
- Not yet verified on hardware: I2C timing under v2.3's load, the effect on barcode
  sample gaps (v2.3 `ANALOG max_gap`) and on control-period jitter, and IMU behaviour
  while v2.3 drives.

## Known risks to check on the car

- The ~2 ms blocking FIFO read on core 0 can coincide with a 10 ms control step or the
  front-ADC block (~1.3 ms) and delay it by up to ~2 ms. Compare v2.3's diagnostic gaps
  with the baseline v2.3 UF2.
- Odometry cleanliness uses v2.3's invalid counters; if v2.3 resets them at run start,
  the first interval of a run is marked unclean (conservative).
