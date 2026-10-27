# V1.1 HANDOVER1 — targeted cruise-exit fix with saved diagnostics

[Download the HANDOVER1 UF2](firmware/buddy23_interface_v1_1_handover_v1.uf2).

This separate test starts from the unchanged-control [V1.1 DIAG1](../interface_v1_1_diagnostics/README.md) at949d4b7. The first physical diagnostic run showed a cruise-exit reset clearing both learned PI integrals even during modest requested slowdowns. See [captured-run analysis](../interface_v1_1_diagnostics/DIAG1_RUN_ANALYSIS_2026_10_27.md). HANDOVER1 changes that specific state handover; it does not incorporate V1.2/V1.3 motor-controller experiments. Physical improvement remains unverified.

## Exact control change

When the old fast flag clears during ordinary visible smoothed following (decision24/25), profile1 or3, both targets>=20mm/s, both controllers already forward/non-turning, and neither measured wheel speed is more than10mm/s above its target or below-8mm/s, the handover retains the existing PI integral rather than clearing both. The original50mm/s reference cap is retained. Normal per-wheel speed feedback, negative-error integration, saturation, falling-target deceleration/reset, low-inner-wheel coasting and zero-target resets still run. No new PWM floor, compensation integrator, output slew, feedforward or gain is added.

External ownership, planned turns, post-turn capture, load assistance, line-loss recovery/search, STOP, profile2 ramp and substantial overspeed all use the original handover. The guard decides where the small state-transfer change is allowed; it does not increase a speed setting or change IR thresholds. The preservation flag marks selection of this path. A later wheel-controller check can still legitimately change/reset its integral on the same or a subsequent update.

The optical/calibration/decoder/junction algorithms, cruising and scanning targets, PI gains, PWM caps, breakaway helper and API/coordinator are otherwise the same as DIAG1. This is not a claim that every jerk has been fixed.

## What to do

1. Flash `buddy23_interface_v1_1_handover_v1.uf2`. Press `h`: check **BUDDY23 INTERFACE v1.1 HANDOVER1** and valid `CAL FRONT`. Existing DIAG1/V1.3 calibration imports. Keep the present calibration if mounting and mat are unchanged; no new center/barcode calibration.
2. Press `1`, then `k`. Run normally over a straight and bend using your usual startup. USB `g` gives2s countdown; `s` or either button stops. Power bank: wait5s, both buttons3s, release, wait1s, tapGP21, wait2s. No repeated fixed-speed probe is required.
3. Once you have seen the jerking, stop and **wait at least5s before disconnecting power**. The firmware saves diagnostics alongside its normal logs.
4. Connect PuTTY, start saving terminal output to a file, press **`p`**, wait for `END BARCODE CIRCUIT LOG`, then press **`d`**, wait for `DRIVE_DIAG_END: complete=1`. Send the complete text file. `v` can remain OFF. Do not start another run before dumping the one you want.

`p` remains the ordinary motion/barcode log. `d` is a separate diagnostic dump, limited to256 rows. `j`, `a` and `z` retain barcode events, raw barcode ADC and coordinator logs. Dump commands refuse to run while driving/settling. Offline power-bank captures restore after reboot with CRC validation; `stored=1 restored=1 matches_motion=1` confirms a restored matching record. On a same-boot dump, `restored=0` is normal. A calibration/profile change after the run can make match checking fail; retain the original outputs first.

## Window selection

At each approximately10ms control update, the recorder observes sensor/steering/motor state. It waits until2s into the run, then triggers on the first ordinary-follow physical PWM change>=80counts in one cycle, a steering differential reversal crossing+10 to-10mm/s (or opposite), or selection of the new preserved-cruise handover. `trigger=4` identifies that handover (bits1/2 remain PWM/reversal and may combine). These thresholds select a recording window ONLY; they do not alter driving or classify a line.

It keeps up to127 preceding rows, the trigger row and128 subsequent rows, then freezes the256-row window. At100Hz this is about2.56s, centered on the selected event. Control and normal logging continue. If you stop before the post-window completes, it saves the available partial window. If no trigger occurs, it retains the latest256 rows. The selected event is a candidate for diagnosis, not proof of a fault. The window may capture an earlier jerk rather than the final one you noticed; its US timestamps align to `p`'s ms clock.

Whole-run tick count, maximum interval, intervals>=20ms, maximum observed control cost and maximum packing time continue after freeze. Record capture does not print or write flash while moving. Flash saving occurs only after stop/settling. No claim of zero timing impact or measured Pico overhead is made before a physical run.

## Reading d

| Fields | Meaning |
|---|---|
| RAW_L/R | Existing averaged front ADC sample, before the existing35% temporal filter |
| ADC_L/R, LEVEL_L/R | Filtered ADC used by the follower and calibrated0..1000 levels |
| OPT_L/R, OPT_DECISION | Original optical wheel requests/decision before V12.2 smoothing |
| SOFT_L/R | Optical requests after smoothing/bypass, before coordinator arbitration |
| TARGET_L/R | Final requests forwarded to the motor controller |
| REF_BEFORE_L/R, REF_L/R | Motor magnitude references before preparation and after the step |
| SPEED_L/R, TICKS_L/R | Filtered measured wheel speed and raw signed encoder increment |
| I_BEFORE_L/R, I_L/R | Integral before preparation and after the motor step |
| FF_L/R | Derived feedforward base at the recorded final reference; not a measurement of torque or a guarantee that an early-return path applied it |
| REQUEST_L/R, PWM_L/R | Computed physical PWM request and the signed PWM hardware compare-register values observed after commit/safety gating |
| MEAN/HALF_BEFORE, MEAN/HALF | Common/differential smoothing state before/after; meaningful only with filter-active flags |
| PHASE, PROFILE, ASSIST, DRIVER, FLAGS | Which branch/helper/driver was used; dump includes the bit legend |
| US, DT, COST | Run-relative microseconds, update interval and observed control-body cost |

SPEED/REF/I/FF/MEAN/HALF are saved at0.5-unit precision. ADC/PWM/ticks are integers. A CLIPPED flag discloses out-of-range values. I/FF units are the legacy driver's native units: raw duty before1.2 scaling for ordinary/loaded DRIVER0, physical duty for maneuver DRIVER1 or ramp DRIVER2. These values must not be compared across driver modes without that conversion.

`max_pack_us` measures the row-push/packing section, excluding sensor/PI snapshot gathering and optical taps. `max_control_observed_us` measures the eligible control body plus diagnostic gathering/packing, excluding ADC work done before control and the final small timing-field write. They are not an exact separate benchmark of logging overhead. Use interval gaps and the runtime capture to assess whether diagnostics have disturbed scheduling.

## Storage and provenance

Diagnostic format2 adds bit262144=`cruise_I_preserved`; FLAGS now uses19bits. Old format1 d-captures are rejected, while the normal motion/calibration record still imports. Make a new HANDOVER1 run before dumping d. Each row packs459 meaningful bits into58 bytes (5padding bits), using per-field widths and explicit signed decoding. RAM capture14848 bytes plus recorder metadata, working observations and flags. The independent16KiB bank is at flash offset0x1ab000..0x1af000, directly before the coordinator bank0x1af000; raw barcode0x1b0000 and motion0x1d0000 are unchanged. Data is programmed before the CRC-protected header. An incomplete/corrupt bank is rejected.

The normal motion layout remains64848 bytes, record23, identifying this diagnostic build. Earlier calibration/records1..22 import. Older firmware cannot read a new23 record, so note CAL references and verify/recalibrate if downgrading. Public nav APIs and audit formats are unchanged. Existing separate API/primitive limits remain; no ultrasonic or IMU policy is added.

`V1_1_BASELINE.json` and `DIAG1_BASELINE.json` record the two provenance layers. `DIAG_PATCHES.json`/`HANDOVER_PATCHES.json` identify adaptations; original motor and optical decision headers remain byte-identical. `scan_smooth_reference.h` remains the exact V1.1 optical reference. The only added control policy is cruise_handover.h and its guarded call in the hardware loop. See BUILD.md for tests and artifact identity.
