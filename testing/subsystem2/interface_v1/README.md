# Buddy 2/3 Interface V1.3 — forward-effort test

[Download V1.3 test UF2](firmware/buddy23_interface_v1_3.uf2). [Controller changes and test procedure](FORWARD_EFFORT_V1_3.md). [Buddy 5 integration instructions](BUDDY5_HANDOFF.md). [Build verification](BUILD.md).

V1.3 is a **physical-test candidate**, not yet a validated smoothness improvement. It starts from V1.1 (`72e11ce`), which preserves the V12.2 optical provider and fixes the V1 startup clock bug. V1.2 performed worse physically; its extra drive trim, slow power reduction and cruise-qualification layer are not active here. Its artifact/source history is retained for diagnosis: [regression evidence](V1_2_PHYSICAL_REGRESSION.md).

The ordinary forward driver now uses one PI through ordinary tracking and qualified sharp-correction load assistance. Assistance opens the existing PWM ceiling instead of switching to pivot feedforward and clearing PI. Positive effort is limited to 1800 PWM counts/s; reductions are immediate. One bounded starting aid is allowed per positive-target episode, followed by the same speed controller; no independent compensation integrator. Measured overspeed releases retained positive I, and a large overspeed or deliberately slow inner wheel can coast immediately. This changes normal forward actuation in profiles 1/3. Planned turns, capture, search, ramp and external API commands retain their existing drivers. See the detailed scope and limitations in the controller note.

Optical targets, calibration, barcode thresholds, A/F/Z handling, junction geometry, search rules and speed qualification are V1.1/V12.2. No new centre threshold, brightness assumption or heading lock is introduced. Profile 1 permits up to60mm/s, profile3 up to135mm/s when the original optical qualifier passes; scanning remains45mm/s. Profile2 is still the paused180mm/s ramp experiment. A higher profile maximum does not guarantee a higher average speed; the existing qualifier can still limit it. Use profile1 first.

The default UF2 runs the autonomous circuit harness without ultrasonic code. Buddy5 adds his module into this same executable, using `nav_api.h`; UF2 files do not communicate at runtime. All Buddy3 wheel demands, Buddy4 terrain reports and Buddy5 avoidance requests pass through the shared coordinator and one generation-checked motor writer. Public interface signatures, ownership and cancellation behavior are unchanged.

## Run the test

1. Flash V1.3. Press `h` and check **BUDDY23 INTERFACE v1.3** and valid `CAL FRONT`. Existing valid calibration imports, including V1.2 record20. Recalibrate only if mounting/material changed or references are invalid: `w` both white, `l` left broad black, `r` right broad black, `k` save. Barcode A0 GP27 remains automatic.
2. Select `1`. USB/PuTTY: `g` starts after2s; `s` or either board button stops. `f` requires GP21 held. Power bank: wait5s, both buttons3s, release, wait1s, tap GP21, allow2s countdown.
3. Test ordinary line following on the same straight, then its bend. The fixed45/45 four-second result is already established; repeating that probe is not a prerequisite. Compare physical hunting/stall/surge before testing a barcode or faster profile.
4. Stop and wait5s for saving. `p` is needed for target/speed/PWM diagnosis; `j` gives barcode events and summary; `z` gives the separate coordinator audit. `v` toggles live state including `FORWARD_EFFORT`. Dumps are refused while moving. Send the full saved `p` output if the trial is still jerky.

## Public interfaces

| Call | Purpose |
|---|---|
| `nav_request_takeover(owner, &lease)` | Stop drive immediately and acquire ownership |
| `nav_request_stop(requester)` | Cancel and stop without automatic resume |
| `nav_drive_distance(owner, lease, mm, speed, &id)` | Bounded forward/reverse movement |
| `nav_turn_relative(owner, lease, degrees, speed, style, &id)` | Bounded rolling/in-place turn |
| `nav_get_status()`, `nav_get_command(id, &result)` | Ownership, progress and completion/failure |
| `nav_resume_line(owner, lease, abandon)` | Explicit handback once stopped with visible line |
| `nav_submit_navigation(BUDDY3, left, right)` | Built-in line/barcode provider's wheel demands |
| `nav_submit_barcode(BUDDY3, code)` | Optional externally verified A/F/Z code |
| `nav_report_terrain(BUDDY4, terrain)` | IMU telemetry, no automatic power policy yet |
| `nav_report_fault(owner, bitmap, active)` | Synchronized shared fault report |
| `nav_request_speed_assist(...)` | Reserved, returns `UNSUPPORTED` |

Acceptance is not completion. Buddy5 owns the whole avoidance/line-reacquisition sequence until explicitly handing back. Stale leases or pre-stop outputs cannot restart motors. The default harness is cooperative on core0; future RTOS/multicore integration needs scheduling and flash-lockout validation.

## Coordinator log

`z` shows requester buddy, current owner, command ID, request, goal/speed/style, acceptance/rejection/result, fault bitmap and measured progress/rotation. Normal Buddy 3 target acceptance is sampled once per second, with separate navigation-phase and decoded-code notices; the per-tick target/PWM evidence remains in `p`. It is not an unbounded dump of every 100 Hz submission. A decoded-code notice is not a new injected command. `NAV_STATE` with `LINE_TARGETS` uses the goal column for the legacy navigation phase; with `BARCODE`, goal is the ASCII code (65=A, 70=F, 90=Z).

The latest 120 audit events are retained; `overwritten` reports older entries lost. `STOPPED` follows measured settling; `DONE` follows finite-command completion and settling. `TARGETS_ACCEPTED` means forwarding accepted, not proof of physical travel. `SESSION_END` requester 0 is the operator/harness stop, whose reason is also in the header. `FAULT_REPORT` state 1/0 means set/clear. API left rotation is positive, unlike the legacy motion log heading sign.

The separate audit uses a CRC-checked 4 KiB flash sector at offset `0x1af000`, immediately before the existing raw log at `0x1b0000`. It saves after the stopped run settles and restores on reboot. Check `NAV FLASH SAVE FAILED` if saving fails. A run without external requests still shows sampled Buddy 3 forwarding/state events. `p` and `j` keep their existing layouts.

## Scope and limits

- The new reverse/arbitrary-angle avoidance primitives are host-tested, not yet physically validated on this robot. In-place turns previously struggled with sensor housings dragging on the mat. Rolling turns shift the axle and must be planned accordingly.
- This provides motion execution and arbitration, not ultrasonic sensing, path planning, automatic line search after avoidance, or an IMU speed policy. Buddy 5 must return the front sensors to the stripe before requesting resume.
- The default harness is cooperative on core 0. Shared coordinator accesses and final motor commits use a Pico spinlock with IRQ exclusion. A dual-core/RTOS integration must additionally provide flash lockout and validate scheduling; do not launch this harness's flash-saving code alongside an unmanaged second core.
- Saved motion record version21 imports earlier records. V1.1/V12.2 cannot import a newly saved version21 record; record calibration references before downgrading. Coordinator logs are separate and versioned independently.
- Known V12.2 limits remain, including straightness qualification of a cross near a recent steering correction and existing sensor-to-axle geometry assumptions. This interface handoff deliberately does not retune those behaviors.
- Compact new request/log representations and shared-state packing are implemented. Whole-firmware minimum-bit compliance, MQTT binary telemetry, and on-target timing/physical evidence remain further work; see the representation document.

## Source map

`motion_api.c` encodes and validates canonical requests. `nav_coordinator.c` arbitrates ownership and records results. `motion_executor.c` handles only new finite avoidance moves. `harness_adapter.h` routes the frozen provider and owns the final hardware commit. `buddy_hooks.c` supplies weak extension hooks. `nav_codec.*` and `nav_log_*` provide compact messages/audit persistence. `v12_2/` is the frozen source snapshot with only documented harness and record-version adaptations; `V12_2_BASELINE.json` and `V12_2_PATCHES.json` record hashes.
