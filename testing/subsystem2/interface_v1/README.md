# Buddy 2/3 Interface V1.2 — frozen V12.2 navigation plus shared coordination

**Current test release: V1.2** adds gradual ordinary-drive effort and cruise qualification from steering/encoder history. It retains the V1.1 startup fix and V1 APIs, but intentionally changes ordinary-drive behavior. See [V1.2 changes and test](SMOOTH_DRIVE_V1_2.md). [V1.1 UF2](firmware/buddy23_interface_v1_1.uf2) remains the pre-smoothing comparison build; original V1 has the confirmed startup bug and must not be used.

[Download the standalone UF2](firmware/buddy23_interface_v1_2.uf2). [Buddy 5 handoff and build/start instructions](BUDDY5_HANDOFF.md). [Data representation and evidence scope](DATA_REPRESENTATION.md). [Build verification](BUILD.md).

This is a separate integration baseline, based on `circuit_with_barcode_v12_2` at commit `9df1b95`. V12.2's optical steering, calibration, barcode thresholds, junction geometry, A/F/Z handling, recovery, speed maxima and motor power caps are retained. V1.2 changes ordinary forward effort and the qualification/ramp of cruising speed in profiles1/3; these are no longer an exact V12.2 motor-output baseline. There is no heading lock, new centre calibration or junction-gate fix. The original V12.2 firmware remains available in its original folder.

The default UF2 still runs the circuit test. It includes no ultrasonic driver or obstacle planner. Buddy 5 adds his source to this same executable to produce a second standalone UF2; the two UF2 files do not communicate at runtime.

```text
Buddy 3 line/barcode provider ──┐
Buddy 4 terrain reports ───────┼── nav_api.h → nav_coordinator → motion executor → single motor writer
Buddy 5 avoidance planner ─────┘                        ↑ command status / results
```

`nav_api.h` is the public entry point. Its `motion_*` aliases share the same implementation and do not bypass the coordinator. Public buddy modules must not write motor PWM/GPIO or directly call the internal coordinator/driver. The existing Buddy 3 planner computes its V12.2 wheel demands; all those demands now pass through coordinator ownership checks before the one hardware writer applies them. This preserves its existing navigation state machine rather than rewriting its turn planning.

| Interface | Purpose |
|---|---|
| `nav_request_takeover(owner, &lease)` | Remove drive immediately, claim ownership, wait for measured stop |
| `nav_request_stop(requester)` | Cancel movement and stop; never automatically resume |
| `nav_drive_distance(owner, lease, mm, speed, &id)` | Finite forward/reverse movement |
| `nav_turn_relative(owner, lease, degrees, speed, style, &id)` | Finite left/right rolling or in-place turn |
| `nav_get_status()`, `nav_get_command(id, &result)` | Ownership, running/completed/failed/cancelled result |
| `nav_resume_line(owner, lease, abandon)` | Explicit handback after obstacle clearance and visible line |
| `nav_submit_navigation(BUDDY3, left, right)` | Existing line/barcode provider's wheel demands |
| `nav_submit_barcode(BUDDY3, code)` | Optional external verified A/F/Z provider; built-in reader already handles these |
| `nav_report_terrain(BUDDY4, terrain)` | IMU telemetry, without changing motion in V1 |
| `nav_report_fault(owner, bitmap, active)` | Synchronized shared fault report |
| `nav_request_speed_assist(...)` | Reserved; returns `UNSUPPORTED` in V1 |

`owner` uses `MOTION_BUDDY1` through `MOTION_BUDDY5`. One movement is active at a time. Acceptance is not completion. All requests are bounded, non-blocking calls; caller code polls results. Buddy 5 can pre-empt another owner; other buddies cannot steal an existing external lease. Every finite move/resume validates the lease; stale commands cannot restart motion after a stop or takeover.

## Run the unmodified test

1. Flash the UF2; `h` must show **BUDDY23 INTERFACE v1.2 | V12.2 BASELINE**. Motors are off at boot.
2. Verify the imported `CAL FRONT` line. Existing valid V12.2 calibration imports. If mounting/material changed, calibrate `w` with both over white, `l` with left centred on broad black, `r` with right centred on broad black, then select `1` and `k`. No barcode calibration is needed.
3. USB/PuTTY: `g` starts after a two-second countdown; `f` requires GP21 held. `s` or either board button stops. Power bank: wait five seconds after boot, hold both buttons three seconds, release, wait one second, tap GP21, allow the two-second countdown.
4. Stop and wait five seconds before changing power. Use `j` for barcode events/summary, `p` for motion plus barcode events, `a` for raw barcode ADC, and **`z` for the separate coordinator audit**. `v` toggles live output, including the added `NAV_COORD` mode/owner/fault/target diagnostic. Dumps are refused while moving.

Profile 1 allows60mm/s and profile3 allows135mm/s after400ms settled steering/encoder history; the new qualifier does not read ADC values or require the two levels to match. Existing calibrated optical visibility/urgent-departure detection remains. Barcode approach remains45mm/s. Profile 2 remains the paused 180 mm/s ramp experiment, not a general faster circuit profile. Profile 3 retains its qualified 135 mm/s setting. Start with profile 1 for integration.

## Coordinator log

`z` shows requester buddy, current owner, command ID, request, goal/speed/style, acceptance/rejection/result, fault bitmap and measured progress/rotation. Normal Buddy 3 target acceptance is sampled once per second, with separate navigation-phase and decoded-code notices; the per-tick target/PWM evidence remains in `p`. It is not an unbounded dump of every 100 Hz submission. A decoded-code notice is not a new injected command. `NAV_STATE` with `LINE_TARGETS` uses the goal column for the legacy navigation phase; with `BARCODE`, goal is the ASCII code (65=A, 70=F, 90=Z).

The latest 120 audit events are retained; `overwritten` reports older entries lost. `STOPPED` follows measured settling; `DONE` follows finite-command completion and settling. `TARGETS_ACCEPTED` means forwarding accepted, not proof of physical travel. `SESSION_END` requester 0 is the operator/harness stop, whose reason is also in the header. `FAULT_REPORT` state 1/0 means set/clear. API left rotation is positive, unlike the legacy motion log heading sign.

The separate audit uses a CRC-checked 4 KiB flash sector at offset `0x1af000`, immediately before the existing raw log at `0x1b0000`. It saves after the stopped run settles and restores on reboot. Check `NAV FLASH SAVE FAILED` if saving fails. A run without external requests still shows sampled Buddy 3 forwarding/state events. `p` and `j` keep their existing layouts.

## Scope and limits

- The new reverse/arbitrary-angle avoidance primitives are host-tested, not yet physically validated on this robot. In-place turns previously struggled with sensor housings dragging on the mat. Rolling turns shift the axle and must be planned accordingly.
- This provides motion execution and arbitration, not ultrasonic sensing, path planning, automatic line search after avoidance, or an IMU speed policy. Buddy 5 must return the front sensors to the stripe before requesting resume.
- The default harness is cooperative on core 0. Shared coordinator accesses and final motor commits use a Pico spinlock with IRQ exclusion. A dual-core/RTOS integration must additionally provide flash lockout and validate scheduling; do not launch this harness's flash-saving code alongside an unmanaged second core.
- Saved motion record version20 imports earlier records. V1.1/V12.2 cannot import a newly saved version20 record; record calibration references before downgrading. Coordinator logs are separate and versioned independently.
- Known V12.2 limits remain, including straightness qualification of a cross near a recent steering correction and existing sensor-to-axle geometry assumptions. This interface handoff deliberately does not retune those behaviors.
- Compact new request/log representations and shared-state packing are implemented. Whole-firmware minimum-bit compliance, MQTT binary telemetry, and on-target timing/physical evidence remain further work; see the representation document.

## Source map

`motion_api.c` encodes and validates canonical requests. `nav_coordinator.c` arbitrates ownership and records results. `motion_executor.c` handles only new finite avoidance moves. `harness_adapter.h` routes the frozen provider and owns the final hardware commit. `buddy_hooks.c` supplies weak extension hooks. `nav_codec.*` and `nav_log_*` provide compact messages/audit persistence. `v12_2/` is the frozen source snapshot with only documented harness and record-version adaptations; `V12_2_BASELINE.json` and `V12_2_PATCHES.json` record hashes.
