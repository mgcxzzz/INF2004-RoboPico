# Interface Continuous v1 — ordinary driving controller test

[Download the UF2](firmware/buddy23_interface_continuous_v1.uf2).

This separate firmware starts from Interface V1.1 HANDOVER1 (`de9bacd792ba514c8340070c0789f17941777c02`). It replaces ordinary profile1/3 forward line-drive control to address the measured stop/boost/drop cycle. Hardware smoothness, bend performance and barcode success still need a physical run. Old firmware folders/artifacts remain available.

## First test

1. Flash `buddy23_interface_continuous_v1.uf2`. Press `h` and check **BUDDY23 INTERFACE CONTINUOUS v1**.
2. Keep the same sensor height and existing valid FRONT calibration. Press **1**, then **k**. No new calibration or fixed-speed probe is required unless mounting or calibration has changed.
3. Use the usual startup: USB **g**, release robot/buttons, wait2s. Power bank: wait5s, hold BOTH3s, release, wait1s, tap GP21, wait2s. `s` or either button stops.
4. Start on a straight, continue into a bend, then another straight. First assess movement; include a barcode if there is space, but do not change multiple settings between comparison runs.
5. Stop, **wait5s** for `RUN SAVED`, then save PuTTY output from **p**, followed by **d**. No live output is needed. `j` gives the shorter barcode log; `z` shows coordinator activity.

## Changes and bounds

- Original ADC calibration, optical tracker, Code39 A/F/Z decoder, pending-command/cross logic and manoeuvre state machines remain. The tracker receives every real sample. A/F/Z still request left/right/U-turn at the next separate junction.
- New `continuous_follow.h` calls the optical/navigation core once, retains barcode scan activity/events, and substitutes only the ordinary visible-line output adapter for profiles1/3. Strong steering changes get a fast bounded response rather than the old instantaneous bypass. Mean/differential state remains feasible and mirrored left/right. Actual tight bends can still reduce an inner wheel to zero.
- Profile1 uses the original optical45mm/s base (outer correction requests up to55), without V12.2's extra small-error +15 cruise boost. Barcode scan targets approach45. Profile3 retains qualified cruise up to135, with the new continuous controller. Profile2 remains the old180mm/s ramp experiment; **use profile1 first**.
- Ordinary per-wheel baseline feedforward is a provisional map through0 and150/180 physical PWM permille at45mm/s for left/right. It is based on observed running effort, not a universal motor calibration. Encoder PI compensates load and can reduce effort; there is no forced PWM minimum.
- Continuous PI uses physical Kp1, Ki3, integral bound+/-100, reference acceleration150/deceleration250mm/s^2 and output rate limit1200permille/s in both directions. It does not clear integral on positive target changes or ordinary overspeed. Actual applied preceding output is tracked for anti-windup. Output cap420 (42%); STOP/zero remains immediate.
- A still wheel with reference>=20 and measured magnitude<3 for100ms may use ONE starting aid per positive command segment: up to300permille for<=150ms, ends early once rolling or low target. It cannot repeatedly re-arm on an unchanged positive command. Controller output tracks the applied aid as it releases; normal load compensation continues through the same PI.
- Legacy running breakaway and loaded-pivot controller switching are bypassed only for the new ordinary C_FOLLOW path. Planned turns, junction approach/alignment, white-loss/search, external Buddy5 manoeuvres and profile2 retain their existing drivers. Transitions back to legacy mode discard private latches and seed its PI from preceding output where applicable.

These are deliberate behaviour changes: this is not a byte-for-byte V12.2 driving build. Slower/bounded corrections can affect a tight bend, which is why the first physical comparison includes one.

## Shared interfaces

`nav_api.h`, `motion_api.h/.c`, `nav_coordinator.h/.c`, `motion_executor.h/.c`, `motion_port.h`, `nav_codec.h/.c`, Buddy hooks and their public data layouts remain byte-identical to HANDOVER1. Buddy3 still calls `nav_submit_navigation`, then the coordinator's output drives the motor controller. `motion_commit` with the current generation is the single normal PWM writer. The independent1ms motor guard uses the same new forward-only420 cap when this driver is active. Stops and ownership changes invalidate stale commits and turn outputs off.

Buddy5 can pause/acquire, execute finite movements and resume line following through the existing APIs. Controller state clears on coordinator reset and rearm. Buddy4 terrain telemetry is retained; terrain-driven speed policy remains future work. The standalone circuit harness and saved offline logs stay enabled. See [Buddy5 handoff](BUDDY5_HANDOFF.md) for the extension/build API workflow.

## Logs and storage

Normal motion/calibration record **24** imports records1..23. Existing calibrated values/profile are preserved; earlier firmware cannot import a new24 record when downgrading. Note the calibration if planning a downgrade.

`d` uses diagnostic **format3**, still459bits/58bytes per row and256 rows. Earlier format1/2 diagnostic captures are deliberately rejected so an old control capture cannot be mistaken for this build. Make a new run first. Flash banks remain diagnostic0x1ab000, coordinator0x1af000, raw0x1b0000 and motion0x1d0000; CRC/readback validation and offline restoration remain.

`DRIVER=3` identifies the continuous motor controller. `REF` is its actual reference; `I/FF` use raw-before1.2 units for drivers0/3 (multiply by1.2 for physical PWM units). `REQUEST/PWM` are physical permille; `PWM` is read from the hardware compare registers. Flags256/512 identify the legacy kick or, under driver3, the bounded starting aid. `ASSIST=0` during new ordinary control is expected: there is no separate running-load driver. `OPT`, `SOFT` and `TARGET` distinguish original optical request, new smoothed request and coordinator output.

After2s, diagnostics can trigger on a large PWM jump or differential reversal. A driver3 window is also scheduled at3s if no earlier trigger fires (`trigger` bit4). This retains127 pre-trigger and128 post-trigger samples even when smooth control never triggers the old jump condition. Timing counters cover the whole run. No diagnostic UART output or flash write occurs while moving.

See [BUILD.md](BUILD.md) for executed validation, artifact identity and limits. Host/model checks do not establish physical smoothness.
