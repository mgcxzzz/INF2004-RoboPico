# Interface Continuous v2 — reduce weaving

[Download the UF2](firmware/buddy23_interface_continuous_v2.uf2).

This separate version starts from Continuous v1 at`577652daa73afd5f293d59fef7136188a6274974`. It retains the continuous motor controller and changes steering response plus the pre-search white-gap motor handover. Physical straight/bend performance remains unverified until tested.

## Today's straight test

1. Flash`buddy23_interface_continuous_v2.uf2`. Press **h** and confirm **BUDDY23 INTERFACE CONTINUOUS v2**.
2. Keep the existing valid FRONT calibration and unchanged mounting. Press **1**, then **k**. No new calibration or fixed-speed probe is required for this comparison.
3. Run your straight normally: USB **g**, release robot/buttons, wait2s. Power bank: wait5s, BOTH3s, release, wait1s, tapGP21, wait2s. Either button or **s** stops.
4. Watch whether side-to-side heading swings decrease while forward motion stays smooth. A straight with a barcode is useful if available, but a decode is not required for the steering comparison.
5. Stop, wait **5s** for saving, then collect **p followed by d**. Live output can stay OFF. Send the complete log and video if possible. Do not start another run before dumping the one you want.

The full autonomous harness remains enabled: line following, bends, A/F/Z scanning, queued junction left/right/U-turn and continued following. The short straight test is an evaluation step, not a firmware restriction. **Today's straight run cannot validate bend handling.** Test left and right bends later.

## What changed

- Brief visible-line corrections begin at **70% of the original steering differential**. Same-direction correction gradually regains100% over **350ms**. Reversal or a straight original request resets that dwell. A tiny nonzero sensor bias inside the original straight dead zone does not earn stronger correction.
- Damping uses the forward-normalized encoder half-speed`(right_speed-left_speed)/2`. Its0.35 gain is weighted by how quickly the absolute calibrated optical error is falling, with a40ms filtered confidence; the half-speed correction is capped at+/-7mm/s. A true zero-differential straight request also permits damping of residual turning. It is not a heading lock and does not need an IMU.
- On a constant sustained bend, return confidence fades tozero and steering gain reaches100%, so the synthetic steady-input tests retain original turn authority, including a zero inner-wheel target. Actual bend trajectories still require testing.
- Existing mean/differential slew limits remain. No extra filter is placed on the optical error or forward speed; motor reference/PWM ramps, gains, provisional feedforward,42% ordinary cap and one bounded startup aid are byte-identical to Continuous v1.
- During pre-search white-gap recovery/probes, the same continuous motor controller now follows the **original** optical targets. It no longer switches to the legacy low-effort controller solely because the white flag appeared. Explicit zero/coast targets remain zero, the original150ms bridge/coasting policy and2s search confirmation remain, and actual sweep/search manoeuvres use their existing driver.

Profile1 remains the usual optical45mm/s base, profile3 retains qualified cruise up to135 and profile2 is the unchanged180mm/s ramp experiment. **Use profile1 first.** This version does not increase cruising speed or add a new raw black/white threshold.

## Interfaces and Buddy5

Public APIs, nav coordinator, executor, codec and hooks are byte-identical to V1. Buddy3 still submits wheel targets through`nav_submit_navigation`; generation-checked`motion_commit` remains the normal PWM writer, with the independent safety ISR/off path. STOP and ownership changes retain their existing behaviour. Buddy5 default hooks are empty; your friend can integrate later through the existing interfaces. No ultrasonic or IMU policy is added.

Original optical/calibration state machines, barcode decoder, junction detection, planned turns, outgoing alignment and search logic remain. Only the ordinary output adapter and pre-search motor selection change. The existing optional Buddy5 example is inherited source, not enabled in the delivered UF2.

## Saved logs

Normal motion/calibration record **25** imports1..24. Existing valid calibration/profile is retained. Older firmware cannot read a newly saved25 record; note calibration values if downgrading.

Diagnostic format **4** uses **464 meaningful bits in the same58-byte row**, consuming V1's five padding bits. Fields **GAIN** and **DAMP** show steering adaptation when`filter_active` is set: GAIN0..7 maps70..100percent, DAMP0..3 maps confidence0..1. These are coarse diagnostic encodings; the controller uses floating-point values. Earlier d-capture formats are rejected so an old capture cannot be mistaken for this run.

`DRIVER=3` is the continuous motor path, including the pre-search gap/probe. `OPT` is the original optical request;`SOFT` is the adapted request;`TARGET` is the coordinator's final request. `REQUEST/PWM` are physical duty permille;`PWM` is observed from hardware compare registers. I/FF units retain V1's raw-before1.2 convention for drivers0/3. `j`=barcode,`a`=raw barcode ADC,`z`=coordinator audit.

Diagnostic scheduling/ring length, CRC/readback saving and offline restore remain. Flash bank offsets stay0x1ab000/0x1af000/0x1b0000/0x1d0000. No live diagnostic output or flash writes occur while moving.

See **BUILD.md** for checks, sizes and artifact identity. Previous firmware folders and binaries remain available.
