# V1.2 physical regression — 2026-10-27

The user's line_test_3.txt and IMG_4310.MOV confirm V1.2 performed worse than the preceding V1.1 run. Do not continue treating V1.2 as an improvement or stable integration baseline. Keep the artifact/source for diagnosis. V1.1 at commit72e11ce retains the startup fix and is the comparison baseline; original V1 must not be used.

## Measured comparison

| Metric | V1.1 profile1 | V1.2 profile1 |
|---|---:|---:|
| Run time |10.924s|16.831s|
| Accumulated distance |309.8mm|295.7mm|
| Average speed |28.36mm/s|17.57mm/s|
| Highest left/right measured wheel speed |62.7/62.4mm/s|84.7/73.5mm/s|
| Strong correction decisions12/13 |19/543 active rows (3.5%)|275/853 active rows (32.2%)|
| Near-stopped (<8mm/s) samples among target>=20 samples L/R |3.8%/5.3%|36.5%/38.2%|

Both were manually stopped by GP21; neither performed a junction turn or decoded a barcode. V1.2's four RECOVERY_WAIT/FOUND pairs are brief line-loss/reacquisition episodes, not four completed search sweeps. Both have invalid encoder counts0/0. Broad front calibration is unchanged (white163/160, black3457/3700). Different initial placement and separate runs prevent attributing every difference exclusively to one cause, but this physical result rejects the claim that V1.2 was an improvement.

## Specific failure mechanism

1. The new gradual ordinary-drive path removes immediate legacy breakaway kicks, but the wheels require different effort to start. Initially the left begins sustained movement before the right; the slow build allows one side to stay stationary while steering errors develop.
2. The existing sharp-correction load helper activates when an outer wheel is underspeed and the inner target is small. V1.2 deliberately bypasses its new smoothing in that mode. At1.072s, right target45, speed7.4, PWM147, assist0; at1.082s, target45, speed5.5, PWM410, assist4. The ordinary30% cap is not being exceeded accidentally: this is the separate legacy loaded-correction path with its42% cap.
3. On return from that path, the new ordinary smoother seeds its duty from the previous high output and limits power reduction. Retained load trim and the old PI also interact. At1.164s assist clears, yet right PWM remains294 while actual speed52.5 exceeds target47; it then surges further.
4. The clearest later example is2.905s: left target39, measured speed84.7, PWM245, while the right wheel is stationary. A smooth decrease in PWM is inappropriate when actual speed is far above the newly requested speed. The wrapper can delay the underlying PI's power reduction, so the chassis continues turning past the stripe.
5. The optical follower then makes a justified strong correction to the other side, deliberately slows/coasts a wheel, and clears retained support. That wheel can stick before the next high-effort assist. The cycle repeats. The video/body motion and alternating sensor/target traces are consistent with this explanation, though encoder estimates do not independently measure slip or chassis yaw.

The speed-qualification change cannot be credited with fixing this: targets still peak at60 in this run, and actual wheel speeds exceed that while the average is lower. Raising speed or all PWM caps would not address delayed removal of drive while overspeed.

## Test coverage gap

The counterfactual input replay held the old measured speeds and sensor-driven targets fixed. It demonstrated removal of some computed PWM edges, not whether the new effort would change speed, cause overshoot, alter optical readings, trigger a legacy load-assist handover, and reverse steering. The simple stiction models did not reproduce the full layered closed-loop transition. Functional API/codec/stop tests passing was not physical smoothness evidence. The next control tests must include both ordinary/loaded transitions and the feedback of motion into steering, as well as a physical same-mat comparison.

## Recommended next steps

1. Return to V1.1 for a same-mat baseline. V1.2 saved record20 is newer than V1.1 can import; note CAL references and verify/recalibrate after downgrade. Do not return to original V1, which has the startup clock bug.
2. Repeat the existing four-second45/45mm/s straight-drive probe on this same mat with the same power, mounting and slack cable. Optical steering is disabled but sensor readings are still logged. Its older record reader also needs valid calibration checked after downgrade. If smooth, focus on the steering/drive interaction; if jerky, characterize ordinary motor effort/feedback first.
3. Measure each wheel's breakaway effort separately from the lower running effort needed to maintain45mm/s on this setup. Reuse one ordinary speed controller with suitable per-wheel feedforward/anti-windup rather than adding another independent compensation layer.
4. Retain a short bounded starting aid if genuinely needed to overcome stiction. Once moving, supply continuous appropriate effort. Ordinary/loaded transitions must be bumpless; do not jump150->410 and then hold elevated duty after the wheel is overspeed.
5. Allow immediate power reduction/zero for overspeed, intentional slowing and STOP. Limiting positive effort changes can be useful, but a symmetric/slow output slew must not override the speed controller's request to remove drive.
6. Restore the existing cruising qualification initially. Change one motor-control element at a time, then validate straight, bend, barcode and junction behavior before separately revisiting brighter/darker-independent cruise qualification.

This is an analysis/recommendation update only. No V1.3 controller or new UF2 is published by this follow-up.
