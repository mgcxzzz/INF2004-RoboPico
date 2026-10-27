# V1.3 physical regression — 2026-10-27

The user reports worse jerk in IMG_4315.MOV and linev1_3.txt. V1.3 is not a validated improvement and must not replace V1.1 as the comparison integration baseline. Original V1 has its known startup bug; V1.2 also failed physically. This update changes documentation only, not motor code.

## Observations

The video shows slow intermittent advance with repeated yaw corrections on the straight. It reaches near the cross and is manually stopped. The moving camera/mat prevents an independently calibrated video speed estimate or exact video/log synchronization. No claim of external wheel/heading measurement is made.

The whole-run summary is29.484s,333.3mm,11.30mm/s; zero completed turns or decoded A/F/Z, invalid encoders0/0, GP21 manual stop. The1000-row ring overwrote534 entries: retained active rows begin10.637s, so the opening10.6s cannot be reconstructed from this motion dump. The950 active retained rows all have phase10 (ordinary follow). None has an encoder feedback-warning mask. One brief RECOVERY_WAIT/FOUND pair at11.624/11.765s did not reach a full search sweep.

In retained rows, at least one wheel target is<=15mm/s in50.3% of samples. Both measured speeds are<3mm/s in24.4% of samples. Among target>=20 samples, measured speed<8 occurs37.4% left/52.5% right using sampled-time weights. One both-near-stationary interval lasts approximately656ms from19.087 to19.743s. These are retained-sample statistics, not whole-run time measurements. Encoder motion still does not independently distinguish tyre slip or sensor faults, although video slow progress agrees with the broad pattern.

Targets average approximately30.1/34.2mm/s in the retained window; measured averages11.0/10.7. Max retained wheel speed64.3/58.0; PWM maxima274/311. The controller is already capable of momentary speed far above the poor average. More blanket PWM or a longer loss timeout is not a demonstrated fix.

## Specific transitions and code implications

- At18.853s targets27/45, actual20.0/48.0, PWM167/263. At18.894s targets33/37, actual28.9/44.6, PWM54/89. The left motor's duty drops dramatically while its measured speed is still below its FINAL target. V1.3's overspeed predicate compares with the LOWER of final/ramped target; an acceleration reference left behind by earlier slow commands can clear positive integral even in this situation. The log does not save the internal reference/integral, so attribution of this precise sample to that line is a code-consistent explanation, not a directly logged internal-state measurement.
- Falling-target and overspeed branches abruptly set positive integral to zero. This erases learned running effort; the feedforward is based on old wheel-bench measurements, so the low-speed operating effort on the current mat can be much larger. Output reduction is unrestricted while rebuilding positive effort takes time. This produces a stop/build/move/cut cycle, alongside alternating optical steering.
- At26.799s targets46/33 and left PWM186; by26.860s targets5/46, left PWM18. At27.042s26/47 becomes46/8 by27.103s and right PWM becomes0. Large optical differential reversals and almost-stationary inner-wheel demands remain despite the gradual-positive-PWM wrapper.
- The optical adapter still bypasses smoothing on a large one-sided level difference (>=650 normalized) or a near-pivot demand. Because narrow-line sensor placement can produce those patterns from small displacement, the same code used for urgent bends can be entered repeatedly on a straight. One frame does not distinguish a real tight bend from coverage changes/contrast.
- At11.644s a separate WHITE recovery path outputs right PWM300, falling to92 by11.664s at target20. This is a legacy bypass outside the new forward controller, not proof that the new one-shot forward aid itself emitted this pulse. It is not the dominant retained behavior; most jerks occur with recovery=0.

CAL remains valid: white163/160, black3457/3700, same references as prior runs. This is not evidence of calibration corruption. It does not independently prove accurate sensing on every tape section.

## Recommended next experiment

Use V1.1's motor-controller/coordination baseline; do not stack another wrapper onto V1.3. The fixed45/45 probe is already accepted as smooth and need not be repeated. Isolate normal STEERING demands: modest differential corrections around a common forward speed, keep ordinary inner-wheel requests within a measured sustainable moving range, and escalate toward tight-bend authority progressively when a departure persists. Do not require equal/high black readings or lock the heading at a barcode. Preserve line guidance, existing decoder, planned manoeuvres and explicit stop.

The minimum sustainable speed is not established by the45mm/s probe. Any proposed ordinary35/55 example is only a starting target experiment, not a measured calibration or a rule for all bends. Tight curves can need a lower inner wheel; retain that authority with appropriate persistent-error transition rather than globally clipping steering.

Separately, if revisiting the motor PI, replace abrupt whole-integral resets with continuous saturation-aware anti-windup; avoid treating a lagging acceleration reference as proof of overspeed relative to the requested motion. Reproduce the measured duty cuts and stop/build cycle in tests. Prior simplified models passing was not proof of physical performance. Test one change on the same straight and bend before changing speed/calibration/geometry simultaneously. This report does not publish a V1.4 firmware.
