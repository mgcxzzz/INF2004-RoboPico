# First V1.1 DIAG1 physical run — 2026-10-27

Source: user's line_diag.txt, p+d from the same USB hands-free profile1 run. No video attached to this run. This report changes no control code.

## Record validity and timing

12.463s,347.2mm,27.86mm/s; GP21 manual stop; no completed junction turns, decoded letters or invalid encoder transitions. All677 motion rows retained (none overwritten). Diagnostic256 rows span1.228711..3.820499s, trigger2.522461s, stored1/restored0/save_failed0/matches_motion1. The window froze normally after128 post-trigger rows. All256 final wheel requests equal the post-smoothing optical requests; no external owner overrides. All computed PWM values match observed hardware compare-register values. No clipped diagnostic fields.

Whole-run1230 control updates: maximum interval10398us, zero intervals>=20000us; maximum observed control-body cost1131us, maximum packing263us. These bounds show no sampled control-timing collapse. They do not establish zero logging overhead; separate baseline timing was not measured, and barcode2ms sampling is a separate schedule.

## Specific captured transition

At2.512345s: front normalised574/762, raw ADC910/2910, optical requests49/41, smoothed/final54/59mm/s; measured52.5/57.0. PI integrals53.5/59.0 raw units, PWM179/205.

At2.522461s: front normalised382/764, raw253/2881, optical50/33, smoothed/final49/50mm/s; measured51.0/56.5. Both PI integrals become0.0, PWM103/110. DF_LEAVE_FAST(bit128) is set. Filtering remains active; this particular transition is not an urgent-bypass event, white loss, assistance mode, external takeover, missed commit or hidden PWM clamp.

The source exactly explains that event: scan_smooth.h lowers the retained common-speed ceiling when imbalance passes350/1000 (382 here); the final targets no longer exceed55, so circuit.fast clears. main.c then calls circuit_leave_fast for BOTH wheels. That helper unconditionally clears integral and caps references to50. The recorded before/after I and flag prove the global handover happened, not merely an inference from PWM. Removal of about53.5/59 raw compensation accounts for approximately64/71 physical PWM counts of the76/95count drop (quantised values). This erases useful running compensation as well as reducing requested speed.

The same DF_LEAVE_FAST event repeats at3.052145s: final53/56 becomes46/50, both measured speeds40.5/32.5 remain BELOW the new requested speeds, yet left I38.5 becomes0 and right I21 becomes0.5, PWM172/180 becomes108/140. Therefore this blanket reset is not conditional on both motors being actually overspeed.

## Consequent restart pulses

At2.645169s right target22mm/s, measured10, PWM89. At2.655371s target23, measured7.5, PWM300 with DF_KICK_R active. Both optical returns are present (371/567). At2.716441s target33 and measured24, the kick releases and PWM falls300->94. These are the existing breakaway helper's pulses, not the barcode planner requesting300 or the coordinator changing ownership. Similar left and right events follow. The selected window contains4 left-kick and25 right-kick rows,22 urgent-bypass rows and26 white-loss rows; those are sample counts, not separate episodes.

## Interpretation and recommended next change

IR changes DO trigger a steering/speed-state change, but the controller adds a large simultaneous power cut through a specific cruise-exit reset, then uses restart pulses after wheels slow. This is stronger evidence than the earlier general theory of IR noise. A sensor sample crossing a line edge versus varying tape reflectance cannot be disambiguated from these logs alone.

The next experiment should start from this unchanged V1.1 diagnostic baseline and modify only ordinary cruise-to-correction compensation handover: preserve/transfer useful running effort without blindly zeroing both integrals, while retaining normal per-wheel deceleration/anti-windup, deliberate coasting and immediate STOP. Review the subsequent per-wheel falling-target reset too; simply retaining arbitrary high compensation for an intentionally slow wheel would recreate a prior overspeed problem. Confirm this exact DF_LEAVE_FAST condition no longer creates the logged I/PWM cliff before considering independent breakaway or optical changes.

Do not simultaneously change IR calibration, thresholds, cruise speed, bend authority, decoder, search or coordinator. Keep d so the handover can be compared directly. This is not another whole motor-controller rewrite. It cannot yet be claimed that one handover fix will remove every jerk elsewhere in the circuit. No new firmware is published by this analysis.
