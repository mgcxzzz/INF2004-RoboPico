# v2.3 + Buddy4 add-on v0.1 hardware tests

Boot functional checks passed: Ready/Operational, Buddy4Idle, zero speed, fresh odometry, encoder0/0, I2C0, missed0. Native MQTT source14 shows TX drops increasing; RX0. Idle max control gap11221us, ADC gap0 (inactive). Original-v2.3 hardware comparison and active timing remain pending. No motor command issued in this phase. SDK firmware preserves originalv2.3 control/button behavior, and has no RTOS Safety supervisor or300mm bench bound. Boot evidence: logs/2026-10-29_v23_b4_v0_1_boot_phase.json.

Static25mm block placed: PASS static25mm estimate; settled10s range21..22mm, median22mm, 20frames, fresh=true, valid=true, stopped=true. NAV=Ready. Evidence: logs/2026-10-29_v23_b4_v0_1_static25_placed_review.json.

Static25mm block return: PASS return toflat; settled5s range-2..1mm, 10frames, all fresh/valid/stopped=true/true/true, I2C=0, missed=0. Evidence: logs/2026-10-29_v23_b4_v0_1_static25_return_review.json.

Short line-follow run: PASS short line-follow/commanded-stop function. User observed normal line following and commanded stop. Final NAV=Safe Stop, speeds0/0, encoder invalid L+0/R+0, I2C+0, missed+83. Encoder distance delta239mm; this was not independently measured. Latest diagnostics max ADC gap6696us, control gap12682us; TX drops1889, RX0. Comparative baseline-v2.3 hardware timing still pending, so do not claim timing parity. Evidence: logs/2026-10-29_v23_b4_v0_1_line1_review.json and rows.

Interpretation of short-line capture: two Start/Stop runs were logged. Missed-counter jumps0->41 and41->83 were first reported after Stop in Idle. Existing v2.3 waits1s then calls blocking save_log(); its post-stop flash work is a plausible source, not directly measured. Driving functionality passed; do not infer full-period timing compliance or timing parity. Source2 freshness dropped in the same post-stop snapshots. Keep83missed and ADC/control maxima as evidence; no suppression or firmware change.

Powered10mm ramp attempt1: mechanically confounded. Front-left IR sensor caught on combotiles and user dislodged car. Final NAV=Safe Stop, liveheight=0mm, storedheight=0mm, humps=0, rejected=0, invalid totals L0/R0, I2C=0, missed=126. Counter comparison must use post-Reset baseline, not prior83 missed. Preserve run; do not count as clean ramp accuracy/clearance pass. Evidence: logs/2026-10-29_v23_b4_v0_1_ramp10_run1_review.json and rows.

Powered ramp attempt2: user reports stutter and raising IR sensor angle. Mark as changed-sensor-geometry/confounded; no clean accuracy pass. Final NAV=Fault, speeds0/0, invalid totals L0/R1; humps0, rejects0. Restore known-working IR mounting or recalibrate with existingv2.3 procedure before another line/ramp run. Evidence: logs/2026-10-29_v23_b4_v0_1_ramp10_run2_review.json and rows.

Attempt2 stop reason confirmed from SDK run summary:47=LS_TIMEOUT, line not recovered/five rolling sweeps exhausted. Sensor-angle change may explain degraded detection but is not proven as sole cause. No motor-code change recommended before restoring known-working sensor geometry and rechecking plain line following.

Reset-display finding after LS_TIMEOUT: user observed Fault remains. Live heartbeat reports RESET accepted, NAV Fault. Source confirms original wifi_state() reports circuit.fault, accepted WP_CMD_RESET clears Wi-Fi flags/lease and resets Buddy4 but does not clear circuit.fault. A fresh successful start_run() calls circuit_begin()/motion_session_begin() to reset per-run state. Preserve baseline behavior; this is distinct from RTOS Reset semantics. Do not treat Reset acceptance as proof all fault states cleared.

After user restored IR sensor, user reports3further plain-line attempts all stuttered. Stop repeated testing; cause not established. Exact summaries/diagnostics/source9motion preserved in logs/2026-10-29_v23_b4_v0_1_line2_restored_review.json. Need comparable originalv2.3 hardware test on same current mounting/calibration/surface/power before attributing failure to add-on.

Prepared originalv2.3 A/B baseline with same local Wi-Fi header as add-on. All185 source manifest files match, zero baseline code edits, Buddy4 absent. ARMbuild exit0, warnings0, errors0; UF2 format/family/bounds checked. Local firmware /Users/skyler/SIT/Y2T1/Embedded/bench_tools/buddy4/buddy123_interface_wifi_v2_3_baseline_local_v0_1.uf2; manifest /Users/skyler/SIT/Y2T1/Embedded/INF2004-RoboPico-v23b4/testing/subsystem2/interface_wifi_v2_3/V23_BASELINE_LOCAL_BUILD_v0_1_MAC.json. Not flashed automatically. Preserve current sensor/calibration/mat/power between comparisons.

Originalv2.3 A/B baseline also stutters (user observation). NoBuddy4, all185baseline files verifiedunchanged. Latest NAV=Safe Stop; wheel speeds0/0; diagnostic invalid0/0, maxADCgap4511us/controlgap11513us. Add-on not necessary for symptom; do not infer it has no timing effect. Preserve sensor mounting and investigate stationary optical readings before furtherdriving. Evidence: logs/2026-10-29_v23_original_local_line1_review.json.

Originalbaseline stationarywhite reference: ADC L142..153 (median147), R169..173 (median172); normalized L0..0, R2..3, stopped=true. No calibration changed. Await black contrast. Evidence: logs/2026-10-29_v23_original_local_white1_review.json.

Stationaryblack line reference: ADC L3271..3294 (median3283), R2681..2709 (median2694); normalized L943..950, R712..720; contrast medianADC L3136/R2522; stopped=true. These static readings do not prove dynamictracking. Evidence: logs/2026-10-29_v23_original_local_black1_review.json.

Right-sensor-onlycentered blackreference: ADC R3089..3106 median3098; normalized R827..832; L ADCmedian159; stopped=true. User intentionallycentered rightsensor; no firmware/calibrationchanged. Evidence: logs/2026-10-29_v23_original_local_right_centered_review.json.

Front recalibration on originalbaseline via existing USBcommands, motorsstopped/MacUSBonly: white158/183, black2532/2913, spans2374/2730, valid1/profile1. k reported CALIBRATION/PROFILE SAVED. Oldreferences163/160,3457/3700 preservedin evidence. No source/firmware edits; no motor commands. Powerbank validation pending. Evidence: logs/2026-10-29_v23_original_front_recalibration.json and consolelog.

Post-recalibration originalbaseline: user reportsbetter, somewobble. ManualStop reason49, duration8632ms/distance310mm. LatestSDKinvalid0/0, maxADCgap6084us/controlgap11257us. 22 reversals between differential targets with magnitude>=10mm/s over170active20Hz frames. Examples L/R20/44 then45/19mm/s; opticallevelsalternate and feedbacklags. Consistent with steeringhunting, solecauseunproven. No speculative motor/IMUchanges. Evidence: logs/2026-10-29_v23_original_recalibrated_line1_review.json.

User clarified remainingwobble is forwardstop/start, not side-to-side. Two brief recovery-wait/low-opticallevel episodes (12/170activeframes) coincide with coast/zeroPWM segments, near-zero speed and resumeddrive; consistent with commanded gap/recovery behavior. Levels48/138 and0/143 are below/near the line-search150/200hysteresis thresholds. This explains a plausible jerk mechanism without proving why optical line contact is lost. Plan stationary centered straight-line placement check awayfromcrosses; no motor or gain changes.

Latest user assessment after recalibration: not completelysmooth, but okay and followsline. Acceptfunctionalbaseline with minorroughness; do not claim quantitative smoothness/peak-speed pass. Stop additional motor-tuning investigation and returntoBuddy4 integration tests. Current frontcalibration158/183 white,2532/2913 black, profile1 saved. Optionalcenteredstraight stationarycheck notperformed. Timing/network findings remain recorded; no firmware changes.

Post-recalibration add-on plainline check: user saysseemsokay. Functional line-follow/Stop accepted by user; finalNAV=Safe Stop, speeds0/0; invalidtotals L0/R0, I2Cdelta0, misseddelta42; sampledTrackingmissedvalues 0. LatestADCmax7097us/controlmax13799us. Functionalacceptance only; timing/network limitations retained. Evidence: logs/2026-10-29_v23_b4_v0_1_postcal_line1_review.json androws.

Postcal poweredramp attempt3: contact ondescent causedstall peruser; photos show low frontIRclearance, no dimensionalclaim fromphoto. Mark mechanicallyconfounded. FinalNAV=Safe Stop, speeds0/0, humps0, rejected1, storedheight0mm. Proposed smallestupwardmount adjustment, unpoweredfullramp clearancecheck, then staticcontrast/frontrecalibration and plainlinecheck. No firmwaretuning. Evidence: logs/2026-10-29_v23_b4_v0_1_ramp10_run3_postcal_review.json androws.

RaisedIRmountcalibration captured andsaved with existingUSBcommands: white142/153, black278/1828, spans136/1675, firmwarevalid1, profile1. Leftblack273..281ADC/stddev1.65; rightblack1822..1831/stddev2.83 over5s. Leftstaticcontrastweakbutstable; notproofdynamicreliability. k reportedCALIBRATION/PROFILE SAVED. Oldsavedreferences158/183white,2532/2913blackpreserved. No motor/PWM/firmwarechanges. Onecontrolledplainlinetestnext; norampuntiltrackingconfirmed. Evidence: logs/2026-10-29_v23_b4_raised_ir_recalibration.json andstability/consolelogs.

User temporarilyconnectedsparesensor toleftinput atsame raisedheight/angle. Stationarywhitereference: ADC L194..202, median199, stddev1.50 over100frames; stopped=true. No calibrationchange or driving. Await matchedblackreference; loosemount notapproved fordriving. Evidence: logs/2026-10-29_v23_b4_spare_left_white_review.json.

Spareleftsensor matchedblackreference: ADC3062..3082, median3070, stddev3.80; white median199, mediancontrast2871 versusoriginalraisedleft136 (21.1x). Same reportedheight/angle/inputchannel, temporarymount. No calibrationchange/driving. Looseposition means not a lab-controlled sensorfault diagnosis; this doescompare usability atrequiredclearance. Evidence: logs/2026-10-29_v23_b4_spare_left_black_review.json.

Mountedspare retry after userconfirmed middlemoduleinfrontphoto isleft: latest5s L224..231/median228, R150..157/median155. BarcodeADC235 atsample. Existinglcommand recapturedleftblack230 versuswhite173 (span57); calibrationnot saved. Both original andspare lowwhenmounted, sparepreviouslystrongwhenheldloose; sensor-onlyfault unproven. Nextproposedstationaryisolation: poweroffdisconnectotherfrontsensor, keepleftmount/height/blackpositionfixed, USBonly/noStart, compareleftADC. Possibleopticalinterference is a hypothesis, not diagnosis.

Neighboroffexperiment: useralsomoved tobetterblackspot. LeftADC3471..3478, median3474, stddev1.41, stopped=true. Bothneighborconnection andtargetlocationchanged; nointerferencecausalityclaim. Rightinputdisconnected; no drivingorcalibration. Needneighbor-on comparison atsameblackspot/height. Evidence: logs/2026-10-29_v23_b4_spare_neighbor_off_changed_black_review.json.

Pairedneighbor-on comparison atusersameblackspot/height/angle: leftADC234..241, median238; neighboroffmedian3474, difference-3236. RightADCmedian152; stopped=true. No calibration/motorcommands. Evidence: logs/2026-10-29_v23_b4_spare_neighbor_on_same_black_review.json.

Neighborcomparison finding: leftblackmedian3474(off) ->238(on), difference-3236 atuserreported fixedspot/mount. Stronginteraction, mechanismunresolved. Proposednext: bothmodules remainpowered/connected; opticallycoveronlyotherfrontmodule withnonconductiveopaquecard, fixedleftposition, noStart. Positiveleftrecovery wouldsupportopticalmechanism; unchangedreading wouldnotalone ruleitout ifcover ineffective. Preserveelectricalalternative. Calibrationnot saved duringdiagnosis.

Bothfrontsensorsconnected withuserblackpaperdivider: leftADC2487..2605, median2590; rightmedian154; comparison neighboron/no-divider238, neighboroff3474; stopped=true. DividerIRopacity/geometrynotindependentlyverified, so unchangedreading cannotexcludeallopticalpaths. Calibrationunchanged. Evidence: logs/2026-10-29_v23_b4_spare_divider_review.json.

Divider interpretation: bothmodules powered/connected; leftmedian238(no-divider)->2590(divider), otherfrontrawstable154. Strongsupportforopticalinterference/coupling contribution; notfullfaultisolationbecause dividercouldalsochangeview/placement. Proposedhardwarefix issecuredopaquedividerwithunpoweredrampclearancecheck, followedbyfreshfrontcalibration. No motor/PWM/sourcechange. Neverdrivewithloosedivider ormissingfrontsensor.

Finalsecureddivider calibration validandsaved usingexistingUSBcommands: white191/154, black1940/2306, spans1749/2152, profile1. Userconfirmedraisedbracket, mountedspareleft, fixeddividerandunpoweredrampclearance. k:CALIBRATION/PROFILE SAVED. Bothsensorsconnected; no motor/PWM/firmwarechanges. Poweredline/rampvalidationpending. Evidence: logs/2026-10-29_v23_b4_secured_divider_recalibration.json andconsolelog.

Secured-divider bootIMUcalibrationtimeout code3 followedbydashboardReset: latestIMU=Idle, valid=true, NAV=Ready, speeds0/0; recoverycheck=true. No firmwarechange. Evidence: secured_divider_line1_phase resetCheck.

Secureddivider poweredplainline trial: userreportsnowleft/rightweaving. ManualStop reason49,8.754s/259mm;35strongdifferential-target signreversals (>=10mm/s), recoveryState0throughout, invalid0/0/I2C0. FinalNAVSafeStop/speeds0/0. Steeringoscillationobserved; severity/useracceptancepending. Newmount/calibration mayaffectresponse; no proofsinglecause, and matchingoriginalbaselinewiththisfinalhardware/calibration notyetperformed. No firmware/gainchange. Evidence: logs/2026-10-29_v23_b4_secured_divider_line1_review.json.

UserreportsfailedYbarcodeatcurrentheight; confirmedbarcodeIRalso raisedwithbracket. Recentruns4/5 decoded0; run5 barcodeADC195..2301, invalid0/0, no recovery stateinrun5. Lastsource14 bank11/threshold212, badFrames0/unsupported0, maxADCgap6893us. Height/blur/spotgeometryandtiming needhighratecapture;20HzliveADCisinsufficienttoidentifyedgefailure. No frontcalibrationorfirmwarechangeforthisfinding. Exportboundedstoredraw/nav/diagnosticsbeforeanynewrun/reset. Evidence: logs/2026-10-29_v23_b4_recent_y_barcode_review.json.

## 30 Oct: secured-divider powered10mm ramp

Userconfirmed no sensor/bracket/dividercontact andnoassistancebeforeStop: mechanicalclearance functionalPASS. UserliftedcarAFTERStop; handling isseparate, notgrounds todiscardearlierrampdata. Runningduration16.579s, encodertravel332mm, manualStopreason49. FinalSafeStop/speeds0/0.

HeightvalidationNOTestablished: lateststoredheight0/humps0/rejected0, nohumpeventsreceived. PreStopperiodicsampledheight-5..8mm,34frames/allfresh,28sidevalid,5levelready,0sampledcandidates. Motionqualityflags includeMAD/acceleration/turn andimpact/geometry; recovery states0/1/2. A reportedimpactat00:30:47.209 occurredbeforeStop~00:30:48.8; laterhandlingimpact00:31:52.638 separate. LatestSDKinvalid0/0, I2C0; Trackingeventsreportmissed0, postStopcumulativemissed43. Do notcall8mmamaximumacceptedrampheight orclaimaccuracyPASS. Preservequalitychecks; linefollowing improvement/targetedarminganalysis neededbeforefurtherpoweredrepeat.

Evidence: logs/2026-10-30_v23_b4_secured_divider_ramp10_run1_review.json (fullSDK/Buddy4rows, events, diagnostics, acknowledgments).

## 30 Oct: ramp-assist v0.1 test build prepared (NOT flashed, no hardware run)

Built on the exact v2.3 base (8d7f3aa) and the Buddy4 add-on v0.1. There are two images:

- `buddy123_interface_wifi_v2_3_b4_v0_1_ra_v0_1_shadow.uf2` (sha256 a58a252c…): logs only, motors unchanged.
- `buddy123_interface_wifi_v2_3_b4_v0_1_ra_v0_1_active.uf2` (sha256 005c7384…): bounded assist applied.

Both are in `Embedded/bench_tools/buddy4` and contain credentials, so they stay local and out of Git.

- **What the assist is:** +20 % of the existing continuous-drive feedforward on both wheels. It is capped at 40 PWM per wheel and ramps in and out over 200 ms.
- **When it starts:** only after a valid Buddy4 tilt of at least 4 mm holds for 80 ms.
- **When it ends:**
  - level, descent, invalid or stale tilt, or the 2.5 s timeout;
  - immediately on pause, Stop, ESTOP, fault, line search, line recovery or junction manoeuvre.
- **Unchanged:** the existing PWM, integral and slew limits, targets and stop paths.
- **Telemetry:** source 15, decoded by `ramp_assist_decode.py`.
- **Constants are provisional.** The missing measurements are listed in `testing/subsystem2/interface_wifi_v2_3/RAMP_ASSIST_V0_1.md`. The shadow run collects them.
- **Run 3 evidence:** the lower-bracket capture showed the controller reaching its integral (authority) limit (output 252/115), not the 420 PWM limit. This happened during a stall the user attributed to IR contact.
- **Found and fixed:** the v0.1 Reset hook had broken the host test `test_wifi_harness.c`. Firmware behaviour was unaffected.
- **Host tests:**
  - ramp-assist suite: 360,841 checks (clang, gcc, gcc ASan/UBSan);
  - existing `run_tests.py`: 35 C executables PASS;
  - Buddy4 host test: PASS.
- **ARM builds:** both images built with 0 warnings; UF2 format verified.
- **Calibration:** commands are unchanged, but flashing keeps the last saved calibration (191/154, 1940/2306). The left sensor has been replaced and the bracket is now lower, so do a stationary white/black check before driving.

Evidence:

- `testing/subsystem2/interface_wifi_v2_3/RAMP_ASSIST_BUILD_MANIFEST_v0_1.json`
- `testing/subsystem2/interface_wifi_v2_3/RAMP_ASSIST_HOST_TESTS_v0_1.txt`

## 2026-10-30 — ramp-assist v0.2 build (crest-hold detector fix), not yet bench-tested

- Trigger: shadow run 1 (02:27, lower bracket, 10 mm right-wheel ramp): v0.1 would have ended/latched the assist with DESCENT 241 ms after activation (tilt rocking 4–7 mm mid-climb); the car then stalled at the crest at 8–9 mm with both |I| = 100 (barcode IR touching the mat; shared bracket, lower setup kept on purpose).
- Change (detector only): 3-sample median of valid |h|; DESCENT needs filtered |h| <= peak/2 (peak >= 6 mm) for 100 ms; LEVEL needs 100 ms. Boost 0.20 / cap 40 PWM, activation, 2.5 s timeout, invalid/stale ends, all stop/pause/fault/search/junction gates and controller limits unchanged.
- Host: replay of the captured climb — v0.1 logic DESCENT at 220 ms, v0.2 full assist to TIMEOUT at 2490 ms; noisy-climb and 60 ms-dip cases hold; all v0.1 checks pass (one post-release overshoot bound relaxed to 3 mm/s for the abrupt-load worst case, integral excess 0). run_tests.py 35 and Buddy4 host test pass.
- Firmware (local only): `bench_tools/buddy4/buddy123_interface_wifi_v2_3_b4_v0_1_ra_v0_2_{shadow,active}.uf2` (active sha256 d4634ecb8f2f1f1c…, shadow 9566d070b92f16d8…), 0 warnings. Details: `testing/subsystem2/interface_wifi_v2_3/RAMP_ASSIST_V0_2.md`.
- Bench result: pending (short supervised active run).

## 2026-10-30 ramp-assist active v0.2: first crossing

Version reported flashed by user; source15 confirms active mode but does not encode release version. Lower bracket/original preset, same10mm right-wheel ramp. User confirms crossed unaided. Run2:323mm,8132ms, manualStopreason49, encoderinvalid0/0, recovery0, finalPWM0/0. Physical crossing PASS for this one attempt; repeatability not established.

Detector caveat: assist activated02:58:33.637 at5mm, ended02:58:33.841 after202ms with DESCENT (peak7/current4mm), then stayed latched while later tilt rose to13mm and finally returned0mm. Logged end-event additions28/30PWM; event peak31PWM. Thus the crossing succeeded, but evidence does NOT show assistance held through the crest or prove the premature cutoff is fixed. Confirm binary release identity and review this sequence before treating detector behaviour as validated. No further firmware change is inferred necessary solely from the successful crossing.

Evidence: testing/integration/logs/2026-10-30_ramp_assist_active_v0_2_ramp10_run1_review.json (full decoded motion and assist events).

## Active ramp attempt2 confirmed
User confirms complete crossing without pushing or lifting. SDK run3:354mm/9241ms, manualStopreason49, recovery0, encoderinvalid0/0, finalPWM0/0. Assist activated03:01:33.029 at4mm, ended03:01:33.251 after214ms, reasonDESCENT, peak7/current4mm, peakaddedPWM36. Later tilt interval max15mm. Evidence: testing/integration/logs/2026-10-30_ramp_assist_active_v0_2_ramp10_run2_review.json.

Active build physical crossings2/2 unaided; shadow v0.1 comparison1failed atcrest withbarcodeIRcontact. This is preliminary evidence of improvement, not proof of causality or established reliability. Both active runs show earlyDESCENTtermination followed by higher tilt; do not claim v0.2 holds assistance throughcrest. Source15identifies active/shadow, not release version: verify actual flashed binary before changing detector again. Keep all firmware and raw evidence.

## 2026-10-30 02:58 / 03:01 — "active v0.2" ramp runs 1–2: image was v0.1 active

- Raw source-15 frames (flat + both ramp runs, 37/37) carry flags detector bits = 0; v0.2 always sets 2. The DESCENT ends (peak 7 / current 4 mm, ~200 ms) are only possible under the v0.1 rule. The flashed image was ra_v0_1 active, not ra_v0_2.
- Both runs crossed unaided, but the assist ended ~200 ms into the climb; added PWM was 0 over the crest. Left |I| sat at the limit for ~1–1.5 s, then the car crossed. Treat these as unassisted v0.1 crossings at a marginal crest (cf. shadow run 1 stall), not as assist evidence.
- v0.2 not yet bench-tested. Next: flash ra_v0_2 active (sha256 d4634ecb…), confirm source 15 detector = 2 on the flat run, then 1–2 supervised ramp runs. Review: `claude_output/HANDOFF_2026-10-30_RAMP_ASSIST_V0_2_RUN_REVIEW.md`.

## Correction: verified firmware identity
The preceding flat and two unaided active ramp runs were v0.1 ACTIVE, not v0.2. Raw source15 flags bits4-6 are0 (legacy detector1); v0.2 marks2. Codex previously missed this marker. Historical filenames containing v0_2 are retained but JSON metadata now records the verified actual release. Early202/214ms descent terminations are v0.1 behaviour, not evidence of a v0.2 regression. Two crossings are physical observations only; effectiveness of boost at the crest is not established. No boost was applied through the crest in these captures. v0.2 has not been hardware validated. Local v0.2 active UF2 SHA256 verified: d4634ecb8f2f1f1c2f6130e105b85c2f00af2ab1718fde244337238df99c8e72. Next step: manual flash that exact file, then require detector2 in source15 before attributing any test to v0.2.

## 2026-10-30 verified detector2 active ramp run1
User confirms crossed unaided. Source15 detector2 on16/16frames verifiesv0.2 active. SDKrun2:332mm/7467ms, manualStopreason49; recovery0, encoderinvalid0/0, finalPWM0/0. Assistance activatedat4mm (03:10:51.618), held2492ms and endedbyTIMEOUT (03:10:54.106), currentheight5mm/peak14mm; laterheightreturned0. PeakaddedPWM37. This verifies the premature~200ms cutoff did not recur in this attempt; descent/level termination not exercised because timeout won. One successful crossing is preliminary; do not infer general reliability or exact physical rampheight from transienttilt.
Evidence: testing/integration/logs/2026-10-30_ramp_assist_v0_2_verified_ramp10_run1_review.json.

## Verified detector2 ramp repeat2
User confirms unaided complete crossing. Telemetry summary:
```json
{
  "detector_counts": {
    "2": 19
  },
  "summaries": [
    {
      "time": "2026-10-30T03:13:19.547602000+0800",
      "run": 3,
      "reason": 49,
      "duration_ms": 8470,
      "distance_mm": 372
    }
  ],
  "last_diag": [
    {
      "time": "2026-10-30T03:13:45.559409000+0800",
      "invalid_left": 0,
      "invalid_right": 0,
      "control_gap_us": 13671
    }
  ],
  "recovery_states": [
    0
  ],
  "events": [
    {
      "time": "2026-10-30T03:13:16.757949000+0800",
      "event": "end",
      "last_end": "invalid",
      "active_ms": 1702,
      "height_mm": "",
      "peak_mm": 12,
      "peak_added_pwm": 37
    },
    {
      "time": "2026-10-30T03:13:16.896734000+0800",
      "event": "activate",
      "last_end": "invalid",
      "active_ms": 0,
      "height_mm": 11,
      "peak_mm": 11,
      "peak_added_pwm": 10
    },
    {
      "time": "2026-10-30T03:13:17.614254000+0800",
      "event": "end",
      "last_end": "invalid",
      "active_ms": 718,
      "height_mm": "",
      "peak_mm": 11,
      "peak_added_pwm": 33
    },
    {
      "time": "2026-10-30T03:13:17.715643000+0800",
      "event": "activate",
      "last_end": "invalid",
      "active_ms": 0,
      "height_mm": 6,
      "peak_mm": 6,
      "peak_added_pwm": 17
    },
    {
      "time": "2026-10-30T03:13:17.801599000+0800",
      "event": "end",
      "last_end": "timeout",
      "active_ms": 70,
      "height_mm": 12,
      "peak_mm": 7,
      "peak_added_pwm": 28
    }
  ],
  "i_limit_ticks": 11,
  "last_motion": [
    {
      "time": "2026-10-30T03:13:45.975233000+0800",
      "uptime": 353438,
      "run": 3,
      "active": false,
      "recovery": 0,
      "speed": [
        0.0,
        0.0
      ],
      "target": [
        0,
        0
      ],
      "pwm": [
        0,
        0
      ],
      "distance": 372
    }
  ]
}
```
Evidence: testing/integration/logs/2026-10-30_ramp_assist_v0_2_verified_ramp10_run2_review.json
Two verifiedv0.2 active crossings now confirmed; small sample, not general reliability proof.

## Accepted-height review: still incomplete
User correctly noted no accepted peak. Decoded Buddy4 source4 from both verifiedv0.2 ramp captures: acceptedhumps0, acceptedheight0, highest0. Run1 emitted humpstart03:10:52.046 and humprejected03:10:53.126 detail5=IMU_REJ_NO_ODOMETRY (fresh-distance availability rejection), eventheight13mm. Run2 emitted humprejected03:13:16.711 detail3=IMU_REJ_RANGE, eventheight13mm, plus motion/shock activity. These reason codes identify the rejection paths, not a proven underlying cause. Rawsource4events and compact acceptance summaries are stored in both run_reviewJSONs.

Physical crossing/assist operation checks are distinct from height-estimation acceptance. Heightestimation remains unvalidated in powered integration. Do not call the firmware fully working or remove validity checks to force acceptance. Next focused investigation: why estimator fresh-odometry gating rejected run1 despite clean encoder diagnostic counters; which intermediate value triggered range rejection inrun2. Preserve originalv2.3drive behaviour and do not increasePWM as an assumed fix for measurement acceptance.

## 2026-10-30 powered acceptance fix — hardware pending

See subsystem2/interface_wifi_v2_3/BUDDY4_POWERED_ACCEPTANCE_V0_2.md and logs/2026-10-30_buddy4_powered_acceptance_v0_2_host_tests.txt. Rejection detail5 was an instantaneous turn gate, not demonstrated stale odometry; detail3 was a transient acceleration quality rejection, not an out-of-range physical height. Built separately versioned B4v0.2/RA v0.2 active firmware; 24 new checks, 45 existing side-ramp checks, service+decoding, 35 v2.3 C executables and unchanged assist suite pass. Hardware acceptance and accuracy remain untested.

## New estimator flat regression — investigate before ramp
New estimator2/detector2 build has two reported flat line-following interruptions. First run lost line around188mm and searched near255mm. User reversed mat for repeat: line loss160mm, active search166mm, reacquired~292mm; travel304mm; encoderinvalid0/0, maxcontrolgap13915us; noassistactivations; source15markers2/2. User reports same behaviour. Cause remains unconfirmed. Do not mark flat regression passed or attribute solely to a tile seam. Next diagnostic: same hardware/calibration A/B against exact previously tested B4v0.1/RA v0.2 active image (SHA d4634ecb8f2f1f1c2f6130e105b85c2f00af2ab1718fde244337238df99c8e72). Source main/motor hashes match but build/runtime timing effects have not been excluded. Preserve new estimator as test candidate, not accepted release.
ReviewJSON: testing/integration/logs/2026-10-30_b4_estimator_v0_2_flat_repeat_033553_review.json.

## Flat A/B outcome: old estimator also loses line
Verified detector2/estimator1 on21 source15frames. User reports same issue on the exact previous image. Driven run:300mm/10403ms, manualStop49; first line loss124mm, active search128mm, encoderinvalid0/0, maxcontrolgap13336us, boostactivations0. A short earlier session summary reason5/duration2ms/0mm also appears before the driven run, with an uptime reset; preserve it separately, do not merge sessions. The new estimator is not required for the symptom. Underlying line sensing/line-following setup remains unresolved; no proof of full timing parity from this A/B alone. Next check stationary normalized optical visibility at the normal starting alignment, rather than more ramp/drive tuning. Code declares white when neither sensor reaches ink hysteresis.
Evidence: testing/integration/logs/2026-10-30_b4_old_estimator_ab_flat_033934_review.json.

## First estimator2 ramp attempt: invalid IMU precondition
04:05 driven run: detector2/estimator2,337mm/8019ms, recovery0, encoder invalid0/0. IMU source4 was Fault/CAL_TIMEOUT3 before Start and stayed faulted; no assist activation, hump start, rejection or accepted peak. Thus not a powered-height acceptance test. SDK line calibration validity does not establish IMU readiness. Before retry, manual Reset with car flat/still, then verify fresh source4 Idle/valid/lastFault0 before allowing Start. Physical unaided crossing still awaiting user confirmation.

## Hardware validation: first accepted powered 10 mm ramp
After manual Reset and confirmed fresh IMU Idle/valid/fault0, user reports complete unaided crossing. Fresh source15 frames confirm detector2/estimator2 (16frames). SDKrun2:353mm/8319ms, manualStop49, recovery0, encoderinvalid0/0, maxcontrolgap13124us. Ramp assist activated04:08:35.726 at5mm, endedonDESCENT after2376ms at1mm (peak10mm), peakaddedPWM36. Source4 humpstart04:08:35.741 and acceptedhumppeak04:08:38.743:height10mm,highest10mm,humps1,rejected0,shocks0,lastFault0. Finalmotors0/0,IMUIdle. This is one observed powered acceptance with estimated10mm matching user-measured10mm ramp; repeatability and broader accuracy remain pending.
Evidence: testing/integration/logs/2026-10-30_b4_estimator_v0_2_ramp10_after_reset_040810_review.json. Native capture preserved. No external sharing required.
Next: one unchanged ramp repeat, then Stop during assistance. Existing intermittent flat line-following issue remains separately open and was absent in this ramp run.

## New estimator powered ramp repeat (04:10)
User confirms complete unaided crossing. Evidence and decoded outcome:
```json
{
  "versions": {
    "detector2/estimator2": 18
  },
  "summaries": [
    {
      "time": "2026-10-30T04:10:53.880205000+0800",
      "run": 3,
      "reason": 49,
      "duration_ms": 8391,
      "distance_mm": 345
    }
  ],
  "last_diag": [
    {
      "time": "2026-10-30T04:11:12.900573000+0800",
      "invalid_left": 0,
      "invalid_right": 0,
      "control_gap_us": 13850
    }
  ],
  "recoveryStates": [
    0
  ],
  "assistEvents": [
    {
      "time": "2026-10-30T04:10:49.746066000+0800",
      "event": "activate",
      "last_end": "none",
      "active_ms": 0,
      "height_mm": 4,
      "peak_mm": 4,
      "peak_added_pwm": 2
    },
    {
      "time": "2026-10-30T04:10:52.247983000+0800",
      "event": "end",
      "last_end": "timeout",
      "active_ms": 2495,
      "height_mm": 6,
      "peak_mm": 11,
      "peak_added_pwm": 36
    }
  ],
  "rampEvents": [
    {
      "time": "2026-10-30T04:10:50.016406000+0800",
      "event": "hump start",
      "detail": 0,
      "humps": 0,
      "rejected": 0,
      "heightMm": 0,
      "highestMm": 0
    },
    {
      "time": "2026-10-30T04:10:53.032721000+0800",
      "event": "hump peak",
      "detail": 0,
      "humps": 1,
      "rejected": 0,
      "heightMm": 11,
      "highestMm": 11
    }
  ],
  "lastIMU": {
    "stateName": "Idle",
    "humps": 1,
    "rejected": 0,
    "heightMm": 11,
    "highestMm": 11,
    "lastFault": 0
  },
  "lastMotion": [
    {
      "time": "2026-10-30T04:11:12.942978000+0800",
      "uptime": 509484,
      "run": 3,
      "active": false,
      "recovery": 0,
      "speed": [
        0,
        0
      ],
      "target": [
        0,
        0
      ],
      "pwm": [
        0,
        0
      ],
      "distance": 345
    }
  ]
}
```
Review: testing/integration/logs/2026-10-30_b4_estimator_v0_2_ramp10_repeat_041012_review.json

## Stop attempt1: too early for assist cancellation
SDKrun4: manualStop49 at163mm/4216ms. Targets/PWM zero on first inactive telemetry frame; speeds later zero. No source15activation or end events, so ordinaryStop works but active-assistance cancellation remains untested. Do not count as an assist-stop pass.

## Stop during active assistance: verified (04:14)
User pressed dashboardStop atcrest. Source15detector2/estimator2: activation04:14:35.460; ended04:14:36.806 reasonSTOPPED after1342ms, alpha0 and addedPWM0/0. First inactive motion frame04:14:36.813 has targets0/0 andPWM0/0; measuredwheel speeds settle0/0 later. ManualStopreason49 at213mm/5111ms, encoderinvalid0/0. Source4 partialprofile rejected detail7=STOPPED, humps0/rejected1, eventheight13mm. Expected aborted profile: nofalseacceptedheight. PASS for commanded-output cancellation; precisephysicalstoppinglatency notindependently measured.
Evidence: testing/integration/logs/2026-10-30_b4_estimator_v0_2_stop_assist_repeat_041347_review.json.

Current new-estimator powered evidence: completed10mm ramp accepted10mm and11mm,2/2unaided; active-assistStop verified; no encodererrors or linerecovery in these two completedramps. Intermittentflatline-following issue remains unresolved and reproduced on priorfirmware. Furtherheights/repeats/fullmission are not claimedvalidated. No firmwarechange required by these latesttests.

## Powered speed sanity check:400mm /11.05s (approximate stopwatch)
User reports middle stall. Physical average36.20mm/s, including stall. LatestSDKrun6 reports404mm/11309ms, average35.72mm/s: distance+4mm(+1.0%), average about1.3% lower. Stopwatch first-motion/physical-end-marker interval differs slightly from SDKStart/processedStop interval; this is approximate agreement, not certified1.3%accuracy. No encoderinvalidcounts; maxcontrolgap13624us. Brief recovery1 at282mm, back0 at287mm; another recovery1 near403mm. Summary stopreason11 preserved without assuming dashboardStop. The stall remains an unresolved motion/line-following issue; it does not invalidate elapsed-average arithmetic. No clean steady-speed response validation is claimed.
Evidence: testing/integration/logs/2026-10-30_powered_speed_400mm_042410_review.json.

## Barcode Y regression: no read, stutter (04:27)
User reports stutter/noYdecode. SDKrun8:245mm/6577ms, manualStop49, recovery0/1 (briefwhitewaitaround211mm), encoderinvalid0/0. LivebarcodeADC168..2697, selectedthreshold212/bank11; decoded0,badframes0,unsupported0, maxADCgap6685us/maxcontrolgap13183us, TXdrops5318. Neither goodsignalrange nor zero badframes proves a complete barcode scan. Source10samples show severaldarkbands, but20Hzdata cannot recover individualbar widths or decide why frame acquisition failed. No source10nonperiodic event or decodedletter recorded. Preserve currenthardware/calibration; do not attribute solely to stutter.
Review: testing/integration/logs/2026-10-30_barcode_Y_lower_042823_review.json.
Next discriminating comparison: exact originalv2.3 baseline WITHOUT Buddy4, samehardware/calibration, oneYpass. Earlier old-vs-new estimator comparisons both includedBuddy4, so they did not exclude add-on timing as a cause.

## Original baseline Y comparison and stored raw replay
User reports less stuttering but still no decode. Latest SDK summary:299mm/7436ms, manualStop49; no recovery reported, encoderinvalid0/0; selectedthreshold212/bank11, decoded0,badframes0,unsupported0; maxADCgap4724us/control11499us. Livecontrast167..1913. Both with-Buddy4 and without-Buddy4 builds failY, so Buddy4 is not necessary for failure; contribution to roughness/sampling margin not excluded.

Read-only MQTT GET_LOG(dataset2) retrieved2048 stored ADC/encoder-distance samples; no motor/configuration commands. Capture covers0..5112.636ms and0..211.03mm of the299mm run; later samples are not stored. Replayed this prefix through original v2.3 barcode_analog/reader across all63threshold banks. No complete frame/candidate/selection. Thresholds200..210 each found one asterisk-like start pattern; threshold200 candidate spans148.00..194.32mm, at4776.8ms. Default212 found no start; coarse512/1024/1536 see12edges, while fine200..210 see24..26. This suggests fine pattern/edge acquisition needs checking, but incomplete raw coverage and no verified barcode artwork prevent a definitive physical cause. Do not label it purely a stutter, threshold, IMU or print problem. No firmware change made.

Files: logs/2026-10-30_barcode_Y_original_stored_raw.csv +.json; logs/2026-10-30_barcode_Y_original_raw_replay.txt; integration/tools/replay_stored_barcode.c (offline diagnostic only). Next input: actual Y barcode artwork/file to verify expected Code39*Y* elements and dimensions before further physical trials or decoder changes.

## Z acquisition diagnosis: print dimensions accepted; software sensitivity lead
User confirms barcode dimensions are correct and Z also fails; do not request dimension verification again. Latest originalbaseline run2:249mm/5698ms, decodedtotal0,lastletter-,noencoderinvalids,maxADCgap4736us. Read-only GET_LOG retrieved2048 Z rawsamples covering0..5031.679ms/0..221.20mm. Original63-bank decoder replay produced0candidates/0selections.

A diagnostic sweep retained the original Code39 reader, width checks, debounce2ms and all character matching, without filtering for an expected letter. Sweeping threshold160..300 in1ADC steps and hysteresis0/1/2/3/8/16 recovered exactlyZ twice: threshold204/hysteresis1, frame35.23..173.30mm/reverse; threshold205/hysteresis1, frame35.23..173.22mm/reverse. These are neighbouring thresholds agreeing on completeframe/direction/span. Originalfinebanks use2ADC spacing and hysteresis3; that combination does not recover this Z trace. This is a concrete software acquisition sensitivity issue in the captured scan, not proof of full hardware robustness at lowerhysteresis. No expectedbarcodebars were inferred or manufactured. Y still produces0completeframes under the same expanded diagnostic search; no universalfix is claimed.

No firmware was changed. A separately versioned barcode candidate retaining legacy thresholds/validation plus fine1ADC/hysteresis1 banks is the next targeted software experiment; measure RAM/CPU overhead and falsepositive protection. Do not weaken Code39 width checks or accept one isolated threshold. Existing Buddy4 estimator fix and calibrated motor code should be preserved.
Artifacts: logs/2026-10-30_barcode_Z_original_stored_raw.csv/.json; barcode_Z_original_replay.txt; Y/Z_fine_probe.txt; integration/tools/barcode_fine_probe.c.

## Control correction: exact release binary now prepared
User proposed testing the known-working artifact at commit8d7f3aa4935565636bba013c74e563f5e8135bcb. Previous files labelled originalbaseline were LOCAL REBUILDS of185 matching sourcefiles, SHA f846cd5eae41d4d40500d735aeab94c3a1044573f7813407cb244cfc29797a58. This does not establish behaviour of the exact shippedUF2. Do not treat prior failedY/Z runs as physical tests of the release artifact.
Extracted firmware/buddy123_interface_wifi_v2_3.uf2 directly from that Gitcommit without rebuilding. Verified pinned SHA256SUMS match cceab279c3dd99b1548e65a67bf5764fcffddf4c288dc5175d3fbdb4d15bdf5e;1009152bytes;UF2blocks/family/magic checked. Saved outsideGit as bench_tools/buddy2/buddy123_interface_wifi_v2_3_original_8d7f3aa.uf2 with manifest. Not flashed by agent. Normalflash preserves WCF1 Wi-Fi config; keep currenthardware/calibration/profile1. Next: user manuallyflash exactbinary and testY/Z before decoder changes. Source15estimator/detector packets absent bydesign. Existing offlineZsensitivity finding remains diagnostic evidence from local rebuild, not a universalfirmwarefix.

## Exact released UF2 physical comparison completed
User reports no barcode frames/letters after two driven attempts on the requested exactrelease artifact. Native summaries:run1=326mm/8757ms at05:04:18.918, run2=276mm/7063ms at05:05:11.408, bothmanualStop49. Latest run2:141 active motion frames, barcodeADC167..2746, recovery0; latestdecoderdiagnostics decoded0,badframes0,unsupported0,encoderinvalid0/0,maxADCgap4585us/control10785us. LiveSDKtelemetry continues; this is barcodeacquisition failure, not missingMQTTtelemetry. Latest screenshot/ack doesnot establish framing. This reproduces no-decode on exactpublishedbinary as well as localrebuild/add-on; those changes are not necessary forfailure. Prior earlystartreason5 wasUSBhostinterlock and is separate from these valid driven runs. Barcodeartwork dimensions accepted peruser; do not request dimension checks. No additional hardware run proposed before a softwareacquisition sensitivity experiment. Current physicalclaim remains no-decode; firmwarecodeunchangedthisreview.
Evidence: logs/2026-10-30_exact_release_barcode_attempt_050557_review.json.

## Barcode fine v0.1 candidate built — hardware pending
Isolated SDK/noBuddy4 build retains original63 banks and adds204/205ADC,hysteresis1 pair. RecordedZ decodes through actual original reader+consensus;Y remains unresolved. Original35C+JS/Python and new ASan/UBSan tests pass, ARM0warnings/errors. No motor/line-control change. See subsystem2/interface_wifi_v2_3_barcode_fine_v0_1/BARCODE_FINE_V0_1.md and integration/logs/2026-10-30_barcode_fine_v0_1_*.txt. No hardware pass claimed.

## 2026-10-30 barcode fine v0.2 reporting repair (host only)

Physical v0.1 Z attempt: no reported letter; source14 diagnostics suppressed by reserved-bit packer rejection. Fixed flags, bank63 codec/decoder bound and added periodic STATUS detail0xbc02 build tag. No change to detection or motion. Host/ARM checks pass; hardware pending. See `testing/subsystem2/interface_wifi_v2_3_barcode_fine_v0_2/BARCODE_FINE_V0_2.md` and logs `2026-10-30_barcode_fine_v0_2_*`. Firmware SHA256 `b11ca752b2ddad4660e7733e80ecec01a389dd76c78a35c2fd1508dfee7c31db`; not flashed by agent.

## 2026-10-30 barcode fine v0.2 physical check, latest run3

Build tag source11 STATUS detail0xbc02 confirmed; fresh source14 diagnostics now arrive. Latest run3 stopped by MQTT (reason49),230mm in5455ms. No letter/pending movement; decoded/bad/unsupported counters all0. Barcode ADC168..2724, recovery states0 only, invalid encoders0/0. Two low-speed frames occur at start (distance0..1mm), not evidence of a midway stall. No change made from this result; fetching stopped-only stored raw data for diagnosis. Evidence: `logs/2026-10-30_barcode_fine_v0_2_latest_0533_review.json`.

Raw run3 saved:2048 rows,0..219.85mm, through5111.699ms of5455ms run. Actual v0.2 selector replay:0candidates/0selections. Unchanged reader diagnostic sweep recovers Z only at threshold205/hysteresis2,35.46..172.43mm, reverse. This isolated match does not meet adjacent-bank consensus. The new h1 pair has no complete candidate. Points to fragile acquisition/threshold margin, not a dashboard letter-display or MQTT delivery failure. No additional firmware changed. Files: `logs/2026-10-30_barcode_Z_fine_v0_2_run3_{stored_raw.csv,replay.txt,probe.txt}`.

## 2026-10-30 barcode fine v0.3 — host fix, hardware pending

Latest failed Z yields one complete205ADC/hysteresis2 frame, blocked by neighbouring-bank consensus. Added measured leading/trailing quiet-zone fallback with conflicting-frame veto; fine family odd193..255/hysteresis2. Both saved Z scans decode; Y unresolved.36 C executables, schema/Python and ASan/UBSan checks pass, zero-warning ARM build. No motor/control changes. See `testing/subsystem2/interface_wifi_v2_3_barcode_fine_v0_3/BARCODE_FINE_V0_3.md`. Local UF2 SHA256 `57fdb5ceb7538bd530eca48122407dc0cb734a821d1cfbcf113709190f2b6643`. No agent flashing/driving; hardware pending.

## 2026-10-30 barcode fine v0.3 physical run1 — still failed

Version0xbc03 confirmed. User reports little stuttering.232mm/5344ms, recovery0, invalid0/0, barcodeADC169..2154, decoded0.2048 raw samples saved,0..222.51mm/5104.406ms. Real v0.3 replay has0candidates; diagnostic original-reader sweep160..300 ADC and hysteresis0/1/2/3/8/16 has no full-frame match. This run is not being blocked merely by quiet-zone/consensus gates. v0.3 is not physically validated as a fix. Need isolate acquisition/optics before further threshold tuning; no firmware change made from this run. See `logs/2026-10-30_barcode_Z_fine_v0_3_run1_*` and reviewJSON.

## 2026-10-30 07:10 — barcode peak decoder v0.1 build (original v2.3 + peak decoder), not yet bench-tested

- Separate barcode-only build: `testing/subsystem2/interface_wifi_v2_3_barcode_peak_v0_1/` (original 8d7f3aa tree, no Buddy4/ramp assist). Bars read from ADC peak positions; spaces from bar-centre spacing in frame units; strict `*X*` acceptance. Original 63-bank decoder unchanged alongside.
- Host: all four failed captures (Z original, Z fine v0.2 run3, Z fine v0.3 run1, Y original) decode correctly in the C streaming code, also with ±10 % odometry scale, half sample rate and ±15 ADC offset; synthetic 930/936 with 0 misreads; harsh 0 misreads (20,000-trial offline run also 0); negatives pass; run_tests.py 35 pass.
- Firmware (local only): `bench_tools/buddy4/buddy123_interface_wifi_v2_3_barcode_peak_v0_1_{shadow,active}.uf2` (shadow 5eef23d7…, active 39123c50…), 0 warnings. Source-15 subtype 0xB1 telemetry; decode with `barcode_peak_decode.py`.
- Bench result: pending (shadow label runs, then active). Handoff: `claude_output/HANDOFF_2026-10-30_BARCODE_PEAK_V0_1.md`.

## 2026-10-30 ~22:40 — barcode peak v0.1 ACTIVE on the car: user reports all labels decoded

- Firmware: `buddy123_interface_wifi_v2_3_barcode_peak_v0_1_active.uf2` (39123c50…), lower bracket, no other physical change.
- User report: "all decoded". Captures not yet reviewed here; repeatability, other letters and junction actions still to be recorded from native logs (source 15 via `barcode_peak_decode.py`).

## 2026-10-30 23:15 — combined build v0.1 (v2.3 + Buddy4 v0.2 + ramp assist v0.2 + barcode peak v0.1), host-tested only

- Folder `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_1/` (base `interface_wifi_v2_3/` unchanged; peak decoder byte-identical). main.c +3 hook lines, CMake option `BARCODE_PEAK_ACTIVE`; drive, Buddy4, ramp assist, original reader/cross logic and MQTT mapping unchanged.
- Coexistence: same-letter duplicates suppressed (state bit-identical to original alone); conflicting letters on one label cancel the pending movement in either order; no-barcode runs leave navigation bit-identical.
- Source 15: ramp (payload 28) and peak (payload 24, subtype 0xB1) dispatched by length/subtype in the dashboard and `source15_decode.py`; combined-build ID = peak flags bit7 + run-start announce (reason 10).
- Dashboard v0.3: source-15 cards; "Last decoded letter (B3)" verified to update from a real SDK decode via the real C packers.
- Host: `run_combined_tests.sh` pass (cloud clang/gcc, Mac VM gcc); dashboard browser checks 13/13 + 44/44. ARM: 0 warnings, bss +2,036 B, main stack 2,368 B, barcode_peak_sample 592 B.
- Firmware (local only): `bench_tools/buddy4/buddy1234_peak_v0_1_ra_active_bp_active.uf2`, sha256 26910f823893797e4fca3e5f68476765540ed64f1e5d88695bd7714bbbe4cd75 (ramp ACTIVE, peak ACTIVE).
- Hardware: pending. Steps in `B4_PEAK_V0_1.md`; handoff `claude_output/HANDOFF_2026-10-30_BUDDY1234_PEAK_V0_1.md`.

## 2026-10-31 00:39 — combined v0.1 on the car: barcode OK; ramp run 20 analysis; combined v0.2 built (host only)

- Earlier capture 00:18–00:20: Buddy4 in Fault 3 (calibration timeout) throughout → tilt invalid, no assist, no hump possible. Re-Reset fixed it.
- Run 20 (00:39:20, 10 mm ramp): assist active at 171 mm (+30/+28); crest 216–219 mm speed → 0 for ~0.15 s; Buddy4 quality reasons ACCEL/SPREAD made tilt invalid in 60 ms bursts → assist ended INVALID 4×; integrals saturated, surge to ~80 mm/s; hump rejected detail 3 (peak 13 mm). No barcode seen; not a peak-decoder effect. Control gap 10.6–11.6 ms (unchanged).
- Combined v0.2 (`interface_wifi_v2_3_b4_peak_v0_2/`, ramp assist v0.3): hold assist up to 150 ms through ACCEL/SPREAD-only exclusions; everything else unchanged. Host tests pass. Firmware `bench_tools/buddy4/buddy1234_peak_v0_2_ra_active_bp_active.uf2` sha256 e16d2f100953dbcc2e6694051c539ce7edfe3cc7076d56aeffc5cbcf61e09c94. Hardware pending.

## 2026-10-31 01:21 — v0.1 ramp stall at 201 mm; combined v0.3 (stall push) built, host only

- Run at 01:21 (v0.1 firmware, detector 2): stopped at 201 mm with right wheel up (tilt 8–11 mm), front settling onto mat (barcode ADC 430→1,230), ~20 s, output 295/165 PWM (limit 420; ff+error+I_max, steering target right 13 mm/s); assist chattered INVALID then timed out.
- Combined v0.3 (`interface_wifi_v2_3_b4_peak_v0_3/`, ramp assist v0.4): v0.2 hold + stall push (+60 PWM both wheels after 200 ms at < 5 mm/s while assisting; released > 15 mm/s; 1.5 s budget, excluded from the 2.5 s timeout; Stop/Pause immediate; 420 clamp). Host model reproduces 295/165 and gives 385/235 with assist + push. Firmware `buddy1234_peak_v0_3_ra_active_bp_active.uf2` sha256 bc376eeca37798f410523a84b6888a60c808cc480314ff5b93dad2799d2447b5. Hardware pending.

## 2026-10-31 01:49–01:51 — v0.3 with bracket tilted up: 4/5 ramps cleared; combined v0.4 (tilt floor) built, host only

- Runs 01:49:11, :29, :50, 01:50:51 cleared; humps accepted 16/14/14/14 mm. Run 01:50:15 stuck at 225 mm after two successful pushes: left 295 PWM, ramp (right) wheel 12–51 PWM because steering targeted 2–9 mm/s. Flat speed today 46–47 mm/s (yesterday 44–48; same 45 target); right wheel needs ~223 PWM on the flat vs ~170–200 yesterday (possible extra drag on the right side).
- Combined v0.4 (`interface_wifi_v2_3_b4_peak_v0_4/`, ramp assist v0.5): each wheel ≥ 300 PWM from assist activation until flat (5 s cap; Stop/Pause immediate; no integral wind). Firmware `buddy1234_peak_v0_4_ra_active_bp_active.uf2` sha256 d5bcd275a4930ec93ee44736f7c87c588cd1ed3d5e29240f3aa00c334fc04271. Hardware pending.

## 2026-10-31 02:06–02:08 — combined v0.4 (300 PWM tilt floor): climbs too fast, stalls at the crest; combined v0.5 (speed hold) built, host only

- Capture `logs/2026-10-31_buddy1234_peak_v0_4_capture.txt`: 2 flat runs, 4 ramp runs (02:07:27, :54, 02:08:13, :43). Each: floor 300/300 → 107–131 mm/s on the climb; ~0.47 s in, non-ACCEL/SPREAD invalid tilt ended assist + floor (single use); car stopped at the crest (193–216 mm, barcode ADC ~1,000–1,100 = nose on the mat); re-activation without floor, ramp wheel 0–25 PWM (steering 0–3 mm/s) until the stall push ~1 s later. Buddy4 rejected every hump (detail 3, 7–12 mm).
- Combined v0.5 (`interface_wifi_v2_3_b4_peak_v0_5/`, ramp assist v0.6): floor removed; until flat 200 ms the speed loop gets min 35 mm/s per wheel, I limit 300, 3× Ki; survives invalid < 500 ms. Firmware `buddy1234_peak_v0_5_ra_active_bp_active.uf2` sha256 dacc4e768ccbcdfa7485ec410e1b2ec4defff48c60b3ab65584292ac66a15118. Hardware pending.

## 2026-10-31 02:26–02:38 — combined v0.5 (ramp assist v0.6 speed hold): ramp and barcode working

- Excerpt `logs/2026-10-31_combined_v0_5_speed_hold_runs_native_excerpt.txt`. Ramp runs 02:26:27, 02:27:42, 02:33:11: detected (detector 6), crossed with no stall and no stall push (slowest 22–31 mm/s at the crest, front touching the mat), Buddy4 hump accepted 12 / 11 / 13 mm. Climb peak ~60 mm/s (v0.4 floor: 107–131). Barcode run 02:38:44: peak decoder read Y (reverse), unmapped so no movement. User reports further runs OK.
- Defaults in `CMakeLists.txt` set to the tested configuration (ramp assist ON, peak decoder ON); the default build reproduces UF2 sha256 dacc4e76…5118.
