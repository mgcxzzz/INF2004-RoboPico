# V16: coverage-based line following

Built2026-10-26, Pico W/RP2040, SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, C11 -Wall -Wextra -Werror.

- Artifact firmware/buddy23_circuit_with_barcode_v16.uf2:242176 bytes,
 473 sequential RP2040 blocks; padded end0x1001d900.
- SHA256:df5c35ff9e0b791db0e6953352919f6156bcef3165806d35331f9e21704c002c
- Linker end0x1001d8f4; text125168/BSS215692; BSS end0x20035dcc,
 stack limit0x20040000, gap41524 bytes.
- Saved record64848 bytes, trace56/summary40 and flash banks unchanged.
 V1..V16 records load with historical CRC lengths; unknown17 rejects.
 New decision codes fit the existing5-bit field. Coverage state is RAM only.
- UF2 family/sequence/address/trailer/markers/build equality passed;
 published V3..V15 hashes unchanged. Published straight probe V1 is unchanged.

## Reason for the change

The actual fixed-speed probe was visually smooth; after1.5s its logged wheel
speeds averaged44.43/43.82mm/s, PWM149..166/166..180, without maximum restart
pulses. Changing motor control again was not justified. Stationary left/right
tests, the underside photo and the user's partial-coverage measurements
exposed a mismatch in the equal-darkness steering assumption. Around800..1000
raw ADC still contains a line return but normalises to only188..249 with the
school calibration. The V15 paired hold demanded700 normalised instead.

V16 opts into CIRCUIT_COVERAGE_FOLLOW. coverage_state.h tracks per-sensor white
confidence/hysteresis and useful coverage. coverage_follow.h chooses targets
by coverage rather than black-strength difference. It is called by the same
ordinary/outgoing-alignment path and reuses existing direction confidence,
short-gap memory and bounded stationary search. Right-hand mirror conversion
swaps the added per-sensor confidence state. Builds without the flag retain
the previous follower path; the straight probe pins its record label to15.

Both>=180 enters45/45; both>=120 retains it. Initial weaker dual coverage
requests30/30. One approaching white requests35/50; white<=60 confirmed30ms
requests25/50; sustained white120ms from first low sample requests0/45.
White clears at>=120 for20ms. Either useful return can end loss after20ms.
Both confirmed white enters loss; both uncertain below120 for80ms also enters
the same bounded path, avoiding an indefinite grey-band coast. The existing
1.5s search wait starts after qualification. Pending route commands remain
protected while lost. Raw front levels, stored references, decoder thresholds,
planned-turn branch detection and normal motor-control implementation remain.

The V15 brightness-balancing and additional PWM slew adapters are bypassed.
Motor PI/feedforward/breakaway/power-source files were compared against
V12(f0e3b35) and match, as in V14 and the probe. No global PWM increase or new
motor controller was introduced. Gentle/firm tiers intentionally change bend
response and still require a physical course trial.

## Executed tests

With the PRODUCTION coverage flag enabled:

- test_coverage_follow: both actual3000/800..1000 ADC orientations request
45/45; raw levels preserved; coverage/white hysteresis;30/120ms boundaries;
 gentle/firm/hard mirrored outputs; transient white flash/reversal; low-band
 bounded loss/search;20ms candidate return; pending search stop; mirrored
 outgoing left/right alignment and invalid-cal rejection.
- test_barcode: generic Code39, historical A captures, queued routes/CRC and
12 alternating A/F turns per profile. Record ABI remains64848/56/40.
- test_analog,test_barcode_sensitivity: A/F/Z frames both sizes/directions,
 low contrast, missing bars/ambiguity rejection, qualified cross and routing.
- test_uturn: complete analogue Z, original-line geometry, lagged wheel
 response, angle/time/drift/settle limits, next cross after reverse and V1..V16
 saved-record integrity. Only outgoing coverage interpretation changes.
- test_start:192 non-optical guard combinations; new white qualification
 followed by bounded no-history search. Test timing accounts for the30ms
 optical qualification instead of assuming immediate loss.
- test_front_calibration,test_barcode_raw,test_maneuver_power,
 test_ramp_profile passed, retaining calibration/records/output envelopes.

Without the coverage flag, test_line_search and test_smooth_follow passed,
checking that opt-in shared-header changes preserve their historical paths.
Full Pico W production compilation completed with -Werror.

Physical straight/bend/barcode success is not claimed. Keep profile1 and the
existing calibration. Collect a short straight's p, then test a bend and one
barcode before drawing conclusions about optical tolerance or scan consistency.
