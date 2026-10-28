# Circuit With Barcode V12.2 — smooth optical following + faster clear-straight cruise

[Download buddy23_circuit_with_barcode_v12_2.uf2](firmware/buddy23_circuit_with_barcode_v12_2.uf2).

This version starts from the isolated V12 `f0e3b35` source, retires the V12.1 heading lock completely, and replaces the visible-line output filter. The original optical follower, real ADC calibration/loss/recovery, generic Code39 decoder, junction geometry, A/F/Z manoeuvres and motor PI/PWM caps remain. `barcode_nav.h` is the exact V12 version, with no steering override hook or synthetic ADC path.

## First test

1. Flash and check `h` says **v12.2**. Existing valid front calibration imports from saved versions1–17. Select `1`, then `k`. No centre calibration is needed. Verify the CAL line; the last user references were white164/168, black3514/3389, not hardcoded.
2. First run a straight and a bend, then scan the current140mm A barcode and approach its separate cross. The front sensors continue steering at every point, including barcode immediately after a bend.
3. Start normally: USB `g`=2s hands-free; `f`=hold GP21. Power bank=boot wait5s, both buttons3s, release, wait1s, tap GP21,2s countdown. Either button or `s` stops.
4. Stop and wait5s before changing power. Send `j` for scan events/decode/junction result. If motion is still jerky, send `p` as a file from a short run before the1000-row ring overwrites the relevant interval.

## Behaviour

- Profile1 requests up to60mm/s on well-visible straights, compared with45 previously. The boost requires both front levels>=200normalised, and fades out as imbalance grows from150 to350. Full boost applies to small imbalance<=150. It is not an unconditional speed increase on bends or white.
- A possible barcode enters the gentler45mm/s mode immediately from existing reader activity, a pending command, or A0>=run-start baseline+16. There is no300ms straightness gate and no captured approach heading. A600ms release dwell prevents mode chatter between bars; original reader history can retain scan mode longer. Noise can slow cruising but cannot queue A/F/Z.
- Small differential corrections use120ms response normally,180ms while scanning. Gain is45% of V12 for tiny errors, blending back to full demand by350. Both the common forward-speed target and differential steering target are filtered. Increasing error shortens response towards25ms and raises steering slew authority. This avoids filtering a bend like small random noise.
- Reference acceleration<=80mm/s^2 and deceleration<=240mm/s^2 in the new adapter, followed by the unchanged wheel controller. A1mm/s hysteretic output quantiser avoids adjacent-integer chatter; settled outputs may be within1mm/s of the nominal target.
- Clear departure (normalised difference>=650 or V12 inner target<=10 with a strong line) returns the original optical demand immediately. Loss/search/stop, planned turns and outgoing-line acquisition bypass this new filter. Thus strong corrections can still be abrupt when the robot is actually leaving the stripe.
- Scan transitions cap a retained high-speed output to50mm/s while settling towards45. This prevents higher cruising from carrying into the barcode. Existing motor reference ramps remain active.
- Motor PWM/PI and breakaway, rolling2s search, turn/U-turn power and saved log fields remain V12. Profile2 remains the180mm/s ramp experiment and should not be used for this test. Profile3 retains135mm/s maximum when qualified.
- Generic full-frame decoding and two-neighbour threshold agreement remain. Unsupported/ambiguous patterns never force A/F/Z. Junction centring uses the existing92mm side-ahead measurement and55mm early rolling-turn start.

## Logs, calibration and limits

Motion24=gentle scan,25=gentle follow. Strong departure retains12/13. SCAN_SOFT_BEGIN/END width shows forward mean-wheel travel since scan-mode entry; an END with flags1 means planned junction handling took over. These events do not mean a valid barcode: check DECODE and decoded_A/F/Z counters. Decoder, raw `a`, motion `p`, events `j`, live `v` and all start controls remain.

Stored record version18 is displayed as v12.2;17 is displayed as v12.1. Layout/CRC/calibration remain compatible. An older V12 binary cannot import a newly saved version18 record; note CAL references before downgrading. Flashing this firmware is not proof of improved physical tracking, higher achieved speed or reliable scans.

BASELINE.json records45 original V12 source/test/include hashes; BASELINE_PATCHES.json lists five intentional snapshot edits (main integration/help, event IDs, record acceptance and two version tests). scan_smooth.h contains the new algorithm. All other baseline headers, including barcode_nav and motor controllers, are exact V12. Run run_tests.py and verify_firmware.py; see BUILD_V12_2.md for executed evidence.
