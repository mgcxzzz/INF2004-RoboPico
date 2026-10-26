# Circuit with barcode V15: small corrections only

Built2026-10-26, Pico W/RP2040, SDK2.2.0, ARM GCC14.2.1,
CMake/Ninja Release, C11 -Wall -Wextra -Werror.

- Artifact firmware/buddy23_circuit_with_barcode_v15.uf2:247808 bytes,
 484 sequential RP2040 UF2 blocks; padded end0x1001e400.
- SHA256:42143adba24e31670ae145cb53d3145ad2b4991e7d2c0834ab66b416b4fbf1b2
- Linker end0x1001e384; text127872/BSS215548. BSS end0x20035d3c,
 stack limit0x20040000, gap41668 bytes.
- Record64848 bytes/trace56/summary40 unchanged. V1..V15 load; unknown16
 rejects. Decision24 fits existing5-bit field; no record or flash-bank growth.
- UF2 family/sequence/addresses/trailers/markers/package equality pass;
 published V3..V14 hashes unchanged.

## Change and scope

User physically confirmed V14 completed straights/bends better, but requested
less straight-line jerking and more gradual speed/PWM adjustments. Supplied
run stopped manually at59.458s,1698mm,28.56mm/s average; no invalid encoders.
Its retained motion window is40.615..60.445s, including settling. No barcode
decode or junction turn occurred. Calibration stays white176/175,
black3483/3498. Small changes in strong black strength can cause avoidable
asymmetric targets; the log also contains genuine edge/loss observations.

V15 qualifies strong paired-black readings for100ms(both>=700,difference<=220),
then requests45/45 rather than following minor brightness imbalance. Existing
60..20ms V14 smoothing brings targets to equality. It releases immediately
when either<600 or difference>300; V14 urgent/near-pivot bypass remains.
Raw optical confidence/loss/barcode/junction inputs are unmodified. Equal
90/135/180 fast-cruise requests are preserved. Decision24 identifies the hold.

straight_pwm.h is a post-controller adapter, not a new speed PI. It limits
positive output changes to+800/-1600 PWM units/s ONLY during paired hold,
with both actual wheel speeds>=25 and both targets35..55. It bypasses startup,
underspeed, zero/coast, special ramp/turn/search, loaded effort and capture.
Actual previous output seeds each update; no separate latent output state.
Normal motor-controller dependencies are unchanged from V14/V12. The failed
V13 visible-wheel controller remains excluded. Stationary search and fast
U-turn are unchanged from V14.

## Executed validation

- test_straight_hold:100ms qualification, asymmetric brightness tolerance,
 hysteretic hold, immediate edge/imbalance release, mirrored urgent bends,
 loss/reset, fast-profile targets, phase/loaded/start/slow/stop isolation,
 and exact positive-PWM step limits. Strong-bend targets remain immediate.
- test_smooth_follow and test_school_restore: V14 attenuation/strong-bend
 bypass, faint boundary/profiles, actual worn-black samples, one-sensor
 reacquisition,300-PWM breakaway and deliberate zero OFF.
- test_line_search,test_uturn: stationary mirrored angle searches, settled
 reversals, bounds, optical return, pending command, Z frames/geometry,
 delayed wheel response and saved-record integrity V1..V15.
- test_barcode,test_analog,test_barcode_sensitivity,test_maneuver_power,
 test_ramp_profile,test_start(192 cases),test_front_calibration,
 test_barcode_raw passed. Pico W build completed with no compiler warnings.
- Updated optional replay_visible_log on925 retained visible rows:58 entries
 to PWM300 from below250,30 optical side reversals.397 rows qualified for
 paired hold;276 also passed rolling-PWM eligibility. Target total variation
2683 recorded vs2601 updated(3.1% less). Recorded-input replay does not model
 the new physical path, prove which samples were straight, or predict decode
 consistency. The user's input file is not committed.

The tolerance can change initial correction on a gentle curve despite
preserving urgent bend authority. Physical straight-to-bend verification is
required. Keep calibration and profile1; test a short straight and bend,
collect p promptly, then one A barcode and j. This does not claim to eliminate
all friction-driven kicks or guarantee successful barcode scans.
