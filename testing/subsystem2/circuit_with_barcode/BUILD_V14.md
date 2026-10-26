# Circuit with barcode V14: restore V12 tracing/controller

Built2026-10-26 for Pico W/RP2040, Pico SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release; C11 -Wall -Wextra -Werror.

- Artifact firmware/buddy23_circuit_with_barcode_v14.uf2:245248 bytes,
 479 sequential RP2040 blocks; padded end0x1001df00.
- SHA256:fbbd53531319f76d56f67a23ac5e14b177367ed71d568bd1a6acba12d88accef
- Linker end0x1001de4c. ELF text126536/BSS215544; BSS end0x20035d38,
 stack limit0x20040000, leaving41672 bytes between these symbols.
- Record remains64848 bytes; historical64840-byte prefix and trace56/summary40
 unchanged. Versions1..14 load; unknown15 rejects. Raw capture/banks unchanged.
- UF2 sequence/family/addresses/trailers/markers/package equality pass;
 published V3..V13 hashes unchanged.

## Evidence and change

User reported materially worse V13 following. The run ended26.291s at the
stationary search endpoint, not because invalid encoders were detected.
Visible tracking repeatedly showed high black levels while low-speed wheels
stopped. The raw/calibrated readings do not support blaming calibration alone.
The new V13 controller/large-error smoothing were the wrong physical tradeoff
for this run; they are removed from the active firmware.

Supplied calibration:white176/175,black3483/3498,span3307/3323,valid1.
Fully covered worn-section samples:3244/2752,3238/2763,3238/2766,3234/2774,
3231/2787. Levels left923..927/right775..786 clearly register black. No line
thresholds, calibration algorithm or stored references are changed.

smooth_follow.h and test_smooth_follow.c are restored from V12(f0e3b35).
main.c no longer includes visible_wheel.h or calls visible_wheel_step, and
normal motion again uses circuit_wheel_step, its breakaway, load/capture
assistance and V12 transitions. The historical V13 helper/test remain on disk
for reference but are outside the production include graph.

User's stationary recovery remains V13:1.5s white qualification; counter-rotation
at100mm/s,25 near35/60/90/120/150deg alternating endpoints; measured-still
reversals; stop-on-sighting confidence;600 sustained/750 bounded start;12s,
1000mm accumulated wheel travel,30mm encoder centre-drift limits. Z fast
turning and A/F decoding are unchanged. No physical success is claimed.

## Executed verification

- Text parity against V12(f0e3b35), normalized only for line endings:
 smooth_follow.h,circuit_wheel.h,signed_wheel.h,boosted_wheel.h,
 wheel_control.h,dual_logic.h,symmetric_logic.h. Production wiring checked to
 call circuit_wheel_step and exclude the V13 visible controller.
- test_school_restore: all five actual worn-black samples remain recognized
 without entering loss; a single clear front return exits white recovery;
 both wheels retain V12's300-PWM breakaway at a stopped45mm/s command;
 deliberate zero remains OFF. These are software checks, not measured motion.
- test_smooth_follow: V12 small-error attenuation, sustained bends, immediate
 strong-error bypass, recovery/stop/junction isolation, weak boundary, profiles.
- test_line_search:1.5s qualification, mirrored encoder-angle endpoints,
 settled reversals, no-tick/time/travel/drift bounds, opposite-sensor return,
 pending command, right-hand mirror and alignment budget.
- test_uturn: analogue Z both sizes/directions; original-line geometry,
 delayed wheel response; missing-return/time/drift/settle checks; signed
 mapping, next junction after reverse and record integrity V1..V14.
- test_barcode,test_analog,test_barcode_sensitivity,test_maneuver_power,
 test_ramp_profile,test_start(192 cases),test_front_calibration and
 test_barcode_raw all passed. Full Pico W build completed successfully.

Start with profile1 and unchanged calibration. Stop immediately after the
short straight/first problematic bend and collect p so its motion is retained.
Then test one barcode and collect j. Reassess physical behaviour before further
calibration or controller changes. V14 restores the baseline; it does not
claim to solve every V12 oscillation or stationary-search geometry limitation.
