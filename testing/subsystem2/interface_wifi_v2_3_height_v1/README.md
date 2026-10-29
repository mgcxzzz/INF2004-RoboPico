# Wi-Fi V2.3_height_v1 - raised-height steering experiment

Based on the working SDK Wi-Fi V2.3 release at `8d7f3aa4935565636bba013c74e563f5e8135bcb`. Original V2.3 remains unchanged in its own folder. This variant is for the reported front mounting around 10 mm and barcode mounting around 11 mm. Its physical performance is not yet validated.

Download [firmware](firmware/buddy123_interface_wifi_v2_3_height_v1.uf2), [complete package](firmware/buddy123_interface_wifi_v2_3_height_v1_bundle.zip), and follow [SETUP.md](SETUP.md).

## Steering changes

Only ordinary visible-line output adaptation changes, in profiles 1/3:

- Brief turn authority starts at 45% (previously 70%), rising to full authority after 450 ms of continuous same-side request (previously 350 ms). Dwell follows the signed optical wheel request; it does not require equal sensor readings.
- Encoder damping continuously opposes measured half-wheel-speed difference exceeding the requested difference. Coefficient 0.65, bounded to 8 mm/s of half-speed correction. A sustained bend at or below its requested turning rate retains full authority.
- Brief visible-line demands use a mean-target floor of 35 mm/s before wheel ceilings are applied; the floor fades to the original requested mean as bend evidence persists. Short-turn half-speed is initially bounded to 12 mm/s, with full inner-wheel range restored for sustained bends. These are requested speeds, not guaranteed measured speeds.
- Existing 120..300 mm/s^2 half-speed slew bounds remain; an old correction releases using the existing 300 maximum when the requested magnitude shrinks or reverses. No additional ADC filter or timed steering lock is added.
- Lost-line, coast/zero, STOP, junction manoeuvres, external Buddy takeovers and profile 2 bypass this adaptation. Profile 1/3 limits and the 45 mm/s scanning wheel ceiling remain.

The upstream optical thresholds, line-loss/recovery decisions, barcode decoder and 63 threshold banks, continuous motor PI, PWM/start-aid limits and all motion/nav APIs are unchanged. Wi-Fi transport, packet schemas, dashboard, helpers, 11-slot lwIP timeout pool and two 8 KiB stacks are unchanged. Buddy 5's default extension remains empty. This is standalone Pico SDK firmware.

## Preloaded raised calibration

| Sensor | White ADC | Black ADC |
| --- | ---: | ---: |
| Left | 191 | 1940 |
| Right | 154 | 2306 |

These are the measured recalibration endpoints inferred from the 29 October run and confirmed by its ADC/level pairs. They are an evidence-based starting calibration, not a newly proven smooth-running sweet spot. No w/l/r/k capture is required for the first test.

Initial boot overrides older-version calibrations and selects profile 1. Only a CRC-valid saved **record 29** from this variant can override this preset later. Manual w/l/r/k remains available. While stopped and settled, `c` restores the preloaded raised values; `k` persists them. Restarting or normally reflashing the SAME variant preserves a custom record29 calibration. Export the previous run before flashing; older motion/raw/diagnostic captures are not imported as a new height-test run on the initial preset boot.

## Tests and limitations

37 C executables, 10 JavaScript schema tests and 20 Python helper tests pass. Tests cover brief corrections, persistent bend authority, mirrored/brightness-independent operation, continuous excess-yaw damping, STOP/recovery bypass and profile ceilings. On identical recorded inputs, old versus height target total variation is 883 versus 780, mean wheel target 31.04 versus 35.14 mm/s and minimum wheel target 6 versus 15 mm/s. This is a fixed-input adapter replay, not a vehicle simulation or evidence of physical smoothness/barcode success.

The real ARM ELF retains separate 8 KiB stacks and 4424 bytes of known driving-chain frames; the actual lwIP allocator test passes with 7+4 slots. No real robot was flashed or driven here. [TEST_LOG.md](TEST_LOG.md) gives evidence; [BUILD.md](BUILD.md) covers reproduction.

Diagnostic rows remain 58 bytes with the same field packing. This variant uses capture **format 5**: GAIN0..7 means 0..100%, DAMP0..3 means absolute half-speed damping 0..8 mm/s. Old format4 diagnostic banks are rejected rather than misinterpreted. Dashboard exports remain compatible raw-row downloads. Historical V1/V2 provenance files are retained; HEIGHT_BASELINE.json, HEIGHT_PATCH.json and SOURCE_SHA256.json are authoritative for this release.
