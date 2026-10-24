# Circuit with barcode V9: ordinary steering smoothing

Built 2026-10-25 for Pico W/RP2040 with Pico SDK 2.2.0, GNU Arm 14.2.1,
CMake/Ninja Release, C11 and -Wall -Wextra -Werror.

Artifact: firmware/buddy23_circuit_with_barcode_v9.uf2

- 215040 bytes; 420 sequential RP2040 UF2 blocks.
- SHA256: 1089a1feeec64c8d89ae86d57f6458749345a189b7e2daeee5138bb6a6da5238
- Padded image end 0x1001a400; linker end 0x1001a3c4.
- ELF text 111552, BSS 215208; BSS end 0x20035be8, stack limit 0x20040000.
  The gap between those symbols is 42008 bytes. Smoother adds 12 static bytes.
- Raw bank 0x101b0000 and all flash record addresses remain unchanged.
- Main saved record 64840 bytes, trace 56 and summary 40 remain unchanged.
  Versions 1-9 load; unknown 10 rejects. Raw capture format is unchanged.
- Packaged/build UF2 equality, addresses, family, sequence and embedded
  markers pass verification. Published V3-V8 artifact hashes are unchanged.

## Scope

smooth_follow.h wraps barcode_circuit_step in main.c. It modifies only
ordinary visible-line C_FOLLOW targets. Small balanced corrections blend
from 65% to 100% differential gain by 350/1000 error. A 60..20 ms filter
and 200..600 mm/s^2 rate bound progressively speed up as error grows.
The discontinuity between faint and normal target scaling at level 400 is
also smoothed. No additional filtering is applied to the raw optical levels,
the memory used by recovery, or the barcode ADC signal.

Error >=700, strong near-pivot demands, blind recovery, stop and every busy
junction phase bypass smoothing and reset its history. Leaving faster cruise
does not retain targets above the historical 50 mm/s correction ceiling.
Calibration, wheel PI, PWM caps and breakaway behavior are unchanged.

Barcode thresholds/validation, command mapping, junction geometry, rolling
turns and search schedules are unchanged. This does not implement heading
lock or an adaptive barcode reader. It isolates steering smoothness as the
next physical experiment. V8 physical A reads have succeeded and failed;
neither host tests nor this build establish barcode repeatability.

## Executed validation

- test_smooth_follow: alternating moderate optical errors produce substantially
  less target variation in a synthetic 10 ms fixture; left/right symmetry;
  sustained 600-level bends converge to original targets within 210 ms;
  clear departures and stops pass through immediately; recovery/junction
  bypass; actual weak/normal 400-level boundary obeys slew; profiles 1/2/3
  retain 45/90/135 cruising; faster cruise exits retain <=50 correction targets.
- test_barcode: measured PDF and historical captured A replays; frame/queue/
  cross qualification and record ABI; 12 alternating left/right turns per
  profile now run through the same smoothing wrapper used by main.c.
- test_analog: full A/F at both sizes/directions, noise/missing-bars rejection,
  ambiguous votes and queued crossings; four alternating turns per profile.
- test_line_search: stage timing, distance bounds, mirror acquisition,
  weak flashes, loaded effort and pending-command protection.
- test_start: 192 startup combinations and bounded no-history recovery.
- test_front_calibration: min-span 50 and measured references/noise/CRC.
- test_barcode_raw and six test_raw_capture_parser cases: saved raw data,
  integrity, bounds and complete-dump validation.
- test_barcode_sensitivity: measured low-amplitude fixtures, complete A/F/Z
  in both directions, missing-bar rejection and unchanged threshold mappings.
- Pico W compilation and UF2 verification pass. Optional SDK wireless/BLE
  submodules remain unused by this standalone USB test.

Test physically on a plain straight with profile 1, then on the same A print,
then on bends in both directions. Keep mounting/calibration fixed. Use j for
the compact result; p is optional when investigating wheel-target behavior.
