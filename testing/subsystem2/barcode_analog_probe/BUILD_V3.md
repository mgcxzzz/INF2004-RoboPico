# Barcode analogue probe V3 build evidence

Built 2026-10-24 for Pico W / RP2040, SDK 2.2.0, GNU Arm 14.2.1,
CMake/Ninja Release and C11 with application `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_barcode_analog_probe_v3.uf2`

- SHA-256 `d063f755bbc5e495609d82216f13cda5ad8f7ef49b92f4821a816b737575951c`.
- 77,312 bytes, 151 sequential RP2040 UF2 blocks. Magic, payload size,
  family, addresses, index/count and trailer verified for every block.
- Padded end `0x10009700`, linker end `0x1000966c`, below saved circuit
  record bank `0x101d0000`. Firmware copy equals build output.
- ELF text 42,600 bytes, BSS 4,068 bytes; USB initialization symbol present.
  No `pwm_*`, `flash_range_erase` or `flash_range_program` symbols linked.
- Binary includes V3/GP27/ADC1/calibration/normalized-level/black-high/black-low
  and narrow-gap/bar capture markers. Former GP26 probe wiring marker absent.
- Source review: unchanged ADC1 on GP27, GP8..11 output LOW before USB and
  continuously. No GP19/front ADC access, motor-start or flash-write path.
- V1/V2 artifact hashes verified unchanged against their build evidence.

Calibration review: b requires a white reference first; w resets black;
references update only on completed captures with >=500 samples; masked
references, >=100-count span and >=6x reference-SD checks gate validity;
normalization uses signed black-minus-white and clamps to0..1000; invalid
calibration reports -1. g/n measurements never alter reference values.
Interrupted USB captures are cancelled; c/p report and r clears RAM only.
Reference checks do not claim narrow bar/gap contrast or decoder success.

No physical calibration results have been measured yet. No driving firmware,
saved record/calibration format or flash bank was modified. Calibration is
volatile and must be repeated after power-off.
