# Barcode analogue probe V2 build evidence

Built 2026-10-24, Pico W / RP2040 / SDK 2.2.0 / GNU Arm 14.2.1 /
CMake-Ninja Release / C11 / application `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_barcode_analog_probe_v2.uf2`

- SHA-256: `800d727f8ef512ade68506252c3a0e433b8ea102d0ccf5d359ab909f2f0f4133`.
- 70,656 bytes, 138 sequential RP2040 UF2 blocks. Every block checked for
  magic, addresses, payload size, family, count/index and trailing marker.
- Padded image end `0x10008a00`, linker end `0x100089e4`, well below saved
  circuit record bank `0x101d0000`. Packaged file equals build output.
- ELF text 39,392 bytes, BSS 4,028 bytes; USB initialization symbol present.
  No `pwm_*`, `flash_range_erase` or `flash_range_program` symbols linked.
- Version/GP27/ADC1/physical pin32/capture labels/motors-off markers verified
  in binary; former GP26 wiring and D0 percentage markers absent.
- Source review: ADC GPIO27 selected with channel `ANALOG_PIN-26` = 1;
  static wiring assertion; GP26/GP28 and GP19 untouched; motors GP8..11 LOW
  before USB and continuously; no motor-start or flash-write path.
- V1 artifact hash remains `031a6276a25ce3f0b3bafa34ddf7816a03c0f29726254835e9935e5ed434fccf`.

V2 changes the probe wiring from temporary GP26 to spare GP27 and removes
the optional D0 comparison. Captures/statistics are otherwise unchanged.
Physical pin mapping verified against Raspberry Pi's Pico W pinout PDF.
No hardware readings have yet been collected; contrast remains unverified.
