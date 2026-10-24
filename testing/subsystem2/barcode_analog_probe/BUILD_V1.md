# Barcode analogue probe V1 build evidence

Built 2026-10-24 for Pico W / RP2040, Pico SDK 2.2.0, GNU Arm 14.2.1,
CMake/Ninja Release, application C11 with `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_barcode_analog_probe_v1.uf2`

- 70,656 bytes, 138 sequential RP2040 UF2 blocks.
- SHA-256: `031a6276a25ce3f0b3bafa34ddf7816a03c0f29726254835e9935e5ed434fccf`.
- UF2 magic, sequential addresses/indexes, payload sizes, total block count,
  family `0xe48bff56`, and end markers checked on every block.
- Padded image end `0x10008a00`; linker end `0x100089bc`. All writes are below
  circuit record bank `0x101d0000`; firmware copy is byte-identical to build output.
- ELF text 39,352 bytes, BSS 4,028 bytes. USB initialization symbol present.
- Linked ELF has no `pwm_*`, `flash_range_erase` or `flash_range_program`
  symbols. Source review confirms GP8..11 output LOW before USB and in loop,
  no motor-start command, only ADC0 and D0 input access.
- Image contains version banner, narrow-gap and adjacent-bar labels,
  D0 comparison field and motors-off diagnostic markers.

No hardware measurements have been performed with this probe. The existing
circuit firmware, decoder, calibration and saved record formats are unchanged.
No threshold or decoding success is claimed until the user measures repeated
gap/bar captures at the normal mounted height.
