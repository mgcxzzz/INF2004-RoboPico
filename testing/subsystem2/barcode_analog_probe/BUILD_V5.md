# Barcode analogue probe V5 build evidence

Built 2026-10-24, Pico W / RP2040 / SDK2.2.0 / GNU Arm14.2.1,
CMake/Ninja Release, C11, application `-Wall -Wextra -Werror`.

- Artifact `firmware/buddy23_barcode_analog_probe_v5.uf2`,96,256 bytes,
  188 sequential RP2040 blocks; padded end0x1000bc00, linker end0x1000bb7c.
- SHA-256 `9f425afa45a2e65a7ef4635cf7160827ae4e10e135a626caedbf8941c986e066`.
- ELF text52,088, BSS168,344; BSS end0x2002a378, stack limit0x20040000.
- verify_firmware.py checks every UF2 block, markers, image bounds,
  build/package equality and unchanged V1-V4 hashes.
- Motors remain LOW, sampling unchanged. Dumps flush and wait5ms per numbered
  row; USB stdout timeout5s, previously10ms. Header count, row indexes and
  CRC32 over normalized-LF numeric lines identify output loss/corruption.
- Probe saved calibration ABI/slots unchanged; incremental CRC implementation
  retains the existing record CRC algorithm. V4 calibration can restore.

## Executed tests

Python audit suite6 tests passed: LF/CRLF complete dump, missing row despite
end marker, modified voltage with failed CRC, truncation/joined lines, mixed
V4/V5 sessions and reversed timestamps. Audit of the actual user PuTTY log
flags all3 V4 dumps incomplete:1069/9690,1369/8241,1347/7321 rows. End marker
alone is insufficient; the second has a14.186533s sample-time hole.

Native C11 host tests (Zig0.13 Windows cc, warnings-as-errors) passed for
normalization both polarities, invalid/noisy/nonfinite references, record
CRC/pin/version/erased/corrupted rejection, old-slot validity, sequence wrap,
and CRC32 golden vector123456789=cbf43926 including incremental updates.

This addresses a plausible source of the observed output loss: SDK USB stdio
can discard unwritten text after its timeout. Paced hardware output remains
to be tested; checksums detect remaining loss, not repair it. No A/F decoding
was inferred across missing sections of the user's captures.
