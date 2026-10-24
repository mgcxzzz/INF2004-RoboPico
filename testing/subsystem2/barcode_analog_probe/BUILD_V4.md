# Barcode analogue probe V4 build evidence

Built 2026-10-24, Pico W / RP2040 / SDK2.2.0 / GNU Arm14.2.1 /
CMake-Ninja Release / C11 / application `-Wall -Wextra -Werror`.

Artifact: `firmware/buddy23_barcode_analog_probe_v4.uf2`

- SHA-256 `e3e52ffe7a9cb2692c563556f16b1df21e474b384bc9d3857a79d5ed163db044`.
- 94,208 bytes, 184 sequential RP2040 UF2 blocks. Every block's magic,
  payload size, addresses, family, count/index and trailer checked by
  `verify_firmware.py`; build/package equality and V1-V3 hashes verified.
- Padded end0x1000b800, linker end0x1000b7c4, below probe calibration slots
  0x101c0000/0x101c1000 and old barcode/circuit/legacy records at1d0000+.
- ELF text51,136, BSS168,344; BSS end0x2002a378, stack limit0x20040000.
  Sweep buffer10,000x16=160,000 bytes. Calibration record72 bytes, calibration
  payload40 bytes. Static ABI checks compile for both record and row layouts.
- No `pwm_*` symbol linked. Flash erase/program routines are linked only for
  explicit calibration save; source review checks no save in capture/sweep.
- ADC1/GP27 remains sole analogue input; GP19 and front ADC inputs untouched.
  GP8..11 output LOW before initialization and in loop. No motor-start path.
- Encoder quadrature logic and distance conversion reuse existing bench/dual
  headers: leftGP0/1, rightGP4/5, left forward-negative/right-positive.
- Sweep buffers raw readings/timestamps/encoder positions; no serial data dump,
  normalization, float formatting or flash write occurs during sampling.
  Sampling jitter/missed slots and encoder invalid transitions are reported.
  USB disconnect ends capture; dump interruption retains the RAM copy.

Persistent calibration source review: version/pin/channel/length/finite data/
span/noise/CRC checks; alternating slots retain previous record until next save
passes readback; wrap-aware sequence selection; firmware-overlap guard; saved
reference status invalidated after edits; sweep keeps its own reference copy.
Power failure can lose the latest update, with the earlier valid slot retained.

The hardware save/restore cycle, encoder signs during hand rolling and analogue
scan have not been physically tested. The installed Arm debugger has no CPU
simulator; no claim is made that host execution of flash or sampling occurred.
Next device checks: w/b/k, power cycle and c; then f/roll/s/p on the small A.
These changes add no A/F inference or decoder thresholds.
