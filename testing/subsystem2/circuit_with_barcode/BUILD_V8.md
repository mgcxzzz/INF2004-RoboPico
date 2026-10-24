# Circuit with barcode V8: measured narrow-feature sensitivity trial

Built2026-10-25 for Pico W/RP2040, Pico SDK2.2.0, GNU Arm14.2.1,
CMake/Ninja Release, application C11 with -Wall -Wextra -Werror.

- Artifact:firmware/buddy23_circuit_with_barcode_v8.uf2,211968 bytes,
  414 sequential RP2040 blocks. Padded end0x10019e00; linker end0x10019d7c.
- SHA256:ffecf42fa8cdd61caeb8ccee7b5e2a2e63b10135e08583096566381b3f6fd570.
- ELF text109944/BSS215196; BSS end0x20035b9c, stack limit0x20040000,
  leaving42084 bytes between these symbols. Raw capture is a static49408-byte
  buffer; it does not allocate a second flash-sized staging copy.
- Main record64840 bytes, trace56, summary40 unchanged. Versions1-8 load;
  unknown9 rejects. V4-V7 bank indices use the historical threshold map.
- Raw record49408 bytes(page256 +4096 rows*12), CRC/readback verified in
  flash bank0x101b0000..0x101bffff. Probe calibration at0x101c0000/0x101c1000
  and main record0x101d0000 are separate. Image must end before the raw bank.
- UF2 format/family/sequence/addresses/trailer/markers and packaged/build
  equality verified. V3/V4/V5/V6/V7 UF2 hashes verified unchanged.

## Evidence and implementation

The user's5 stationary samples per position were broad white182/179/182/181/
180, narrow black221/220/215/218/219, narrow white204/206/203/205/206.
Their means are180.8/218.6/204.8. The closest observed black and narrow white
are9 counts apart. This supports a controlled sensitivity trial; it does not
establish contrast across all features while moving.

Original31 coarse thresholds and+/-32 hysteresis remain. Add32 thresholds
192..254 at2-count spacing, each with+/-3 hysteresis. All63 banks are sorted.
Default preview212 separates the supplied held-surface pair, but acceptance
still requires two adjacent thresholds decoding the entire Code39 *X* frame.
The reader still validates every bar/gap and both sentinels; no A/F guessing,
missing-element inference or partial-pattern acceptance was introduced.
Event flags expand the analogue bank index to6 bits under record version8.

The existing main-loop ADC owner records the first4096 active A0 samples with
actual microsecond timestamps, signed mean-wheel progress*100 and raw ADC.
Capture never overwrites its beginning. It marks full when later samples are
omitted; actual nominal500Hz timing is logged rather than assumed. Used-row
CRC and whole-record CRC protect data and metadata. Saving occurs only after
the motor stop/settling flow, without writing the existing probe banks.

The new a command requires stopped/settled state and cancels a pending launch.
It prints a paced5ms/row CSV with begin/end counts and binary-row CRC. On boot,
the capture restores independently; matches_motion compares duration/ticks/
profile/reason. If raw flash verification fails, the RAM capture remains
available until power removal. p/j retain their existing outputs.

V7 line steering, wheel gains, search and junction headers were not changed.
Front calibration/wiring remains GP26/GP28; barcode A0 remains GP27.

## Executed tests

- test_barcode_sensitivity: complete synthetic A/F/Z frames in both directions
  using the supplied narrow bar/gap amplitudes, plus1800-count wide black and
 181-count broad white; fine adjacent-frame acceptance; missing narrow bars
  reject; all original coarse thresholds remain; legacy mapping preserved.
- test_barcode_raw: bounded4096-row capture/no overwrite; full/max-gap/ticks;
  CRC/header/version/pin/torn-data/erased rejection; timestamp order and signed
  path; C/Python shared binary-row CRC fixture0x32c011eb.
- test_raw_capture_parser:6 Python cases cover LF/CRLF success, missing rows,
  changed ADC/CRC mismatch, missing footer, truncated line and incomplete last
  dump. Partial dumps are rejected before analysis.
- Existing test_barcode, test_analog, test_line_search, test_start and
  test_front_calibration pass with target min_span50 and rolling search enabled.
  They cover frame rejection/acceptance, queue/cross qualification, repeated
  mirrored junction turns, recovery bounds/effort, startup and record CRC/ABI.
- Pico cross compilation and UF2 verification pass. The compiler caught and
  the final build corrected a misleading-indentation warning in a status-print
  block. Optional SDK wireless/BLE submodule warnings do not affect this test.

No physical V8 decode is claimed yet. The next test starts the offset point
about20mm before a small A and stops beyond its final bar, then dumps j and a.
