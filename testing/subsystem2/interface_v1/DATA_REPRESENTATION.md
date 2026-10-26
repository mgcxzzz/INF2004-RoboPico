# Minimum-bit requirement: Interface V1 implementation and evidence

**V1.2 retains the same shared API representations and4960B packing saving.** Its ordinary-drive/tracking state adds48B BSS; packet layouts and precision remain. Drive state uses a16-bit0..60000us deficit counter,8-bit0..180 prior target and1-bit active flag. Tracking state uses19-bit0..400000us qualify,17-bit0..120000us departure counters and Boolean flags; float working calculations remain further audit scope. Current verification is in BUILD.md; original release measurements are archived in BUILD_V1.md / BUILD_V1_1.md.

## Compliance boundary

Interface V1 starts the Tribunal change at the new cross-buddy boundary: canonical binary requests, compact finite-state/flag storage, one synchronized fault bitmap, and compact saved coordinator events. This is **partial implementation**, not whole-firmware compliance. The frozen V12.2 navigation/encoder/raw/motion data, floating-point working calculations and legacy record layouts remain. MQTT transport is not present in this standalone test. The remaining team-wide migration and required physical timing/performance evidence must still be completed for Week 13.

Keeping the known V12.2 record ABI and driving algorithms during the interface handoff lets the group verify the representation migration separately from steering changes. It does not exempt legacy fields from the eventual requirement.

## Canonical request representation

`nav_codec.c` explicitly packs least-significant bit first into bytes. This wire/storage representation does not depend on C bit-field order, padding, host endianness or struct casts. Each request uses only its own payload. The common header uses kind 0..9 in 4 bits and buddy 1..5 in 3 bits. Unused final-byte padding is zero and rejected if nonzero on decode. These are synchronous local messages; there is currently no RTOS request queue or network packet transport.

| Field | Valid encoded range | Bits | Implementation |
|---|---:|---:|---|
| Kind | 0..9 | 4 | Explicit masks/shifts |
| Buddy | 1..5 | 3 | Explicit masks/shifts |
| Lease | 0..2^32-1 | 32 | Full token preserved; execution rejects stale/zero leases |
| Drive goal | -1000.0..1000.0 mm at 0.1 mm | 15 | Offset integer; API excludes magnitude below 1 mm |
| Drive speed | 0 or 20..60 mm/s | 6 | 0 selects default 45 |
| Turn goal | -180.0..180.0 degrees at 0.1 degree | 12 | Offset integer; API excludes magnitude below 1 degree |
| Turn speed | 0 or 20..90 mm/s | 7 | 0 selects default 45 |
| Turn style / abandon flag / fault-active | 0..1 | 1 each | Masks/shifts |
| Route | A/F/Z | 2 | Compact enum, not ASCII on request wire |
| Terrain | UNKNOWN/FLAT/ASCENDING/DESCENDING | 2 | Compact enum |
| Left/right provider target | -180..180 mm/s | 9 each | Offset integer |
| Fault bitmap | Ten defined conditions | 10 | Shared bitmask |

| Request | Used bits including header | Bytes |
|---|---:|---:|
| Takeover or stop | 7 | 1 |
| Drive | 60 | 8 |
| Turn | 59 | 8 |
| Resume | 40 | 5 |
| Route / terrain | 9 | 2 |
| Wheel demand | 25 | 4 |
| Fault report | 18 | 3 |

Speed assist is reserved and returns `UNSUPPORTED` before dispatch; its reserved codec slot does not activate an IMU speed policy. Values are validated before copying them into bit-fields so invalid caller arguments cannot silently truncate into accepted commands. Avoidance goals are quantized once to 0.1 unit; original autonomous wheel targets are integers and pass through exactly. These new primitive resolutions/tolerances need confirmation against the final project accuracy requirements.

## Shared runtime state and coordinator audit

Native convenience structs are not treated as binary packets. Runtime floats and some compatibility fields are intentionally retained; they are further minimum-bit audit work. Finite states/flags use the following bit-fields under a shared lock:

| Native field | Range | Bits |
|---|---:|---:|
| Motion mode | 8 states | 3 |
| Owner | 0..5 | 3 |
| Command state | 5 states | 3 |
| Terrain | 4 states | 2 |
| Session-active / stopped / interrupted | Boolean | 1 each |
| Legacy stop reason carrier | 0..63 | 6 |
| Faults | Ten independent bits | 10 |
| Event type | 0..14 | 4 |
| Event kind | 0..9 | 4 |
| Request result | 0..8 | 4 |

Shared fault assignments: bit 0 timeout, 1 failed settling, 2 heading deviation, 3 wrong direction, 4 encoder, 5 IMU, 6 no echo, 7 Wi-Fi, 8 control timing, 9 stall. Reporting is telemetry; an explicit stop API is required when a buddy's policy needs a halt. Agree one responsible reporter per bit; clear is a global bit clear, not a per-reporter reference count.

Saved coordinator audit events use 31 bytes (241 meaningful bits, 7 zero padding bits): full 64-bit microsecond timestamp; full 32-bit event sequence and command ID; requester/owner 3 bits each; type/kind 4 each; state 3; faults 10; result 4; style 1; speed 7; goal 15 at 0.1 unit (-1000..1023); progress 24 at 0.1 mm (-838860.8..838860.7); relative left rotation 16 at 0.1 degree (-3276.8..3276.7); explicit overflow 1; two wheel targets 9 each. Clipping applies only to audit serialization, sets overflow, and does not change live control/result calculations. Event progress is command-relative, not an unbounded whole-course distance. Header metadata preserves full 64-bit run duration.

The coordinator log contains 120 events and a 256-byte header in one 4096-byte sector. Header-last programming, schema/length checks, header CRC and payload CRC reject erased, torn or corrupt records. `overwritten` is explicit. The default audit records Buddy 3 targets at one-second intervals and phase/decode notices, plus external requests/results; the 100 Hz motion trace remains separate.

## RAM and message measurements

`INTERFACE_UNPACKED_REFERENCE=ON` compiles the **same interface features and algorithms** with the new status/result/event enum/flag fields expanded to ordinary unsigned members. This is a measurement reference, not a published robot firmware. Both variants pass the same API tests. See BUILD.md for final ARM ELF measurements.

| Host C object (same compiler/ABI) | Unpacked reference | Packed interface |
|---|---:|---:|
| MotionStatus | 64 B | 32 B |
| MotionCommandStatus | 20 B | 16 B |
| MotionEvent | 72 B | 40 B |
| Request working object | 16 B | 16 B (not a transmitted packet) |
| Actual serialized request | Not applicable: no old deployed queue | 1..8 B; wheel demand 4 B |
| Serialized audit event | 72 B if copying reference struct | 31 B canonical |

The identical ARM builds show **4,960 B less BSS** with compact fields: 152 event slots save 4,864 B, 16 result slots save 64 B, and status saves 32 B. This is a like-for-like interface representation saving. The integrated firmware uses more BSS than the old V12.2 firmware because coordination and its audit buffers are new features. Do not present that integration overhead as a whole-firmware RAM reduction. Copying a C struct is only a comparison baseline, not a supported protocol; actual packet lengths above are measured from the codec tests.

## Synchronization and remaining evidence

Coordinator reads, writes, bitmask changes, result slots and audit slots all use one port lock. The Pico implementation claims a spinlock and masks local IRQs. The final motor commit checks a generation token while holding the same lock, so a request calculated before stop/takeover cannot subsequently restore motor PWM. ISR-owned encoder/guard snapshots retain the existing IRQ protection. Public buddy calls belong in normal poll/task context, not ISR context.

The standalone harness runs cooperatively on core 0. Its stopped flash-saving path disables IRQs on that core; moving this to a two-core/RTOS design requires explicit flash lockout and a validated scheduling port. Packed bit-fields are never modified through unsynchronized public pointers. Canonical bytes must be used for future queue/MQTT transport, not shared raw C bit-fields.

Executed host evidence covers exhaustive 0.1 mm drive-goal codec roundtrips, padding/range rejection, ownership/lease/pre-emption, stale motor commits, cancellation and finite results, interrupted-junction handback, corrupt/torn saved logs, the Buddy 5 dispatcher workflow, all retained V12.2 tests, and 9,000 optical/PI routing-parity ticks across all profiles. These are logical/model checks, not measurements of Pico execution latency or robot accuracy.

Still needed for final submission: inventory all legacy application fields and valid ranges; agree physical precision; migrate remaining records/states/shared queues and MQTT payloads with versioned compatibility; compare the final complete firmware before/after; measure on-device timing/control-loop gaps, packet sizes and stack/RAM usage; and demonstrate normal following, barcode/junction behavior, IMU integration and obstacle avoidance physically. Runtime timeouts remain, but passing them on a real run is not a substitute for the required timing comparison.
