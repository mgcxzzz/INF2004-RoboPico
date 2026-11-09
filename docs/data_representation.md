# Data representation and compatibility

Public working structures use named fields and validated ranges. Requests and telemetry use explicit canonical bit/byte packing, not native C struct layout. Coordinator locks protect shared request/status state; the current SDK application uses cooperative core0 buddy calls, while core1 sees copied network packets only. Do not call the facade from an ISR.

| New request field | Valid range | Wire bits |
|---|---:|---:|
|Request kind|0–11|4|
|Buddy owner|1–5|3|
|Verified physical barcode index|0–25|5|
|Ramp feedforward fraction, milli|0–200|8|
|Stall push, PWM×100|0–6000|13|
|Ramp hold level, milli|0–1000|10|
|Assist TTL, ms|1–200|8|
|Convenience speed-assist percent|0–20|5|

The ramp request is46 meaningful bits including kind/owner, stored in6 bytes. Unused trailing bits must be zero and out-of-range fields are rejected. The physical-barcode approval request is12 bits in2 bytes. Existing finite-motion requests remain at most8 bytes; existing coordinator audit rows remain31 bytes and drive diagnostics58 bytes. Numeric control algorithms are unchanged by log quantization.

Source15's new21-byte C1 telemetry is length/subtype checked alongside existing28-byte ramp and24-byte peak layouts. It adds visibility without relabelling existing source9 requested targets. Some diagnostic values deliberately saturate (for example ramp |I| statistics at255 while the controller may allow300); consult the feature README when interpreting them.

New coordinator state has a RAM cost; this refactor does not claim a measured RAM saving or whole-firmware minimum-bit compliance. The actual linker layout is recorded in the build manifest. A formal tribunal before/after RAM comparison requires matching compiler/options and a separately rebuilt baseline. Historical subsystem bit-packing documents alone are not evidence for new fields.

## RTOS migration Step5 diagnostic additions

The migration images use micro T-Kernel tasks. The earlier cooperative/core
description above applies to the main SDK image. Motion still uses the shared
coordinator/API, and only the kernel-owned core0 service calls lwIP/MQTT.
Other tasks exchange copied records. Human-readable USB diagnosis is separate
from the existing compact binary MQTT application frames.

| Added working field | Valid range / interpretation | Representation |
|---|---|---|
| IMU executing core | 0–1 | 1-bit field |
| IMU calibration progress/best | 0–100 | 7-bit fields |
| Calibration restart-reason bitmap | Four independent reasons | 4-bit field |
| Calibration state/fault code | 0–6 | 3-bit fields |
| Calibration rejected axis span | 0–65535 mg | uint16_t |
| Calibration elapsed ms | 0–65535, explicitly capped | uint16_t |
| Ten-second IMU read/late counts | At most501 releases at50Hz | 9-bit fields |
| IMU FIFO batch count | 0–32 | 6-bit field |
| I2C last phase/failure reason | 0–6 | 3-bit field |
| I2C last device address | 0–127 | 7-bit field |
| Acquisition/profile event counters | 0–UINT32_MAX, modulo lifetime counters | uint32_t |
| Elapsed wall-time samples/maxima | Full uint32 microsecond delta range | uint32_t |
| Profile accumulated wall microseconds | Full uint32 modulo range | uint32_t |
| FIFO/init budget limits and task priority | Fixed build configuration | Constants, not queued fields |

Modulo microsecond totals wrap after about71.6minutes of accumulated time;
use unsigned subtraction for deltas. They are elapsed wall measurements,
including preemption/waits, rather than exclusive CPU accounting. Side-poll
measurements exclude their own counter-recording lock/update overhead.

The IMU task alone writes acquisition/calibration state. It copies snapshots
to the reporting mailbox under a short spinlock. Motion alone writes side-poll
counters, also under its existing copy lock. Readers copy before formatting;
neither sensor I/O nor formatting is performed under these locks. This also
protects read-modify-write operations on packed fields from cross-task races.

Step5 v0.1.4 adds no binary MQTT message fields: existing request/telemetry
packers and payload lengths are retained. The new USB diagnostics add RAM;
the v0.1.3/v0.1.4 linker measurements are recorded with the update. This is
evidence for the new fields and does not claim completion of the repository's
outstanding whole-firmware Tribunal before/after evaluation.


## Step5 v0.1.5 qualification fields

| New field | Defined domain | Representation |
| --- | --- | --- |
| Hold reason |0–3|2-bit field|
| Clear and retained failure reasons |0–10 each|4-bit fields|
| Gap-disarmed and heading-guard states |0–1 each|1-bit fields|
| Last/current gap milliseconds |0–127, capped|7-bit fields|
| Bounded gap microseconds |0–131071, capped|17-bit field|
| Qualification flags |four independent booleans|4-bit mask|
| Heading anchor |full signed16 modulo heading|int16_t|
| Heading anchor timestamp |full uint32 modulo microseconds|uint32_t|
| Hold/recover/disarm/early-clear counts |full modulo32 domain|uint32_t|

Context is written solely by the IMU task. Copied reporting fields cross tasks
under the existing short spinlock; formatting occurs after releasing it.
New fields do not alter binary message sizes. Compare static RAM using the
matching v0.1.4/v0.1.5 ARM build profile in the Step5 manifest/update.


## Main RTOS v0.2.0 release fields

| Field | Domain | Representation |
|---|---|---|
| Stored-export dataset |0–3|2-bit lease field; request uses existing 4-bit dataset slot|
| Frozen row count |0–2048|12-bit lease field|
| Run ID |full modulo16|16-bit lease field|
| Lease active |boolean|1 bit|
| Lease deadline / navigation sequence |full modulo32|uint32_t each|
| Release version major/minor/patch |0–15 each|4 bits each in explicit packed bytes|
| Wireless/log/calibration capabilities |independent booleans|individual bits|
| Profile / calibration origin |0–3 each|2 bits each|
| Four ADC calibration references |0–4095 each|four explicit 12-bit fields, 6 bytes total|
| Runtime timestamps / lifetime counters |full modulo32|uint32_t, explicit little endian|
| Hump clear/failure reason |0–10 each|4 bits each|
| Hold reason / qualification flags |0–3 / four booleans|2 bits / 4-bit mask|
| Safety flags |seven booleans|7-bit mask|

The lease has one writer (core1 navigation owner); shared Start checks use the
existing short copy lock. Calibration/history and IMU runtime snapshots are
copied under their existing locks before encoding; no sensor I/O, formatting,
network call or waiting occurs under those locks. Network sequence allocation
uses the existing synchronized allocator.

The shared source15 extension has validated subtype/length: C2 metadata is
15 bytes including header; C3 runtime is63 bytes. At1Hz each they add78 bytes/s
before MQTT/TCP overhead. Existing binary payload sizes and numeric source IDs
are unchanged. Stored rows reuse the existing source13 chunk protocol, with
bounded49-byte chunk data. The RTOS controller row is56 bytes, raw GP27 row10,
and navigation event31; no text field is used to represent these MQTT values.

The final build manifest compares linked static RAM against Step5 v0.1.5.
This is not dynamic stack high-water measurement or completion of the remaining
whole-firmware Tribunal before/after audit. Full-range counters intentionally
retain32 bits; narrowing them would lose their defined modulo domain.


## v0.2.1 diagnostic snapshots

No existing MQTT record lengths change. Shared source15 uses exact validated
subtypes: C4 encoder48 bytes, C5 barcode-attempt30 bytes, C6 network-stop32 bytes
(including their3-byte frame headers). With prior C2/C3, all five1Hz records
sum188 payload/frame bytes per second before MQTT/TCP overhead; C3/C5 require
ordinary telemetry enabled, while C2/C4/C6 still report with telemetry off.

| New field | Domain | Packed representation |
|---|---|---|
| Encoder valid/kind/wheel |boolean /0..3 /0..2|1/2/2-bit native fields and explicit flags|
| Direction duration |0..131071 diagnostic cap|17 bits, clamping flag|
| Trigger-window bad/changes |ten uint16 buckets; wire0..1048575|20 bits each, clamping flag|
| Requested wheel speed |−180..180|signed9 bits each|
| Previous PWM |−1000..1000|signed11 bits each|
| Measured speed |full signed16 diagnostic input|16 bits each|
| Lifetime edges/errors, timestamps, overwrite counters |full modulo32|32 bits each|
| Attempt raw count |0..2048|12 bits|
| Attempt reason/bars/flags |0..10 /0..24 /six booleans|4/5/6 bits|
| Attempt letter |wire unknown orA..Z; native ASCII0..90|5-bit wire index;7-bit native ASCII|
| Pre-roll head/count |0..511 /0..512|9/10-bit native fields|
| Network flags/link/failure mask/valid |three booleans /−3..3 /seven booleans /boolean|3/3/7/1 bits|

Encoder and network snapshots latch once in the core1 owner path. Readers copy
under short locks, encode after unlocking, and use existing bounded TX buffers.
Raw capture and its packed metadata have one writer (core1); network gets a
copied metadata view. Stopped export has an owner-side gate/Start lease. No
new subsystem calls lwIP/MQTT, no hardware I/O occurs under the new locks,
and no packed read-modify-write value is shared without synchronization.
The build manifest gives linked static RAM growth. This is not a substitute
for dynamic stack high-water or the remaining whole-firmware Tribunal audit.


## v0.2.2 drive diagnostics

C7 adds a 20-byte frame: subtype8, uptime32, decision serial32, run ID16,
threshold12, hysteresis8 (0..205), signed width16 (centi-mm, saturation flagged),
result3 and five boolean flags. Four padding bits must be zero. Width and the
classifier threshold/hysteresis/valid/armed evidence are frozen together;
pending-action state is current at snapshot copy. Native categorical fields
use bitfields. Width clips explicitly at +/-327.67 mm; classification uses the
original double width before diagnostic quantisation.

C8 adds an 18-byte frame: subtype/version8 each, uptime32, run ID16, bend
phase2, side2, blend percentage7, recovery phase3, sweep stage3, relative yaw
signed13 (0.1 degree), targets signed9 each and saturation1. Seven reserved
bits are zero and checked. Continuous control arithmetic retains float32
precision; it is quantised only for copied diagnostic messages. Timestamps and
encoder coordinates retain their full input domains. No gain/measurement is
quantised before controlling a motor.

With all telemetry enabled, C2-C8 add 226 frame bytes/second at 1 Hz, before
MQTT/TCP overhead. Two previously unused bits in the existing 56-byte controller
row retain the full-rate bend phase: meta bit31 and aux[1] bit31. Other row
fields and lengths stay unchanged; older rows decode phase0. Network readers
never access mutable planner state directly. This is not a claim that the
remaining whole-firmware Tribunal audit is complete.
