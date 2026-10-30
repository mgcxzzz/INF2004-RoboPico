# WiFi V1 representation evidence

This is **partial implementation** of the Tribunal minimum-bit requirement. Interface state/faults, canonical navigation requests and diagnostic/audit records remain compact. MQTT uses explicit binary encoders. Some telemetry fields remain byte-aligned/wider than their minimum range; legacy working floats/records remain. This does not claim project-wide compliance.

| Changed data | Range/precision | Representation |
|---|---|---|
|Canonical route|26 A-Z values|5 bits; kind4+buddy3+letter5=12 meaningful bits in2 bytes|
|Map|Four distinct A-Z letters|20 bits:four5-bit indices|
|Configured/revision|Boolean/0..255|1+8 bits,upper3 bits zero in uint32|
|Captured action|S/U/L/R or absent|Native byte; further packing remains|
|Captured revision|0..255|8 bits,stable after decode|
|Saved barcode event|Existing event data plus revision|32-byte row; revision/reserved consume former2 padding bytes|
|MQTT motion|Speed0.1mm/s,target mm/s,heading0.01degree|45-byte payload; explicit LE; ADC/levels uint16 containers|
|Raw network row|us,path0.01mm,ADC|10 bytes:u32,i32,u16|
|Log fragment|At most49 data bytes|12 metadata+data+3 common header|
|Motion record|Schema26|Latest384 rows; old front calibration/profile imported|
|Raw record|Schema2|First2048 samples; old raw layout rejected|

Exact network offsets/bounds are in wifi_packets.h/WIFI_CONTRACT.md. Native working structs are not packets. Firmware does not send JSON/text scalar telemetry.

Retained finite state: mode3 bits,owner3,command state3,terrain2,active/stopped/interrupted1 each,stop reason6,faults10. Canonical requests explicitly pack kind4,buddy3,drive goal15 at0.1mm,drive speed6,turn goal12 at0.1degree,turn speed7,lease32,wheel targets9 each. Validate before assignment.

| Canonical request | Bytes |
|---|---:|
|Takeover/Stop|1|
|Drive/Turn|8|
|Resume|5|
|Route/Terrain|2|
|Wheel demand|4|
|Fault report|3|

A-Z route still uses2 bytes. Coordinator audit:241 meaningful bits+7 padding in31 bytes. Diagnostic format4:464 meaningful bits in58 bytes (ADC12,levels10,targets9 and finite-state fields). Audit clipping is disclosed, never changes control. Normal motion/counters retain legacy precision.

## RAM comparison

| Identical WiFi features | ELF BSS bytes |
|---|---:|
|Packed shared interface|193144|
|Unpacked shared-state reference|198104|
|Saving|**4960**|

The reference expands shared enum/flag fields only:152 event slots save4864B,16 result slots64B,status32B. Host object comparisons remain MotionStatus64 to32B,MotionCommandStatus20 to16B,MotionEvent72 to40B. Audit encoding31B.

Network-on adds45072B versus network-off148072B. Total BSS is44364B below Continuous V2's237508B because histories/storage scratch were reduced; this is **not** a like-for-like packing saving. BUILD_METRICS.json records variants.

Core0 atomically replaces mapping/captures revisions. Coordinator faults/leases/audit and generation-checked commits retain spinlock/IRQ discipline. Core1 owns networking; copied queues/status use a separate spinlock. SDK flash lockout parks core1 before stopped writes mask IRQs.

Remaining project work: inventory all fields/ranges, migrate remaining legacy/byte-aligned fields, agree physical precision, measure complete-system timing/worst-case stack/heap, and physically demonstrate networking/following/barcode/junction/IMU/obstacles. Tests/linker symbols do not replace hardware evidence.

V1.1 adds copied network diagnostics: SDK link state -3..3 in int8, latest MQTT result with -32768 unavailable sentinel in int16, IPv4 and attempt count in uint32. These native diagnostic fields are byte-aligned, not a claim of minimum-bit completion. Core1 updates them under the transport spinlock; core0 n reads a locked copy. The added status state increases each variant BSS by8B after existing padding.

V1.2 adds three-bit network phase (0..6), full-width uint32 completed-loop count/boot-ms/stored broker IPv4, and a core0-private byte setup-result code (0..9 or255 not attempted). Shared network diagnostics retain the transport lock. This diagnostic representation is still only partial minimum-bit compliance. Each ARM variant adds16B BSS versus V1.1; protocol packets, control thresholds and log formats are unchanged.
