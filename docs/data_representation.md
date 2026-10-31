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
