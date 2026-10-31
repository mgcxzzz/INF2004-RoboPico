# Telemetry and stored diagnostics

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Data paths

The canonical binary packet format is retained. Byte/bit packing is explicit; native C bitfield layout is not a wire protocol. Existing motion, barcode/navigation, summary, diagnostics and stopped-log export layouts remain compatible. `drive_diag.h` stores a58-byte record with raw/filtered ADC, normalized levels, optical/soft/final requested targets, speed/reference/integral, output and timing/flags. `nav_log_storage.h` stores31-byte coordinator events.

The detailed drive ring freezes around its diagnostic trigger and is not an unlimited recording. Motion history contains384 rows; raw GP27 history contains the first2048 samples. A later failure may be outside the raw window. Do not assume an export's download time is the run time, or that video seconds equal run-relative timestamps.

Source15 has validated layouts distinguished by length/subtype:

| Payload | Meaning |
|---|---|
|28 bytes|Existing ramp-assist event/statistics|
|24 bytes, byte4=0xB1|Peak decoder result/build flags|
|21 bytes, byte4=0xC1|New coordinator-approved ramp levels and requested/effective targets|

The C1 fields are uptime32, subtype8, flags8, boost-milli8, push×100 unsigned16, hold-milli16, requested L/R and constrained L/R signed16 at0.1mm/s, owner8 and TTL8. Undefined flag bits and out-of-range control fields are rejected by the dashboard. The constrained targets include the speed-hold floor; stored REF records the later slewed motor reference. Old source9 target fields continue to mean the optical/coordinator request, rather than being silently relabelled.

## Export workflow

Record live messages during a run. Stop and wait at least5s for settling/saving, then use the dashboard's stored-data requests and download Motion, Drive diagnostics, Navigation and Raw sensors as CSV plus the messages/events text. USB alternatives are `h` (build/calibration), `p` (motion/summary), `d` (detailed drive), `j` (barcode), `a` (raw) and `z` (coordinator). Let each dump finish. No live USB printing is required during driving.

Counter snapshots, ACKs and event summaries are not substitutes for the raw signal around a rejected label. Include firmware filename/build manifest, calibration readback, barcode letter/direction and a video when useful. Historical evidence is indexed separately and is not merged into current-run measurements.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
