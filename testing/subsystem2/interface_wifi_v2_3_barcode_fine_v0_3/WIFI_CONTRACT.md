# WiFi V1 integration contract

The standalone Pico SDK harness preserves Continuous V2 controls, motion APIs, one navigation coordinator and one motor writer. Default Buddy5 hooks are empty. Buddy1's MQTT v2 foundation comes from commit b09403074939f45cf4d7c11b7606148f6c2a4e73; its RTOS scheduler is not imported.

Core0 owns ADC, validation, mapping and motion. Core1 exclusively owns CYW43/lwIP/MQTT. Copied spinlock-protected queues hold12 outgoing packets (max64 bytes) and8 incoming copies (max8 bytes, original length retained for rejection). Callbacks never invoke motion. Network retries occur on core1; reconnect alone never starts motion. Network loss does not stop/restart autonomous following.

Network-enabled firmware ships without credentials. Saved USB settings or an optional local header enables core1. Power-saving set/readback is reported by n. Stopped-only flash writes park core1 using SDK lockout before masking IRQs; failure rejects the write.

## MQTT

Topics: is28/cmd, is28/tlm, is28/ack, is28/hb, is28/lwt. Commands from this dashboard are non-retained. lwIP receive callbacks do not expose the retained flag: do not install retained movement commands on the lab broker.

Commands:1 Start,2 Pause,3 Resume,4 Stop,5 Reset,6 ESTOP,7 telemetry configuration,8 Ping,9 whole-map replacement,10 map readback,11 GET_LOG. Direct steering/drive/turn12/13/14 are prohibited. Ordinary commands4 bytes, map5, ACK4. Results:0 accepted,1 prohibited,2 unknown,3 format,4 busy,5 unsupported. Acceptance does not imply maneuver completion.

Start retains a2s countdown/calibration/button/rearm checks. Manual stop cancels it. Pause acquires a Buddy1 coordinator lease; Resume retains measured-stop/line gates. ESTOP latches until stopped/settled Reset.

Mapping slots Straight/U-turn/Left/Right hold four distinct A-Z letters in20 bits (four5-bit indices). Configured bit20; revision bits21..28; upper3 bits zero. No boot assignments. Decode captures action/revision; subsequent changes affect later decodes. Queued/executing action stays stable. Straight consumes one isolated junction without turning; L/R/U use unchanged geometry. Stop/Reset retain map; reboot clears it. Matching readback confirms the dashboard map.

## Binary telemetry

Explicit little-endian encoders, never raw C structs. Common3-byte header:[source<<4|2,payload length,sequence]. Exact offsets: wifi_packets.h and dashboard/is28_schema.js.

| Source | Meaning | Total bytes |
|---|---|---:|
|1|Heartbeat|15|
|8|Map readback|8|
|9|Motion/speeds/PWM/ADC/levels/distance/heading|48|
|10|Barcode pipeline/pending/executing revisions|24|
|11|Coordinator request/result/ownership/event|26|
|12|Whole-run summary|25|
|13|Stored row fragment|15..64|
|14|Barcode counters/threshold/timing/drops|63|

Motion/line nominal20Hz; heartbeat/status/diagnostics1Hz. Actual decode/queue/junction/dispatch/completion remain separate. Packet times/drop counters disclose delivery loss. Unavailable levels use65535.

GET_LOG: high argument nibble selects dataset0 motion,1 diagnostics,2 raw,3 coordinator; low12 bits row index. Core0 rechecks stopped/settled state before reading. Source13 includes IDs,row,total,fragment index/count,status,length. Diagnostic58-byte rows split49+9. Browser matches every row, retries at most3 times, aborts incomplete downloads.

Page-session archive:72000 motion samples/20000 events, disclosed truncation. Charts latest240. Stored motion flag128 marks run-relative timestamps; live timestamps boot-relative.

Host tests exercise bounds/reserved bits, A-Z/actions/directions, revision capture, actual coordinator/harness, CRC/import, fragments and C-to-JS vectors. Browser MQTT is simulated. Three strict-warning ARM builds cover network-on/off/unpacked reference. Physical connection/course/timing still need hardware tests.
