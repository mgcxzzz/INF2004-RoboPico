# Wi-Fi protocol attribution

The command parser, acknowledgement layout, heartbeat source-1 layout, and
source-8 four-letter map layout preserve the reviewed Buddy 1 schema v2 and
algorithms from upstream snapshot `b09403074939f45cf4d7c11b7606148f6c2a4e73`:

- `app_program/subsystem1/b1_proto.c` and `b1_proto.h`
- `app_program/subsystem1/b1_mapping.c` and `b1_mapping.h`
- `app_program/system/sys_tlm.h`
- `tools/dashboard/is28_schema.js`

The implementation in this folder is a pure-C adaptation with the public
`wp_` API. It preserves the four-byte command and ACK frames, map word bit
positions, idempotent map replacement/revision behavior, and source-1/source-8
wire compatibility. Command 11 is the documented stopped-only log download
extension. Sources 9–13 are new bounded telemetry records documented by
`wifi_packets.h`; they do not replace or reinterpret the upstream frames.
