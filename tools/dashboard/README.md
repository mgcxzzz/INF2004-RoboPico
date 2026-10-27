# Buddy 1 dashboard and barcode configuration

Run Mosquitto with `tools/dashboard/mosquitto.conf`, build the Pico with
`WIFI=cyw43 NET=lwip MQTT=1` and your gitignored `config/wifi_secrets.h`,
then open `index.html`. Full bench steps and evidence rules are in
`testing/subsystem1/HOW_TO_TEST.md`. The dashboard currently loads MQTT.js
from a pinned CDN URL, so its first load needs internet access.

## Network ownership

All lwIP/MQTT calls stay in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`,
running in the kernel WiFi service context. Application tasks send copied
frames through Buddy 1's TX message buffer and consume copied commands
from its RX message buffer. Network-side operations use nonblocking
polling; a poll publishes at most four queued messages plus a heartbeat.

TX capacity is 2048 bytes, maximum body 40 bytes. RX capacity is 64 bytes,
maximum staged body 8 bytes; valid commands are only 4 or 5 bytes. A full
queue drops the message and increments the drop count. Oversized inbound
messages are staged to 8 bytes and rejected by the command checker, so a
valid-looking prefix cannot be applied. Heartbeats are produced separately
from the outgoing telemetry queue.

Only `is28/cmd` is subscribed. The topic is resolved to this fixed route
inside the callback; no borrowed topic or payload pointer is retained.
Payload fragments are copied to bounded staging, then copied to the RX
message buffer before the final callback returns. If more subscribed
topics are added later, carry a bounded topic copy or explicit route ID
with each RX message.

Bridge calls guard their object IDs before accessing queues, so early
network callbacks cannot use uncreated objects. The client requests its
subscription on each successful connection. Real startup, full-buffer,
fragmentation and reconnect behaviour remain hardware test S1-U21.

## Barcode mapping wire format

The instructor chooses four distinct uppercase letters A–Z before the
demo, assigning Forward, U-turn, Left and Right. There are no default
assignments. Application source files do not call the MQTT APIs.

Publish non-retained binary commands on `is28/cmd`:

| Command | Byte 0 | Byte 1 | Remaining bytes |
|---|---|---|---|
| BARCODE_MAP | `0x92` | request ID, 0–255 | 3-byte little-endian packed map |
| BARCODE_GET | `0xA2` | request ID | `0x00 0x00` |

The map contains four 5-bit letter indices (A=0, Z=25), in this order:
bits 0–4 Forward, 5–9 U-turn, 10–14 Left, 15–19 Right. Bits 20–23 are
zero. Duplicate letters, indices 26–31, nonzero reserved bits and wrong
lengths are rejected. Existing supervisory commands and acks stay 4 bytes.
Direct STEER, DRIVE and TURN commands remain prohibited.

Both commands answer with an ack on `is28/ack` and an 8-byte source-8
configuration frame on `is28/tlm`. The shared schema is
`app_program/system/sys_tlm.h`. Its 3-byte header is followed by a packed
word (map bits 0–19, configured bit 20, 8-bit wrapping revision at 21–28,
zero reserved bits 29–31) and the request ID. Source-8 sequence numbers
count readbacks and are excluded from periodic telemetry-delivery tests.

The dashboard shows the draft separately from **Active on car** and needs
matching readback to confirm application. Ack alone does not confirm the
map in the UI. A full outgoing queue can drop readback after a successful
application: the ack still truthfully says accepted; GET resolves whether
it took effect. Repeating the same map is idempotent. Updates are atomic
as a whole under a kernel mutex; readers never see four partial updates.

Configuration lives in RAM. A supervisory RESET retains it; power loss
clears it. On reconnect the dashboard queries it again. The dashboard
publishes configuration with `retain=false`; it does not store the mapping
as a retained MQTT command.
Configure before starting a run; state-dependent update policy can be
agreed when navigation integration is undertaken.

## Interface for later Buddy 3 integration

`b1_barcode_mapping_get(&snapshot)` in `subsystem1.h` copies a coherent
packed map/configured/revision word under a mutex, from task context on
either core, using `TMO_POL`. It returns E_OBJ before creation and an
error if busy; retry rather than treating an uninitialised snapshot as valid.
`b1_mapping_lookup(snapshot, uppercase_letter)` in `b1_mapping.h` returns
NONE, FORWARD, UTURN, LEFT or RIGHT. Unconfigured/unassigned/invalid input
returns NONE. These enums are configuration values, not motor commands.

Buddy 3 still needs A–Z decoding in both directions, stored-letter state
and next-cross-junction execution for all four movements. Buddy 2 motion
integration is also deferred. Nothing here changes either buddy's code,
and the dashboard does not label a derived mapping lookup as a pending or
executed movement.

## Test limitations

Host checks exercise the actual C parser/packing/lookup and compare C
vectors with the JavaScript decoder. The browser test uses a simulated
MQTT transport. Firmware compilation with example credentials validates
buildability only. Real WiFi/MQTT, queue concurrency, reconnects and physical
navigation require their own recorded Pico runs.

Connectivity verdicts require the full 300 seconds, assess delivery per
observed source, and include silent first/last intervals in heartbeat gaps.
Delivery is a wrapping-sequence estimate; it cannot prove receipt of a
source that never appeared or distinguish every long outage from reboot.
Record expected sources and use independent sent/received counts when
claiming end-to-end delivery.

For the heartbeat-gap diagnostic, the local gitignored `wifi_secrets.h`
may define `WIFI_PM_MODE CYW43_NONE_PM`. The WiFi service task applies
this after interface setup and logs the requested and read-back settings.
The low nibble of the active value is the PM mode: 0 means no power saving.
Require both API return codes to be zero before calling the PM-off setup
verified. Removing the define restores the template's default behaviour.
Keep this as an experiment until the unchanged five-minute timing test is
repeated; the setting is not a proven fix. No heartbeat interval, threshold,
authentication mode or subsystem integration changes are part of this test.
