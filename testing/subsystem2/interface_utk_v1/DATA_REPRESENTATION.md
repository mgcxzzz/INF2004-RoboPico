# Data representation and integration scope

This document describes the bounded binary interfaces used by the first-phase
network proof image. It is not a declaration that the complete robot firmware
or every subsystem requirement is implemented.

## MQTT frame envelope

Buddy 1 uses the shared version-2 team frame on the existing MQTT topics:

| Topic | Direction | Purpose in this image |
|---|---|---|
| `is28/hb` | Pico → laptop | Periodic Buddy 1 source-1 heartbeat |
| `is28/tlm` | Pico → laptop | On-demand source-8 map readback |
| `is28/ack` | Pico → laptop | Acknowledgement for an accepted or rejected command |
| `is28/cmd` | Laptop → Pico | Bounded supervisory command |
| `is28/lwt` | Broker → dashboard | Broker last-will status when the Pico disconnects |

Telemetry frame bytes are:

```text
byte 0: schema version in low nibble; source number in high nibble
byte 1: payload length in bytes
byte 2: per-source sequence number (8-bit wrap)
bytes 3…: fixed-layout source payload
```

Integer words are little-endian. The current live payloads are:

- **Source 1, 12-byte payload (15-byte frame):** packed Wi-Fi/MQTT state,
  kernel navigation state, last command/result, uptime, publish/loss counters,
  accepted/rejected command counters, reconnect count, and outgoing queue fill.
- **Source 8, 5-byte payload (8-byte frame):** 32-bit active-map snapshot and
  the request ID that caused the readback. The snapshot includes four 5-bit
  A–Z letter indices, configured status, and map revision. Only readback
  confirms a requested update.

The dashboard can decode additional team source IDs because its decoder is
shared with the stable WiFi dashboard, but this RTOS network proof does not
publish the other subsystem telemetry. Unavailable values are not synthesized.

## Command bounds

Commands use schema version 2. Ordinary supervisory commands have a fixed
4-byte body; setting a four-slot barcode map uses a fixed 5-byte body. The
Buddy 1 receiver stages no more than 8 bytes before validation. Oversized or
malformed messages are rejected rather than accepted as valid prefixes.
Acknowledgements are fixed 4-byte messages containing the echoed command
header, request ID, result, and reported navigation state. An accepted ACK
means Buddy 1 accepted/queued the supervisory request; it is not proof that
physical movement occurred or that a task completed.

The dashboard exposes start, pause, resume, stop, reset, emergency-stop, ping,
and map set/read commands. It does not expose the protocol's direct steer,
drive, or turn command codes. In this image supervisory commands route through
Buddy 1 to the kernel navigation stub, while motor outputs remain disabled.

## Persistent Wi-Fi settings

The separate BOOTSEL provisioning helper writes a 256-byte WCF1 page:
four-byte magic, CRC32, then the fixed-size network settings structure and
unused reserved bytes. SSID, password, broker IPv4 address, and client ID are
bounded fixed-width text fields. The WCF1 structure matches the existing
interface WiFi settings page, so an existing valid page remains compatible
when flashing this network image. These credentials are not MQTT telemetry,
are not compiled into the UF2, and are not displayed by the dashboard.

## What this does and does not establish

Host tests check protocol packing/decoding and settings-page validation; the
browser smoke test checks the dashboard with deterministic mock packets. A
successful bench check should separately observe the real `Operational` link,
subscription to `is28/cmd`, Buddy 1 heartbeat, command ACK, and matching map
readback. Those observations establish the network/command/map bridge only.
They do not validate the line firmware, motion control, barcode sensing,
avoidance, closed-loop behavior, safety responses, or full project compliance;
those require later integration and physical tests.
