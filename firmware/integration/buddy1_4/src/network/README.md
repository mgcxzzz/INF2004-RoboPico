# Wi-Fi and MQTT — Buddy 1 transport

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Ownership and buffers

The working SDK architecture is retained: core0 produces copied binary telemetry and consumes validated command data; core1 owns Wi-Fi, lwIP and MQTT. Subsystem logic never invokes MQTT callbacks directly. Fixed-size TX/RX queues bound memory, copy callback-owned data before returning and count drops. Local provisioning settings are stored separately from motion logs.

Startup loads the saved configuration, joins a2.4GHz access point, waits for an IP address, establishes TCP/MQTT and restores subscriptions after reconnect. TCP connection success alone is not MQTT readiness: CONNACK and SUBACK must complete. `n` exposes link, polling, MQTT stage, counters and provision status. Retry/timeout paths keep the control loop independent.

Heartbeat normally publishes once per second. Motion snapshots normally publish20Hz. Command IDs and ACK/result state are distinct from reported operating state. Runtime A–Z mapping is read back by the dashboard; changing it does not rewrite an already-queued action's captured revision. Closing the Wi-Fi configuration helper is normal; the dashboard/broker helper stays open while the car needs the laptop broker.

## Preserved fixes

`config/lwipopts.h` reserves `LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4` timeout entries. The original MQTT cyclic-timer allocation failure and the regression log are retained in the evidence archive. The build reserves two disjoint8KiB stacks with heap/BSS below them. The SDK startup/timebase remains; this is not the earlier failing UTK Drive timebase experiment.

Public artifacts are built with no local Wi-Fi credentials header. Configure SSID/password/laptop LAN IPv4 using `tools/provisioning/provision_wifi.py` with the Pico on USB, stopped and settled. Then normal saved settings allow wireless running from its usual power source. Reflashing does not normally erase the saved Wi-Fi sector. Never commit `wifi_config.local.h` or private provisioning files.

## Evidence and tests

Host tests compile the real transport with boundary-only SDK stubs and test connection stages, reconnect/subscription, timeout paths, queue behavior and provisioning. Helper tests mock broker/serial access. These tests do not prove the newly restructured firmware has connected on the car; new heartbeat-under-motion hardware acceptance is pending. The complete bundle includes the dashboard, vendored MQTT.js, local broker configuration and Windows launchers.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
