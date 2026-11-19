# RoboPico v0.2.10 network stop test build

This candidate on `barr-c-cleanup` fixes a false stale-bridge stop caused by
reading the clock before the cross-core timestamps. The control and safety
tasks now both retain the failed network guard check in C6 before stopping.
The first recorded failure survives reconnect and is cleared on a new run.
The wire format, stop limits and v0.2.9 calibration fixes are preserved.

The 19 November v0.2.9 baseline showed 21 heartbeat gaps over 2 seconds,
one broker disconnect, and one stop while driving without a recorded C6 cause.
This patch addresses the verified timestamp race and missing evidence. The
underlying heartbeat delivery delays still need hardware diagnosis.

## Reproduce on the same hotspot

1. Extract this bundle and flash `firmware/robopico_buddy1_4_rtos_v0_2_10_network_stop.uf2`.
   Open the bundled dashboard and confirm both version labels show v0.2.10.
2. Start **Capture wireless test.cmd**. On macOS use
   `python3 tools/rtos_migration/capture_wireless.py` from the bundle folder.
   Leave the robot idle for 30 seconds, then Start and drive the same route
   with normal telemetry and the same hotspot and power setup.
3. If it stops unexpectedly, leave the capture running for another 20–30
   seconds. Save the **Event CSV** from **Logs & exports** before a new Start
   or power cycle. The C6 card is near the bottom of **Telemetry → Barcode
   and junction progress**. The raw MQTT capture also includes C6.
4. Send the matching MQTT CSV and summary JSON from `rtos_wireless_session`,
   the event CSV, and the approximate time and whether the robot stopped or
   only the badge went stale. A clean run is useful for comparison too.

The native regression injects a cross-core update between the clock and
timestamp reads. It also checks that a truly stale bridge still records the
first C6 cause, and that later failures and read-only probes preserve it.
The ARM build and package verify the existing firmware profile, bounds and
integrity. Physical validation is pending.
