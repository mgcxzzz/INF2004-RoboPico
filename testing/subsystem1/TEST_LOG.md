# Subsystem 1 — Wi-Fi, MQTT and dashboard

[Network bridge](../../app_program/network_bridge/) and [dashboard tests](../../dashboard/test/).
Run the named programs using the [test runner](../README.md#run-the-tests).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S1-U20 | MQTT queue and delivery bookkeeping | FR03, NFR03 | auto | `test_mqtt_delivery` | Priority traffic retains headroom; sequence tracking and disconnect handling pass. |
| S1-U21 | Network stop snapshot | FR03, FR09 | auto | `test_network_stop` | A qualified network stop latches and exports its reason in C6. |
| S1-U22 | Stored log codec | FR10 | auto | `test_release_exports` | Stopped transfers preserve row identity, chunk bounds and generation. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | S1-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Queue headroom, wrap, priority and disconnect cancellation exercised. |
| 2026-11-21 | S1-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Guard boundaries, failure mask, latch and clear passed. |
| 2026-11-21 | S1-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Stopped gate, frozen lease, row formats and chunk bounds passed. |
