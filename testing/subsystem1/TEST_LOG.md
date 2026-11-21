# Buddy 1 — Wi-Fi, MQTT and dashboard

- Automated checks cover MQTT delivery and sequence tracking (`test_mqtt_delivery`),
  network-loss stop (`test_network_stop`), command dispatch (`test_release_bridge`)
  and stored log packaging (`test_release_exports`).
- Hardware procedures are T2-01, T2-09 and T2-10 in the [case table](../TRIBUNAL_2.md).
  The [team circuit run](../evidence/tribunal2/run-4/README.md) records the live
  heartbeat and telemetry window. The five-minute acceptance case also requires
  a measured delivery ratio and a separate link-loss trial.
