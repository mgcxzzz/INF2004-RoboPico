# Buddy 1 standalone bench results

Selected results from the [original ledger](../../evidence/buddy1/standalone/TEST_LOG.md), with the full 35-row history and per-run UF2 IDs there. The bench used development firmware based on `3f6ecf1` with local changes and later test images. The source commit identifies the evidence archive. Its image digests identify the bench firmware.

| Test | Tester | Result | Saved source | Observation |
| --- | --- | --- | --- | --- |
| S1-U05, S1-U06, S1-U18 | Host harness | PASS (host) | [Original](2026-10-27_S1-U05-U18_host.txt) | 21 bridge checks, 23 mapping checks and 20 C/JavaScript agreement checks. |
| S1-U07 | Team | FAIL (setup) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Two connection attempts failed at Wi-Fi authentication before MQTT. |
| S1-U07 | Team | PASS (connection) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | After the network settings were corrected, the Pico subscribed and sent five consecutive heartbeats. Initial Ready display still needed a follow-up. |
| S1-U08 | Team | FAIL (gap) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U08_native_review.json) | 300/300 heartbeats. Dashboard maximum gap 2.647 s and five gaps over 2 s. |
| S1-U08 | Team | PASS (PM-off retest) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U08_pm_off_native_review.json) | 300/300 heartbeats. Native maximum gap 1.124805 s. |
| S1-U08 | Team | PASS (standalone) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U08_final_native_review.json) | 300/300 heartbeats over 300 s. Native maximum gap 1.155431 s under Buddy 1-only load. |
| S1-U09 | Team | PASS (observed) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Start, Pause, Resume, Stop and Reset acknowledgements and state transitions agreed. |
| S1-U10 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Steer/Drive, a short command and an unknown command were rejected with Ready unchanged. |
| S1-U11 | Team | PASS (latch/reset) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Remote E-stop inhibited drive state and subsequent Start. Reset restored Ready. Physical stop latency was unmeasured. |
| S1-U12 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U12_review.json) | 10 Ping replies took 45–169 ms, mean 86.2 ms. |
| S1-U13 | Team | FAIL (display) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Broker recovered but the dashboard kept the old fault mask after Reset. |
| S1-U13 | Team | PASS (retest) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Reconnected 2.654–3.654 s after broker restart. Reset published a fresh zero fault mask. |
| S1-U14 | Team | DEFERRED | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Hotspot radio outage was deferred. The broker restart exercised broker loss. |
| S1-U15 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Last Will arrived 6.583–6.634 s after bracketed USB removal. |
| S1-U16 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | 74 heartbeats and zero ordinary telemetry frames during forwarding-off. Mapping readback remained available. |
| S1-U17 | Team | PASS (state routing) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | Physical GP20/GP21 activity appeared in navigation and Safety telemetry. |
| S1-U19 | Browser harness | PASS (simulated MQTT) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U19_dashboard_ui_final.txt) | 16 UI cases passed on the expanded browser run. |
| S1-U40 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/TEST_LOG.md) | SET/GET, revisions, duplicate validation, Reset, power-cycle, invalid payload and withheld readback were exercised. UI waited for actual readback. |
| S1-U41 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U21_live_burst.json) | 1,024 GET requests exercised overflow. The queue returned to 0% and Ping/GET worked afterwards. |
| S1-U41 | Team | PASS (bench) | [Original](../../evidence/buddy1/standalone/logs/2026-10-27_S1-U21_broker_late_review.json) | A Pico booted before the broker recovered its MQTT subscription and accepted Ping and mapping commands. |

[Saved host transcript](2026-10-27_S1-U05-U18_host.txt) · [Saved five-minute connectivity export](2026-10-27_S1-U08_final_connectivity.txt)
