# Subsystem 1 — Wi-Fi, MQTT and dashboard

[Network bridge](../../app_program/network_bridge/) and [dashboard tests](../../dashboard/test/).
Run the named programs using the [test runner](../README.md#run-the-tests).

Standalone cases S1-U01–S1-U19 retain their original IDs. Standalone mapping and MQTT stress are S1-U40 and S1-U41 here, recorded as S1-U20 and S1-U21 in the original ledger. Their standalone
firmware and test harness are pinned in the [archived standalone ledger](../evidence/buddy1/standalone/TEST_LOG.md).
The current combined firmware supervises the network and requires an explicit
Start after a qualified link stop.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| S1-U01 | command checking on the Pico: START accepted (a), DRIVE rejected as prohibited (b), 3-byte message rejected as bad format (c) | FR02 | auto | `subsystem1_test.c` (`make TEST=1`) | `[TEST] S1-U01a/b/c PASS` |
| S1-U02 | heartbeat frame packs and decodes again (team schema, source 1, 12-byte payload) | FR03 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | a prohibited command (DRIVE) is counted as rejected and never reaches the Navigation Coordinator | FR02 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS`, Nav state unchanged |
| S1-U04 | remote ESTOP goes straight to the Safety Supervisor and latches (a); RESET clears it (b) | FR09 | auto | `subsystem1_test.c` | `[TEST] S1-U04a/b PASS` |
| S1-U05 | host tests: every command code, CONFIG values, wrong length / version, acknowledgement packing, heartbeat clamping and round trip, back-off 1-2-4-8 s, queue fill % | FR02, FR03 | auto (PC) | `make -C testing/subsystem1/host` | `21 passed, 0 failed` |
| S1-U06 | the dashboard decoder reads the same bytes the C code writes (heartbeat, Nav, Safety, command, short frame, sequence gaps and restarts), plus mapping and connectivity verdict checks | FR03 | auto (PC) | `make -C testing/subsystem1/host vectors`, then `node check_dashboard.mjs` | `D01`–`D20` PASS |
| S1-U07 | connects to the broker: Disconnected → Connecting → Operational; heartbeat arrives on `is28/hb` | FR03 | manual | [Part 3](../evidence/buddy1/standalone/HOW_TO_TEST.md) | console `mqtt: Operational`; dashboard *car: online*, heartbeat age < 2 s |
| S1-U08 | 5-minute connectivity test: messages delivered and largest heartbeat gap | NFR03 | manual | dashboard *Start 5-minute test*, then *Save results* | ≥ 95 % delivered per source; largest gap ≤ 2 s; result `PASS` |
| S1-U09 | supervisory commands from the dashboard: Start, Pause, Resume, Stop each answered | FR02 | manual | dashboard buttons, read the ack line and the operating state | ack `accepted`; operating state follows (Running, Paused, Running, Safe Stop) |
| S1-U10 | steering and drive commands, wrong length and unknown code are rejected and never acted on | FR02 | manual | dashboard *Send STEER*, *Send DRIVE*, *Bad length*, *Unknown code* | acks `prohibited`, `prohibited`, `format`, `unknown`; operating state unchanged; *rejected* count + 4 |
| S1-U11 | remote emergency stop and reset | FR09 | manual | during a (simulated) run press *EMERGENCY STOP*, then *Reset* | Safety ESTOP, operating state Emergency Stop; after Reset back to Ready |
| S1-U12 | command round trip time (dashboard → car → ack) | FR02 | manual | press *Ping* 10 times, read the latency shown | each ack within 500 ms (record the values) |
| S1-U13 | Broker loss and reconnect on the standalone Buddy 1 bench | FR03 | manual | stop Mosquitto for 20 s, start it again | Reconnect with backoff; confirm dashboard state refresh after Reset. |
| S1-U14 | Hotspot outage and recovery on standalone Buddy 1 | FR03 | manual | switch the hotspot off for 20 s during a run, then on | Verify the legacy warning and Reset behavior described by the branch procedure. |
| S1-U15 | broker's last will: dashboard told when the car vanishes | FR03 | manual | unplug the Pico W | dashboard *car offline (broker last will)* within about 8 s |
| S1-U16 | telemetry forwarding switched off and on remotely (CONFIG) | FR03 | manual | *Telemetry off*, wait 5 s, *Telemetry on* | no `is28/tlm` frames while off (heartbeat continues); frames resume when on |
| S1-U17 | GP20 / GP21 buttons still work with WiFi running (bench) | FR09 | manual | press GP20, then GP21 | ESTOP, then reset and start; dashboard shows both |
| S1-U18 | configuration parser/lookup, all 358800 distinct letter assignments, malformed/oversized commands, readback, revision wrap and idempotent retries | FR10 | auto (PC) | `make -C testing/subsystem1/host` runs `test_mapping` | M01–M23 PASS; no boot mapping or partial replacement |
| S1-U19 | actual dashboard UI with simulated MQTT, C/JS mapping agreement and trustworthy connectivity verdicts | FR03, FR10 | auto (PC/browser) | `check_dashboard.mjs`, `check_dashboard_ui.mjs`; [Part 7](../evidence/buddy1/standalone/HOW_TO_TEST.md) | D01–D20 and UI01–UI16 PASS; no success on send/ack alone, invalid input rejected, desktop/mobile layout clean |
| S1-U40 | real Pico mapping SET/GET, changed/identical update, invalid payload, telemetry-off, RESET and power-cycle behaviour | FR10 | manual + automated live MQTT | [Parts 7–8](../evidence/buddy1/standalone/HOW_TO_TEST.md) | ack + exact active readback, invalid input preserves old map, RESET retains, power-cycle clears; lost dashboard readback must not falsely confirm SET |
| S1-U41 | real MQTT bridge startup, reconnect/resubscription, fragmented/oversized input and full-queue behaviour | FR03, FR10 | manual + automated live MQTT; source inspection for chunk boundaries | [Parts 7–8](../evidence/buddy1/standalone/HOW_TO_TEST.md) | bounded copied buffers, no prefix applied, no task stall; counted drops and queue recovery after finite burst; mapping commands work after reconnect |
| S1-U20 | MQTT queue and delivery bookkeeping | FR03, NFR03 | auto | `test_mqtt_delivery` | Priority traffic retains headroom; sequence tracking and disconnect handling pass. |
| S1-U21 | Network stop snapshot | FR03, FR09 | auto | `test_network_stop` | A qualified network stop latches and exports its reason in C6. |
| S1-U22 | Stored log codec | FR10 | auto | `test_release_exports` | Stopped transfers preserve row identity, chunk bounds and generation. |
| S1-U42 | RTOS broker startup and recovery | FR03, NFR01 | hardware bench | Restart broker with the Step 3A Pico powered and drive inhibited; record heartbeat and mapping readback. | Heartbeat and subscription resume, uptime and active mapping are retained. |
| S1-U43 | Supervised hotspot heartbeat gaps | FR03, NFR03 | hardware capture | Record a live native MQTT session and compute receiver gaps from heartbeat timestamps. | Maximum complete-window heartbeat gap is at most 2 s; investigate and retain any fail. |
| S1-U44 | One-click run export on car | FR10 | hardware + dashboard | Stop and export available stored datasets and live dashboard history. Record complete and interrupted export outcomes. | ZIP has all four datasets on success. Interrupted transfer is visibly partial and retains available capture. |
| S1-U45 | Integrated circuit telemetry receipt | FR03, NFR03 | hardware + dashboard | Record heartbeat and telemetry receipt over the circuit evidence window. | Motion and events reach the dashboard. Record actual heartbeat gap and measure formal delivery separately. |
| S1-U46 | Earlier SDK Wi-Fi runtime under telemetry | FR03 | hardware diagnosis | Review the SDK watchdog and lwIP timeout reports from bench and hotspot trials. | Capture runtime fault and host timer-pool regression evidence under the recorded versions. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-10-27 | S1-U05, S1-U06, S1-U18 | Standalone bench | Host harness | PASS (host) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 21 bridge checks, 23 mapping checks and 20 C/JavaScript agreement checks. |
| 2026-10-27 | S1-U07 | Standalone bench | Team | FAIL (setup) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Two connection attempts failed at Wi-Fi authentication before MQTT. |
| 2026-10-27 | S1-U07 | Standalone bench | Team | PASS (connection) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | After the network settings were corrected, the Pico subscribed and sent five consecutive heartbeats. Initial Ready display still needed a follow-up. |
| 2026-10-27 | S1-U08 | Standalone bench | Team | FAIL (gap) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 300/300 heartbeats. Dashboard maximum gap 2.647 s and five gaps over 2 s. |
| 2026-10-27 | S1-U08 | Standalone bench | Team | PASS (PM-off retest) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 300/300 heartbeats. Native maximum gap 1.124805 s. |
| 2026-10-27 | S1-U08 | Standalone bench | Team | PASS (standalone) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 300/300 heartbeats over 300 s. Native maximum gap 1.155431 s under Buddy 1-only load. |
| 2026-10-27 | S1-U09 | Standalone bench | Team | PASS (observed) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Start, Pause, Resume, Stop and Reset acknowledgements and state transitions agreed. |
| 2026-10-27 | S1-U10 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Steer/Drive, a short command and an unknown command were rejected with Ready unchanged. |
| 2026-10-27 | S1-U11 | Standalone bench | Team | PASS (latch/reset) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Remote E-stop inhibited drive state and subsequent Start. Reset restored Ready. Physical stop latency was unmeasured. |
| 2026-10-27 | S1-U12 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 10 Ping replies took 45–169 ms, mean 86.2 ms. |
| 2026-10-27 | S1-U13 | Standalone bench | Team | FAIL (display) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Broker recovered but the dashboard kept the old fault mask after Reset. |
| 2026-10-27 | S1-U13 | Standalone bench | Team | PASS (retest) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Reconnected 2.654–3.654 s after broker restart. Reset published a fresh zero fault mask. |
| 2026-10-27 | S1-U14 | Standalone bench | Team | DEFERRED | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Hotspot radio outage was deferred. The broker restart exercised broker loss. |
| 2026-10-27 | S1-U15 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Last Will arrived 6.583–6.634 s after bracketed USB removal. |
| 2026-10-27 | S1-U16 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 74 heartbeats and zero ordinary telemetry frames during forwarding-off. Mapping readback remained available. |
| 2026-10-27 | S1-U17 | Standalone bench | Team | PASS (state routing) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | Physical GP20/GP21 activity appeared in navigation and Safety telemetry. |
| 2026-10-27 | S1-U19 | Standalone bench | Browser harness | PASS (simulated MQTT) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 16 UI cases passed on the expanded browser run. |
| 2026-10-27 | S1-U40 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | SET/GET, revisions, duplicate validation, Reset, power-cycle, invalid payload and withheld readback were exercised. UI waited for actual readback. |
| 2026-10-27 | S1-U41 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | 1,024 GET requests exercised overflow. The queue returned to 0% and Ping/GET worked afterwards. |
| 2026-10-27 | S1-U41 | Standalone bench | Team | PASS (bench) | [Bench record](logs/2026-10-27_buddy1_standalone.md) | A Pico booted before the broker recovered its MQTT subscription and accepted Ping and mapping commands. |
| 2026-11-21 | S1-U20 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Queue headroom, wrap, priority and disconnect cancellation exercised. |
| 2026-11-21 | S1-U21 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Guard boundaries, failure mask, latch and clear passed. |
| 2026-11-21 | S1-U22 | `8aa5f2c` | Release verification | PASS (host) | [Saved output](logs/2026-11-21_v0.4.3_host.txt) | Stopped gate, frozen lease, row formats and chunk bounds passed. |
| 2026-10-28 | S1-U46 | SDK network baseline | Team | FAIL (runtime) | [Saved log](../evidence/buddy2/network_history/timeout_pool_panic_2026-10-28.txt) | The SDK trace records the timeout pool panic. See the [host regression](../evidence/buddy2/network_history/lwip_timeout_pool_regression.txt) and [separate hotspot stall capture](../evidence/buddy2/network_history/hotspot_stall_2026-10-28.txt). |
| 2026-11-04 | S1-U42 | `4be0f44` | Team | PASS (stopped bench) | [Saved log](../../firmware/migration/micro_tkernel/step_03_wifi_mqtt/HARDWARE_ACCEPTANCE.md) | 79 broker heartbeats, 1.162011 s maximum gap. First heartbeat after broker restart about 2.226 s. Uptime and ABCD mapping persisted. |
| 2026-11-19 | S1-U43 | v0.2.9 | Team | FAIL (observed gap) | [Saved log](logs/2026-11-19_v0.2.9_heartbeat_capture.json) | 186 heartbeats, 15.667822 s maximum gap in the interrupted capture. [v0.2.10 fix](https://github.com/mgcxzzz/INF2004-RoboPico/commit/614bebb) addressed a reproducible guard clock race and added stop diagnostics. The RF cause remains unresolved. |
| 2026-11-21 | S1-U44 | v0.3.12 | Team | PARTIAL | [Saved log](logs/2026-11-21_partial_wireless_export.json) | 640 motion rows transferred. Diagnostics reported busy after 170 rows. The ZIP retained dashboard history and marked the result partial. |
| 2026-11-21 | S1-U44 | v0.4.2 | Team | PASS (run 4) | [Saved log](../system/logs/2026-11-21_run_4/README.md) | One on-car run ZIP contained all four stored datasets and live dashboard history. |
| 2026-11-21 | S1-U45 | v0.4.2 | Team | PASS (receipt) | [Saved log](../system/logs/2026-11-21_run_4/mqtt.txt) | 16,749 received telemetry frames and 370 heartbeats. Largest observed gap 1.218 s. Sequence-counted delivery was outside this capture's scope. |
