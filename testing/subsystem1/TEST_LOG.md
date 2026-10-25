# Subsystem 1 — test log

**Subsystem:** WiFi and MQTT: supervisory commands, heartbeat and telemetry  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results, and
**`HOW_TO_TEST.md`** (this folder) for step-by-step instructions for every
test below.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | command checking on the Pico: START accepted (a), DRIVE rejected as prohibited (b), 3-byte message rejected as bad format (c) | FR-WIFI-02, FR-WIFI-03 | auto | `subsystem1_test.c` (`make TEST=1`) | `[TEST] S1-U01a/b/c PASS` |
| S1-U02 | heartbeat frame packs and decodes again (team schema, source 1, 12-byte payload) | NFR-WIFI-02, IF-13 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | a prohibited command (DRIVE) is counted as rejected and never reaches the Navigation Coordinator | FR-WIFI-03, IF-12 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS`, Nav state unchanged |
| S1-U04 | remote ESTOP goes straight to the Safety Supervisor and latches (a); RESET clears it (b) | FR-WIFI-02, IF-12 | auto | `subsystem1_test.c` | `[TEST] S1-U04a/b PASS` |
| S1-U05 | host tests: every command code, CONFIG values, wrong length / version, acknowledgement packing, heartbeat clamping and round trip, back-off 1-2-4-8 s, queue fill % | FR-WIFI-02, FR-WIFI-03, NFR-WIFI-02 | auto (PC) | `make -C testing/subsystem1/host` | `21 passed, 0 failed` |
| S1-U06 | the dashboard decoder reads the same bytes the C code writes (heartbeat, Nav, Safety, command, short frame, sequence gaps and restarts) | IF-13 | auto (PC) | `make -C testing/subsystem1/host vectors`, then `node check_dashboard.mjs` | `D01`–`D06` PASS |
| S1-U07 | connects to the broker: Disconnected → Connecting → Operational; heartbeat arrives on `is28/hb` | statechart 6.1 | manual | HOW_TO_TEST Part 3 | console `mqtt: Operational`; dashboard *car: online*, heartbeat age < 2 s |
| S1-U08 | 5-minute connectivity test: messages delivered and largest heartbeat gap | NFR-WIFI-01, NFR-WIFI-02 | manual | dashboard *Start 5-minute test*, then *Save results* | ≥ 95 % delivered per source; largest gap ≤ 2 s; result `PASS` |
| S1-U09 | supervisory commands from the dashboard: Start, Pause, Resume, Stop each answered | FR-WIFI-02, IF-12 | manual | dashboard buttons, read the ack line and the operating state | ack `accepted`; operating state follows (Running, Paused, Running, Safe Stop) |
| S1-U10 | steering and drive commands, wrong length and unknown code are rejected and never acted on | FR-WIFI-03 | manual | dashboard *Send STEER*, *Send DRIVE*, *Bad length*, *Unknown code* | acks `prohibited`, `prohibited`, `format`, `unknown`; operating state unchanged; *rejected* count + 4 |
| S1-U11 | remote emergency stop and reset | FR-WIFI-02, NFR-SYS-04 | manual | during a (simulated) run press *EMERGENCY STOP*, then *Reset* | Safety ESTOP, operating state Emergency Stop; after Reset back to Ready |
| S1-U12 | command round trip time (dashboard → car → ack) | – | manual | press *Ping* 10 times, read the latency shown | each ack within 500 ms (record the values) |
| S1-U13 | broker lost and back: car reconnects with back-off, keeps running | statechart 6.1 | manual | stop Mosquitto for 20 s, start it again | console Reconnect Wait, retries 1, 2, 4, 8 s apart; Operational again within 10 s of the broker returning; operating state unchanged |
| S1-U14 | WiFi lost and back: link loss is a warning, not a fault; fault bit cleared by Reset | statechart 6.1, IF-15, IF-17 | manual | switch the hotspot off for 20 s during a run, then on | car keeps its operating state; after reconnect the fault mask shows *WiFi/broker lost*; *Stop* then *Reset* clears it |
| S1-U15 | broker's last will: dashboard told when the car vanishes | – | manual | unplug the Pico W | dashboard *car offline (broker last will)* within about 8 s |
| S1-U16 | telemetry forwarding switched off and on remotely (CONFIG) | – | manual | *Telemetry off*, wait 5 s, *Telemetry on* | no `is28/tlm` frames while off (heartbeat continues); frames resume when on |
| S1-U17 | GP20 / GP21 buttons still work with WiFi running (bench) | IF-12 | manual | press GP20, then GP21 | ESTOP, then reset and start; dashboard shows both |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (console, dashboard, phone video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
