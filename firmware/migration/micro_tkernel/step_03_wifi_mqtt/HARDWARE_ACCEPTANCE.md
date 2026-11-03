# Step 3A stopped network checks — 2026-11-04

**Steady baseline and functional broker-reconnect checks passed.** This accepts
the stopped network checkpoint on the tested Pico/environment, not moving
supervision or complete Buddy1–4 migration. Firmware was unchanged:
`robopico_rtos_network_bench_v0_1.uf2`, release4be0f44, SHA256
`93e2be079f6e3b85d1f1940d311a49abddbc7d91d9f5bbfec3ee6afa89a8fb31`.

## Steady baseline

User evidence: `RTOS_STEP3A_network_baseline_2026-11-04.zip`, also included in
`RTOS_STEP3A_network_baseline_and_reconnect_2026-11-04.zip`.

- 79 broker-received heartbeats, maximum observed gap1.162011seconds.
- 74.00215seconds laptop time versus74.0kernel/73.999997hardware TIMER seconds.
- Control99.997Hz; no new late intervals or USB drops.
- Connected/subscribed throughout; polls39687 ->64309, published123 ->203,
  received2 ->5. TX drops stayed2, RX0. These are not zero lifetime TX drops.
- Ping, mapping and mapping GET acknowledged. Baseline text alone did not
  establish exact mapping values; the subsequent binary readback did.

## Broker interruption and recovery

Evidence: reconnect_2026-11-04_01-04-56 review JSON, USB and broker captures.
The broker was stopped, both1883/9001 listeners closed, then restarted about
10seconds after the stop request. The Pico remained powered.

- First received heartbeat approximately2.226seconds after restart launch.
- Broker capture confirms the Pico reconnecting and subscribing to is28/cmd;
  later USB reports connected1/subscribed1 with advancing polls/published.
- Before/after source8 packets independently decode to snapshot0x00318820,
  configured1, revision1, lettersABCD in Straight/U-turn/Left/Right order.
- Uptime617 ->638seconds across approximately21.076laptop seconds; no reboot
  indicated. Motors explicitly DISABLED throughout observed status.
- Excluding the first2seconds of stale buffered USB output: control100.008Hz,
  kernel25.0seconds, hardware25.00015seconds, laptop25.00121seconds; no new late
  intervals or USB drops.
- TX drop count2 ->12 during the intentional outage/stale-message cleanup;
  RX remained0. This is distinct from the uninterrupted no-new-drop baseline.

## Observations carried into Step3B

- Bench heartbeat advertises state1, which the dashboard labels Running even
  though outputs are inhibited. Correct the operating-state encoding when
  attaching motion; do not interpret the label as a physical motor test.
- The reconnect observer has a2.507second receive gap followed by buffered
  heartbeats. The broker's longer log also contains two3-second timestamp gaps
  and bursts. Cause is not established; do not certify uninterrupted <=2second
  heartbeat performance from this reconnect capture. Check recovery jitter
  and moving network timing during Step3B.
- This test establishes broker recovery, not AP/hotspot-loss recovery or safe
  driving after link loss. Dashboard Start/Pause/Resume motion and telemetry
  remain Step3B work. Mapping survives broker reconnection in RAM; persistence
  through reboot was not claimed or tested.
- Step2 left/right and sharp-bend acceptance remains pending sufficient mat.

No calibration changes or new firmware are needed to preserve these results.
