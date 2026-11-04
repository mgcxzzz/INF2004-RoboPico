# Step3B v0.1.1 — encoder guard and network delivery retest

Experimental migration checkpoint; hardware acceptance pending. Main is
unchanged. This package contains BOTH changes discussed after the November4
Start/Pause/Resume tests. It remains USB-supervised line following, not the
complete Buddy1–4 firmware. Barcode decoding is Step4, IMU/ramp assist Step5.

## Changes

The previous run stopped with reason9/flag0x40 when the right encoder reached
eight invalid two-bit transitions across the entire run. In the Pause/Resume
test it had six before Pause and reached eight after Resume. Recorded speeds
were forward: this was not the wrong-direction guard. Isolated errors could
remain counted forever under that old rule.

The new owner-only check retains raw lifetime counts for diagnosis, but trips
on either eight invalid transitions within100ms, or at least eight in a rolling
window AND at least5% of observed state changes invalid. Ten100ms buckets
cover the last900–1000ms. Error timestamps also catch bursts crossing a bucket
boundary. Pause ages the window without resetting lifetime totals; new Start
establishes a fresh baseline. The original wrong-direction check (>20mm/s
opposite target for50ms) remains. These quality thresholds are provisional
until the hardware retest.

Network changes address measured drops/heartbeat gaps and confirmed lwIP
capacity limits; they are not proof that every gap came from those limits.
The old build had four MQTT requests and a256-byte output ring. This build
uses16 requests/1024 bytes, disables TCP Nagle batching after connection, and
reserves two requests/128 output bytes from telemetry for ACKs/heartbeat.
It also checks TCP send headroom before admitting telemetry.

Copied source9 motion and source11 navigation keep one latest-sample slot
each. An unconsumed sample is replaced and counted as `coalesced`: deliberate
live-stream loss, not a full motion recording. Mapping replies, heartbeat and
command ACKs retain separate bounded buffers. ACK/HB have priority. ERR_MEM
telemetry cannot remain pending ahead of them. Stale telemetry expires after
250ms; ACK/HB retries remain limited to2s. MQTT remains QoS0. `published`
means accepted by lwIP, not confirmed delivered to the broker.

Only the professor-style kernel Wi-Fi owner calls lwIP/MQTT. Subsystems use
copied bounded data; owner queue operations remain non-blocking. The existing
extra four lwIP timer slots,1Hz heartbeat,5s MQTT keepalive, network-loss stop
and no-auto-restart behavior are retained. Dashboard stale limits have not
been extended to conceal delays.

Continuous V2 steering, gains, nominal profile1 speed45mm/s, PWM caps, line
thresholds, calibration priority and Step2 junction experiment are unchanged.
Normal UF2 flashing retains Wi-Fi/calibration sectors. Your car should still
show white122,138 / black246,267; check actual readback. Do not recalibrate
or press c for this comparison.

## Set up

1. Extract `robopico_rtos_connected_line_v0_1_1_bundle.zip` into a NEW folder.
   Keep v0.1 for rollback. Close the old dashboard/server terminal.
2. Stop the robot, remove motor/battery power and close serial programs.
   Enter BOOTSEL and copy `firmware/robopico_rtos_connected_line_v0_1_1.uf2`
   onto RPI-RP2. This is the only application reflash needed.
3. Run **Start dashboard.cmd** from the NEW folder; keep its terminal open.
   Open http://localhost:8765/ and connect to ws://localhost:9001. Hard-refresh
   if cached. Use one robot and one dashboard for this baseline.
4. Reuse saved Wi-Fi if SSID/password and laptop broker IPv4 are unchanged.
   Otherwise run **Setup Wi-Fi.cmd**, entering BOOTSEL only when prompted.
   Its settings-only UF2 does not require an application reflash afterward.
5. Close PuTTY. Run **Capture connected test.cmd**, choose COM4 or actual
   port, and keep it actively reading throughout the test. Mac equivalent:
   `python3 tools/rtos_migration/capture_connected_line.py` with the full
   `/dev/cu.usbmodem...` port. USB DTR supervision is STILL REQUIRED.
6. Check the captured USB text: connected-line v0.1.1 banner, CLOCK PASS,
   LINE flags0x3, usb3, NET connected1/subscribed1, advancing polls/last_ms.
   `NET_TX ... nagle_off=1` confirms the TCP setting. Check CAL_FRONT matches
   the previous car calibration before driving.

Capture defaults to300seconds and sends no robot commands. Stop the robot
FIRST, then Ctrl+C to save early, or let capture finish. A finished capture
window is not an active serial connection; reopen capture before another
Start. Never run PuTTY and capture on the same COM port.

## Test A — heartbeat without forced refreshes

With motor power off and no Start, capture at least130seconds: Telemetry off
for60seconds, then on for60seconds. Avoid Ping, mapping and repeated commands
within those intervals because commands can force extra heartbeats. Heartbeat
should continue about once per second in BOTH phases, normally no received
gap above2seconds without an intentional outage. Then separately try Ping
and mapping apply/readback. Preserve the capture if it disconnects.

## Test B — short straight, Pause/Resume and stored export

1. Fresh capture, ordinary tested motor power, loose USB cable, sensors fully
   over a straight on the mat. Wait for CLOCK PASS, released buttons and the
   dashboard online for at least1second.
2. Start; allow the2second countdown. Once moving, run about4seconds, Pause
   for2seconds, Resume for about4seconds, then Stop. Do not click Start while
   Paused. Video this test if practical, showing the full car.
3. Keep capture open while stopped until at least30seconds were captured,
   then Ctrl+C. Keep the Pico powered. Do not Reset/reboot/reflash/new Start.
4. Immediately run **Export line log.cmd** and choose the same port. It sends
   d only. Send its three files from **rtos_line_session**. The RAM history
   holds the last1536 samples (~15.36s), including the terminal stopped row.

If successful, separately try30–60seconds of flat tracking where mat space
permits. This tests that sparse errors no longer accumulate to a whole-run
cutoff. Capture the entire run and export it separately after Stop. The RAM
export contains only the latest15.36seconds; USB/MQTT capture covers the run.
Do not infer sharp-bend or L/R-junction acceptance from straight tracking.

Once A/B pass, separately recheck GP21 Stop, GP20/dashboard E-stop and eligible
Reset. Repeat broker/AP loss and reconnect with wheels lifted first. It must
stop on detected loss, reconnect/resubscribe and remain stopped until a new
explicit Start. Network loss detection is not a guaranteed250ms RF-loss
deadline; GP20/GP21 remain the local stop paths.

## Logs to send and new fields

- From each **rtos_connected_session** capture: `_usb.txt`, `_mqtt.csv`,
  `_summary.json`, now including delivery and encoder guard evidence.
- From each stopped **rtos_line_session** export: `_usb.txt`, `_line.csv`,
  `_summary.json`. Stored100Hz CSV/binary format is unchanged.
- Video of TestB helps assess steering; TestA needs no video. State which
  command was clicked and whether an outage was intentional. Do not send
  passwords or settings-only UF2 files.

`ENCODER_GUARD` reports recent bad/observed changes and retained trip cause:
0=none,1=burst,2=density,3=wrong direction; wheel1=left,2=right. `ENCODER_TRIP`
prints the exact cause/window at a stop. Evidence remains until next Start.
WHEELS invalid values are still raw lifetime counts.

`NET_TX` reports `mem` (publish ERR_MEM attempts), `errors`, `deferred` (polls
without telemetry headroom), `coalesced` (replaced live samples), `free_req`,
`ring_used`, `tcp_free`, `nagle_off`, `admit_ms` (last publish accepted by
lwIP), and `complete_ms` (QoS0 TCP acknowledgement activity, NOT individual
broker delivery). These are copied owner counters, never application reads
of lwIP internals. Coalesced/deferred values alone do not prove heartbeat
failure; compare them with actual received gaps, ACK latency and reconnects.

The summary still says REVIEW_PHYSICAL_TEST. Connected motion no longer
falsely marks healthy clock timing FAIL; latched motor faults remain separate
readiness blockers. Hardware acceptance of these fixes is still pending.
