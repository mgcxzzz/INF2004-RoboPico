# How to test Buddy 1 (WiFi and MQTT), step by step

This guide tells you what to do for every Buddy 1 test in `TEST_LOG.md`, in
order. Part 1 needs only a computer. Parts 2–5 need a Pico W (on the Maker Pi
Pico or the Robo Pico) and a computer running the MQTT broker. The wheels
never turn in this guide (`SYS_SIM_CAR` is 1). How to record a result is in
Part 6 and in `../README.md`.

---

## Safety first

**Default setup, confirmed 27 October 2026:** use the car's Pico W on the
Robo Pico with all peripherals connected. Do not assume a separate Maker
Pi Pico bench board or ask for sensors to be unplugged unless the user
specifies a different setup. All three IR sensors use analog outputs;
their exact ADC-channel mapping still needs confirmation when S3 is tested.

On the current standalone `buddy1` branch, the linked `mc_*` functions
are no-output weak stand-ins, and template S2/S3/S4 tasks do not access
sensor/motor pins. Bench glue only configures GP20/GP21 as button inputs;
USB console builds keep the UART mirror off (`CRASH_REPORT=0`). Kernel
sample I2C is disabled and sample ADC pin-initialisation switches are all
off. This is a source/build check, not a measured electrical test.
Recheck ownership after any later subsystem integration. This branch
cannot establish a physical motor-stop deadline using the stand-ins.

- **Power: two USB sockets, one rule.** The Pico W's own USB socket may go
  to the computer (console and flashing). The Robo Pico's power input goes
  to the power bank. **Never both at the same time**, and **never plug the
  Robo Pico's power input into the computer.** For this guide only the Pico
  W's USB is needed.
- Keep `SYS_SIM_CAR` = 1 in `app_program/system/sys_config.h`: the motors are
  never driven, so a START from the dashboard cannot make the car drive off
  the desk.

## How the pieces fit

```
 Pico W (car)  --WiFi, MQTT port 1883-->  Mosquitto on your computer
                                              |
 browser: tools/dashboard/index.html  <--WebSockets, port 9001
```

| Topic | Direction | What |
|---|---|---|
| `is28/hb` | car → dashboard | Buddy 1 heartbeat, about once a second (source 1, `system/sys_tlm.h`) |
| `is28/tlm` | car → dashboard | every other buddy's telemetry frame, unchanged; source 8 is barcode mapping readback |
| `is28/cmd` | dashboard → car | existing 4-byte commands; BARCODE_MAP is 5 bytes, BARCODE_GET is 4 |
| `is28/ack` | car → dashboard | answer to every command: accepted or why it was rejected |
| `is28/lwt` | broker → dashboard | the broker's last will: the car vanished |

## What you need

| Item | For |
|---|---|
| Car Pico W on Robo Pico, all peripherals connected, USB from the **Pico W's** socket to the computer | Parts 2–5 |
| A **2.4 GHz** WiFi network that the Pico W and the computer both join (phone hotspot works: on iPhone turn on *Maximise Compatibility*) | Parts 2–5 |
| Mosquitto on the computer (below) | Parts 2–5 |
| Internet on the computer the first time the dashboard opens (it loads MQTT.js from unpkg.com) | Parts 3–5 |
| `gcc` and `node` on the computer | Part 1 |

The robot hardware (motors, sensors) is not needed.

---

## Part 1 — host tests on the computer (S1-U05, S1-U06)

No Pico needed.

```sh
cd testing/subsystem1/host
make                    # S1-U05: builds and runs test_b1
make vectors            # writes vectors.json from the C code
node check_dashboard.mjs  # S1-U06: dashboard decoder against the C bytes
```

Expected: baseline `21 passed, 0 failed`, mapping `23 passed, 0 failed`,
seven Safety mask-regression checks (zero failures), then `D01` … `D20`
PASS. The Safety harness exercises the production event and periodic
handlers with host kernel/navigation stubs; it is regression evidence for
S1-U13, not a physical Safety timing test. Save the output
in `../logs/` (for example `make 2>&1 | tee ../logs/2026-10-26_S1-U05_host.txt`).

## Part 2 — broker, WiFi settings, build

### 2.1 Start Mosquitto on the computer

macOS:

```sh
brew install mosquitto                      # once
mosquitto -c tools/dashboard/mosquitto.conf -v
```

(If `mosquitto` is not found, use `$(brew --prefix)/sbin/mosquitto`.) Run it
from the repo root and leave the window open: `-v` prints every connection,
which helps. If macOS asks whether to accept incoming connections, choose
**Allow**. The config opens port 1883 for the car and 9001 for the
dashboard, with no password, so use it only on your own network.

Find the computer's IP address on that network:

```sh
ipconfig getifaddr en0      # macOS, WiFi
```

### 2.2 WiFi settings

```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h   # once
```

In `config/wifi_secrets.h` set `WIFI_SSID`, `WIFI_PASSWORD` and
`MQTT_BROKER_IP` (the address from 2.1). `wifi_secrets.h` is in
`.gitignore`: never commit it. If the computer's address changes (new
hotspot, new network), change it here and rebuild.

### 2.3 Build and flash

```sh
cd build_make
make NET=lwip MQTT=1 -j8            # normal image
make NET=lwip MQTT=1 TEST=1 -j8     # self-test image (Part 3.1)
```

Images: `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` and
`..._wifi_lwip_mqtt_test.uf2`. Hold BOOTSEL, plug the Pico W's USB into the
computer, copy the `.uf2` onto the drive that appears.

Without `NET=lwip MQTT=1` the firmware still builds and runs; Buddy 1 then
prints `[wifi] built without WIFI=cyw43 NET=lwip MQTT=1` and does nothing.

### 2.4 Console

```sh
ls /dev/cu.usbmodem*                   # MTK3PICO1 is this firmware
screen -L /dev/cu.usbmodemMTK3PICO1 115200     # quit: Ctrl-A K Y
```

The port disappears on every reset, so start `screen` again after each
flash. Buddy 1 prints its state changes (`mqtt: Connecting`,
`mqtt: Operational`, `[wifi] ...`) and one line per command received.

## Part 3 — on the Pico

### 3.1 Self-tests (S1-U01 to S1-U04)

Flash the `_test` image. The checks run at start-up and print
`[TEST] S1-U01a PASS` … `[TEST] S1-U04b PASS`. They do not need the broker.
Save the console.

### 3.2 Connect (S1-U07)

1. Mosquitto running (2.1), normal image flashed, console open.
2. Console shows WiFi joining, then `mqtt: Connecting`, then
   `mqtt: Operational` and `mqtt: listening on is28/cmd`. Mosquitto's window
   shows a new client `mtk3smp-pico` (or your `MQTT_CLIENT_ID`).
3. Open the dashboard: `open tools/dashboard/index.html` (any browser).
   Broker URL `ws://localhost:9001` (or `ws://<computer IP>:9001` from
   another device). Press **Connect**.
4. Pass: the car pill turns to *car: online*, the heartbeat age stays under
   2 s, *Link* shows Operational, the operating state shows Ready.

If it stays in Connecting / Reconnect Wait: check `MQTT_BROKER_IP`, that
both are on the same network, and the macOS firewall prompt.

## Part 4 — commands (S1-U09 to S1-U12, S1-U16, S1-U17)

Watch three places: the ack line on the dashboard, the operating state on
the dashboard, and the console line `[wifi] command ...`.

| Test | Do | Pass |
|---|---|---|
| S1-U09 | **Start**, **Pause**, **Resume**, **Stop**, then **Reset** | each ack `accepted`; state Running → Paused → Running → Safe Stop → Ready |
| S1-U10 | **Send STEER**, **Send DRIVE**, **Bad length**, **Unknown code** | acks: steering not allowed (twice), bad format, unknown command; operating state unchanged; *rejected* count up by 4 |
| S1-U11 | **Start**, then **EMERGENCY STOP**, then **Reset** | Safety ESTOP and operating state Emergency Stop at once; Reset goes back to Ready |
| S1-U12 | **Ping** 10 times | every ack arrives; write down the latency shown (ms) |
| S1-U16 | **Telemetry off**, wait 5 s, **Telemetry on** | no `is28/tlm` lines in *Messages* while off, heartbeat keeps coming; frames return when on |
| S1-U17 | press GP20, then GP21 (Maker Pi Pico: the GP20 and GP21 buttons; Robo Pico: same pins) | ESTOP, then reset and start; the dashboard shows both state changes |

`accepted` means the command was checked and handed to the Navigation
Coordinator or the Safety Supervisor. They still apply their own rules: for
example START is ignored unless the car is Ready, so the state not changing
after START from Paused is correct.

## Part 5 — connection tests (S1-U08, S1-U13 to S1-U15)

### S1-U08 — 5-minute connectivity test (NFR-WIFI-01, NFR-WIFI-02)

1. Car Operational, dashboard connected. Press **Start** so that Nav and
   Safety publish too (optional; the heartbeat alone is enough for the
   test).
2. Press **Start 5-minute test**. Do not touch anything for 5 minutes.
3. Pass: complete the full 300 seconds, every observed telemetry source
   ≥ 95 % by sequence-gap estimate, largest heartbeat gap ≤ 2 s, result
   **PASS**. Early stop is **INCOMPLETE**. Silence before the first or after
   the last heartbeat counts towards the gap. Press **Save results** and
   put the file in `logs/`.

Delivery is counted from the 8-bit sequence number in each frame: a missing
number is a lost message. Do it once next to the router or phone and once
from the far end of the track.
This is an estimate over received sequence numbers, not an independent
sender/receiver audit. A source which never appears cannot be assessed;
record which sources were expected. Jumps of 128 or more are ambiguous
(restart versus loss); long outages require separate sender-side evidence.
Source 8 is on-demand configuration and is excluded from the delivery test.

### S1-U13 — broker lost and back

1. Car Operational. Press Ctrl-C in the Mosquitto window.
2. Console: `mqtt: Reconnect wait`, then attempts 1, 2, 4, 8, 8 … s apart.
3. After about 20 s start Mosquitto again (same command). The dashboard
   reconnects by itself (every 2 s).
4. Pass: Operational again within about 10 s; the operating state did not
   change because of the broker.
5. Confirm **WiFi/broker lost** remains latched after recovery. Send a
   **Ping**, then **Stop → Reset**. Expect **Ready** and a fresh Safety
   frame with the WiFi fault bit clear. With no other faults, the mask
   should show **none**. Save the export after the fresh Safety frame.
   A Ready Navigation frame alone does not prove the Safety mask refreshed.

### S1-U14 — WiFi lost and back

1. Car Operational, press **Start** (state Running).
2. Turn the hotspot off (or walk out of range) for about 20 s, then on.
3. Pass: the car stays Running (losing the link is a warning, not a fault);
   console shows the link going down and coming back; after reconnecting the
   dashboard's fault mask shows *WiFi/broker lost*. Press **Stop**, then
   **Reset**: the bit clears.

### S1-U15 — last will

1. Car Operational, dashboard connected.
2. Unplug the Pico W's USB.
3. Pass: within about 8 s (1.5 × the 5 s keepalive) the dashboard shows
   *car offline (broker last will)*.

## Part 6 — record results

- One row per test run in `TEST_LOG.md` → *Results*: date, test ID, commit
  (`git rev-parse --short HEAD`), tester, PASS/FAIL, log file.
- Logs go in `logs/` named `YYYY-MM-DD_S1-Uxx_short_name.txt`: the console
  (`screen -L` writes `screenlog.0`; rename it), the dashboard's saved
  message log and connectivity results, and photos or screen recordings for
  manual tests.
- A failed test is still recorded. Write what you saw in *Notes*.

## Part 7 — barcode mapping (Buddy 1 only)

The instructor chooses four A–Z letters for Forward, U-turn, Left and
Right before the demo. Use **Barcode letter mapping** on the dashboard.
No default letter assignments are installed at power-up. The mapping is
RAM-only, retained through the supervisory RESET, lost on power-off.

### S1-U18 — mapping logic and protocol, on the computer

`make -C testing/subsystem1/host` runs `test_mapping` as well as `test_b1`.
The mapping test checks all 358,800 distinct four-letter assignments,
wrong lengths, duplicate/invalid letters, revision wrap and readback.
This does not exercise the RTOS queues or real MQTT.

### S1-U19 — dashboard schema and browser UI, simulated MQTT

Generate the C vectors, then run `check_dashboard.mjs` as in Part 1.
Optional browser test, with Playwright and its Chromium installed:

```sh
node testing/subsystem1/host/check_dashboard_ui.mjs
```

To use an installed Chrome, set `IS28_BROWSER_CHANNEL=chrome`. A bundled
Playwright package can be selected with `IS28_PLAYWRIGHT_MODULE`; an
absolute output directory in `IS28_UI_SCREENSHOTS` saves desktop/mobile
screenshots. All MQTT in this browser test is simulated; it is not hardware
evidence. The test checks draft versus active mapping, lost readback,
invalid input, disconnects and incomplete connectivity-test verdicts.

### S1-U20 — real mapping round trip, Pico and broker

1. Configure the local 2.4 GHz WiFi and broker in `config/wifi_secrets.h`.
   Build with `WIFI=cyw43 NET=lwip MQTT=1`. Keep motors simulated.
   A build made with example credentials is compile-only, not a test image
   ready to connect to your actual network.
2. Fresh power-up, connect dashboard, **Read mapping from car**: expect
   **Car has no mapping**. Record the console and dashboard message log.
3. Select any four distinct demo letters. For an explicit test vector,
   use Forward=Z, U-turn=A, Left=M, Right=Q. These are test inputs only.
   **Apply mapping**: expect accepted ack AND matching source-8 readback.
   Verify the **Active on car** column, revision and request id.
4. Send the same map again: revision unchanged. Swap two assignments and
   apply: revision increases once; all four active assignments change as
   one configuration. Use **Read mapping from car** to verify it again.
5. Duplicate a selector: the dashboard must refuse to publish. Also send
   malformed binary commands using the wire format in
   `tools/dashboard/README.md` (duplicate fields, index 26, reserved bit,
   wrong length): each is rejected and GET returns the previous map.
6. **Telemetry off**, then read/apply a map: ack/readback still work.
   **Reset**: mapping remains. Power-cycle: mapping becomes unconfigured.
7. Disconnect broker/car while applying: the dashboard must not claim
   success without readback. Reconnect and explicitly GET before retrying.

Save evidence in `logs/YYYY-MM-DD_S1-U20_mapping_roundtrip.txt`, record
actual firmware commit plus any local modifications, board and broker.
Pass requires observations from the real Pico; host tests do not close it.

### S1-U21 — network bridge and reconnect checks

Repeat S1-U13/U14, then GET and apply a mapping after each reconnect.
Verify that commands arrive after a new subscription. Exercise oversized
and fragmented inbound payloads and queue saturation with a test MQTT
client; no valid prefix may be applied, no buffer overrun or task stall,
and drops must be counted. Verify startup with the broker already up and
with it starting later. Record this separately from simulated UI results.

Buddy 2 motion and Buddy 3 A–Z/bidirectional decoding, pending-letter
storage and execution at the next cross-junction are deferred. The
dashboard's mapped movement for the last letter is a configuration lookup,
not proof of a pending or completed navigation action.

## Part 8 — automated live edge checks and fault injection

These checks use a real Pico, not the simulated browser harness. Use the
standalone Buddy 1 image with no-output motor stubs. Keep the dashboard
controls untouched during a script batch. Commands below run from the repo
root; replace the example output prefix for a new run so old logs survive.

### S1-U20 — mapping edge batch

```sh
python3 testing/subsystem1/host/test_mqtt_live.py --run \
  --output testing/subsystem1/logs/YYYY-MM-DD_S1-U20_live_edges
```

The standard-library client connects to the local broker, subscribes,
sends Stop/Reset and installs the test map Z/A/Q/M. It checks duplicate
letters, index 26, reserved bits, short/long messages and 1024-byte messages
with valid mapping prefixes. Each malformed command must return FORMAT
(code 3), and GET must preserve the complete mapping and revision. It then
checks SET/GET and identical SET while ordinary telemetry forwarding is
off, restores Z/A/Q/M and telemetry on, and checks RESET retention. It
saves a raw transcript and JSON assertions. A script error is a harness
failure until reviewed; it is not automatically a firmware failure.

### S1-U21 — finite request burst

```sh
python3 testing/subsystem1/host/test_mqtt_live.py --run --stress-only \
  --output testing/subsystem1/logs/YYYY-MM-DD_S1-U21_live_burst
```

One burst sends 1024 GET requests without changing the map. Expect a rise
in the shared lost counter, continued heartbeats, subsequent responsive
Ping/GET and an unchanged map. Record acknowledgement count, drop-counter
delta and final queue occupancy. This shared counter combines RX/TX/network
drops; do not claim that it attributes or accounts for every request.

The oversized-message checks exercise inbound payloads larger than the
SDK parser's default 128-byte staging buffer. Multiple callback chunks are
inferred from that parser source, not counted by target instrumentation.
Publisher-side TCP splitting alone does not establish Pico callback splits.

### S1-U20 — real dashboard loses a SET readback

Start the temporary loopback proxy in a separate terminal:

```sh
python3 testing/subsystem1/host/mqtt_ws_fault_proxy.py \
  --arm /private/tmp/is28_drop_mapping_readback \
  --log testing/subsystem1/logs/YYYY-MM-DD_S1-U20_live_readback_proxy.jsonl
```

1. In the dashboard, Disconnect, change the broker URL to
   `ws://127.0.0.1:9002`, then Connect. Read/confirm Z/A/Q/M first.
2. Create the arm file with `touch /private/tmp/is28_drop_mapping_readback`.
   Change Forward to Y and Apply. The proxy drops mapping readbacks on the
   dashboard path while keeping ACKs and heartbeats. Its log must show an
   actual drop before this fault injection counts as executed.
3. Expect the accepted ACK, but no confirmation of Y/A/Q/M. After the
   five-second timeout the dashboard retries GET; while replies are
   withheld it should continue waiting rather than confirm the new map.
   An independent MQTT GET can establish that the Pico really applied it.
4. Remove only the arm file: `rm /private/tmp/is28_drop_mapping_readback`.
   Click Read mapping. Expect confirmed Y/A/Q/M. Restore Forward to Z,
   Apply and confirm Z/A/Q/M. Save screenshots, UI text and native evidence.
5. Disconnect, restore `ws://127.0.0.1:9001`, Connect and confirm the map.
   Stop the proxy with Ctrl-C. Normal broker/server processes stay running.

This isolates readback loss. It does not establish every possible timing
of a full broker or Wi-Fi disconnection during SET.

### S1-U21 — broker starts after the Pico

Stop the project broker, then reboot or flash the Pico. Save its USB log:
expect connection attempts and Reconnect wait without a task stall. Start
the same broker again. Save broker and MQTT logs showing connection and a
fresh command subscription, then verify Ping and mapping SET/GET. Before
any Start/Stop/Reset, inspect a fresh heartbeat: initial state must be Ready.
Use an independent timestamped subscriber to measure reconnect bounds.

Current deferred cases and evidence qualifications are in `STATUS.md`.
