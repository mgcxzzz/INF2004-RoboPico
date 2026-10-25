# How to test Buddy 1 (WiFi and MQTT), step by step

This guide tells you what to do for every Buddy 1 test in `TEST_LOG.md`, in
order. Part 1 needs only a computer. Parts 2–5 need a Pico W (on the Maker Pi
Pico or the Robo Pico) and a computer running the MQTT broker. The wheels
never turn in this guide (`SYS_SIM_CAR` is 1). How to record a result is in
Part 6 and in `../README.md`.

---

## Safety first

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
| `is28/tlm` | car → dashboard | every other buddy's telemetry frame, unchanged |
| `is28/cmd` | dashboard → car | 4-byte command (START, PAUSE, RESUME, STOP, RESET, ESTOP, CONFIG, PING) |
| `is28/ack` | car → dashboard | answer to every command: accepted or why it was rejected |
| `is28/lwt` | broker → dashboard | the broker's last will: the car vanished |

## What you need

| Item | For |
|---|---|
| Pico W on the Maker Pi Pico or the Robo Pico, USB from the **Pico W's** socket to the computer | Parts 2–5 |
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

Expected: `21 passed, 0 failed`, then `D01` … `D06` PASS. Save the output
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
3. Pass: every source ≥ 95 % delivered, largest heartbeat gap ≤ 2 s, result
   **PASS**. Press **Save results** and put the file in `logs/`.

Delivery is counted from the 8-bit sequence number in each frame: a missing
number is a lost message. Do it once next to the router or phone and once
from the far end of the track.

### S1-U13 — broker lost and back

1. Car Operational. Press Ctrl-C in the Mosquitto window.
2. Console: `mqtt: Reconnect wait`, then attempts 1, 2, 4, 8, 8 … s apart.
3. After about 20 s start Mosquitto again (same command). The dashboard
   reconnects by itself (every 2 s).
4. Pass: Operational again within about 10 s; the operating state did not
   change because of the broker.

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
