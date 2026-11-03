# Step 3A — RTOS network bench v0.1

**User test update:** [Steady baseline and functional broker reconnect passed](HARDWARE_ACCEPTANCE.md).
Moving integration remains Step3B; status encoding and recovery timing observations
are recorded with the hardware results. The existing UF2/bundle is unchanged.

**Experimental, motors disabled. No mat is needed.** This is the first half of
Step3, testing the kernel network service alongside the verified100Hz timing
foundation. Step3B will connect it to Step2 line motion after this bench passes.
This is not the finished Buddy1–4 RTOS image.

Step2 status: straight following and one U-turn trial worked on v0.1.2. Left/right
testing was inconclusive because the front sensors left the available mat.
Retest those and sharp bends on extended mat before calling Step2 accepted.
No further steering/calibration/junction changes are made for this checkpoint.

## What to download

Extract `robopico_rtos_network_bench_v0_1_bundle.zip` into a **separate folder**.
Keep your v0.1.2 bundle for tomorrow's mat tests. The new bundle contains the
UF2, source, dashboard, Wi-Fi setup and read-only capture helpers.

## 1. Flash and start the laptop broker

1. Stop the car, disconnect motor/battery power and close the serial console.
   Use **USB power only** for this entire stopped test.
2. Enter BOOTSEL and copy `firmware/robopico_rtos_network_bench_v0_1.uf2` onto
   RPI-RP2. Let it reboot normally.
3. Double-click **Start dashboard.cmd** in the extracted bundle. Keep this
   broker/server terminal open. The dashboard opens at http://localhost:8765/.
   Mosquitto needs TCP1883 for the Pico and WebSockets9001 for the browser.
   If a conflicting Windows service occupies one port, use the same documented
   broker configuration as your previous successful dashboard setup.
4. Identify the laptop's **current Wi-Fi LAN IPv4** using ipconfig(Windows) or
   the Mac network settings. Do not reuse a previous session's address, the
   gateway, a virtual Ethernet adapter address or127.0.0.1 for Pico settings.
   Laptop and Pico must use the same2.4GHz Wi-Fi/hotspot with client access.

Mac dashboard equivalent, from the extracted bundle directory:

```bash
python3 tools/dashboard/run_dashboard.py
```

## 2. Configure Wi-Fi if needed

If the existing saved WCF1 settings still match today's Wi-Fi and laptop IPv4,
they can be reused. Otherwise:

1. Close PuTTY/any serial console.
2. Double-click **Setup Wi-Fi.cmd**. Enter the SSID, hidden password and laptop
   broker LAN IPv4 when prompted. Do not send the password in chat.
3. When instructed, disconnect Pico power, hold BOOTSEL while reconnecting USB,
   release it and wait for RPI-RP2. Press Enter in the helper.
4. The helper sends a **settings-only UF2** and the Pico reboots. **Do not flash
   the firmware again afterward.** BOOTSEL does not erase the application.
5. The helper may close after completion. Only the dashboard/broker terminal
   must stay open. Sending the settings file is not device readback proof;
   verify the following status after reboot.

Mac equivalent:

```bash
python3 tools/rtos_migration/provision_network.py
```

No live RTOS flash write or pyserial is used for provisioning. Only the
WCF1 settings sector is replaced; calibration/history sectors remain untouched.

## 3. Confirm the car really connects

Open the USB serial console at115200, flow controlNone. Check:

```text
ROBOPICO RTOS NETWORK BENCH v0.1
MOTORS DISABLED
CLOCK ... status=PASS
NET BENCH motors=DISABLED configured=1 connected=1 subscribed=1 ...
NET_ADDRESS ip=<Pico address> broker=<current laptop LAN address>
```

Press **n** if necessary. Allow the Wi-Fi association/DHCP/MQTT handshake to
finish. `mqtt: connection requested ... result=0` alone does **not** prove
connection: look for subscribed=1 and advancing polls/last_ms/published counters.
Release both buttons. If an old stop latch is shown, use local **x**, then wait
for CLOCK PASS again. Motors remain inhibited even if CONTROL says READY.

On the dashboard, use **ws://localhost:9001** and click **Connect**. Confirm
Car online and heartbeat updates. Choose four distinct letters for
Straight/U-turn/Left/Right and apply. Check the car's mapping confirmation,
then click **Ping** and check its ACK. These are RAM mapping/protocol tests;
there is no barcode decoder or motion in this image.

Start/Resume are unsupported and manual Drive/Turn/Steer are prohibited.
Serial1/2/3 starts are rejected. Blank line/motion/IMU panels are expected;
the bench does not publish fake sensor telemetry. Do not use it on the circuit.

## 4. Capture the evidence (about77 seconds)

1. Leave the dashboard/broker running. Close PuTTY so the capture helper can
   open the same USB port. Keep the Pico powered and stopped.
2. Double-click **Capture network test.cmd** and enter COM4(or the current port).
   Install pyserial once if prompted: `py -3 -m pip install pyserial`.
3. After about10seconds, click dashboard **Ping**, then apply/change the four
   letter mapping. The helper sends no robot commands; it passively subscribes
   to heartbeats and reads USB timing/network status.
4. Keep the broker/hotspot up for the full capture. Do not test E-stop or reset
   during this baseline clock capture. The helper checks both clocks against
   laptop wall time,100Hz control cadence, advancing network polls, received
   dashboard requests, no new drops and actual heartbeat gaps no greater than2s.
5. Send all three files generated in **rtos_network_session**:
   `_usb.txt`, `_summary.json`, `_heartbeats.csv`.
6. In the dashboard's MQTT message log, click **Save log** and send the resulting
   `is28_messages_*.txt` too. **Video is not needed for this stopped network test.**
   A screenshot of Car online plus the applied mapping is useful if available.

Mac equivalent (pyserial is needed only for capture):

```bash
python3 -m pip install pyserial
python3 tools/rtos_migration/capture_network_bench.py
```

The monitor uses127.0.0.1:1883 because it runs on the broker laptop. This is
different from the Pico's broker setting, which must use the laptop LAN IPv4.
If the broker is on another computer, add `--broker <that computer's IPv4>`.
Send REVIEW/error output too; do not reflash or recalibrate simply to clear it.

## 5. Reconnect test, after the baseline passes

Keep the Pico powered. Close the dashboard/broker terminal with Ctrl+C, wait
about10seconds, then restart **Start dashboard.cmd**. Reconnect the browser.
The Pico should reconnect/subcribe again, resume heartbeat and report the same
RAM mapping without reboot. Send the dashboard Save log and copied n status if
it does not recover. A heartbeat gap while the broker is deliberately stopped
is expected and is separate from the uninterrupted baseline capture.

Next, we will attach the proven Step2 controller and test heartbeat/Stop/Pause
while moving, then wireless supervision. Dashboard Start/Resume and driving-time
failure handling are not accepted by this bench. Tomorrow, reflash your existing
v0.1.2 UF2 for the extended-mat junction/bend tests; do not recalibrate for this
network checkpoint. Neither public image overwrites calibration.

## Build and architecture

```bash
bash tools/rtos_migration/build_network_bench.sh
```

Profile: APP=rtos_network SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1
CRASH_REPORT=0 TEST=0. Kernel-owned mqtt_utk.c and bounded TX/RX message buffers
follow the professor's execution model. The existing extra4lwIP timeout slots
are retained and checked. No alternate SDK worker/core is launched.
See app_program/network_bridge/README.md for task ownership, command policy,
representation and data flow. Hardware acceptance is pending.
