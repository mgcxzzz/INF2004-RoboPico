# Buddy 1/2/3 Interface WiFi V1.1

V1.1 fixes a blocking network readiness check: the radio-only SDK query returns JOIN (1), while IP-ready UP (3) comes from cyw43_tcpip_link_status. V1 compared the wrong query with UP, preventing MQTT attempts even on a joined network. Motor/steering parameters, routing, settings and stored-log schemas are unchanged. Flash V1.1; normal flashing preserves the settings sector, so check n before repeating provisioning.

The extra n line WIFI_LINK reports SDK status (0 down,1 joined,2 waiting for IP,3 IP ready,-1 failure,-2 SSID not found,-3 authentication failure), Pico IPv4, MQTT attempts and latest result. mqtt_result=-32768 means no attempt yet; result0 alone is not connection proof. mqtt=1 subscribed=1 remains the required broker confirmation. Hardware verification is still needed.

This independent Pico W firmware combines the smoother **Continuous V2** controller with Buddy 1's MQTT v2 protocol/mapping and an extended browser dashboard. It retains one navigation coordinator and motor writer. Buddy 5's default hooks are empty; Buddy 4 assistance is still unsupported.

Baselines: V2 `1263390b069ae9617c412e47212177e77807c505`; Buddy 1 `b09403074939f45cf4d7c11b7606148f6c2a4e73`. See [UPSTREAM.md](UPSTREAM.md), [BUILD.md](BUILD.md), [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md), [dashboard/README.md](dashboard/README.md).

## First setup

1. Flash [buddy123_interface_wifi_v1_1.uf2](firmware/buddy123_interface_wifi_v1_1.uf2) using BOOTSEL. PuTTY `h` must say **BUDDY123 INTERFACE WIFI v1.1**. Motors stay off on boot.
2. Install [Mosquitto for Windows](https://mosquitto.org/download/) on the laptop. The supplied config listens on MQTT TCP **1883** and browser WebSockets **9001**. Put laptop and Pico on the same 2.4 GHz Wi-Fi/hotspot. A shared hotspot is useful when a school network isolates clients.
3. In this folder, run `py run_dashboard.py`. It starts Mosquitto if installed, serves/opens the dashboard and prints the laptop's LAN IPv4. Leave that terminal running. For an unusual install path: `py run_dashboard.py --mosquitto "C:\path\mosquitto.exe"`.
4. Close PuTTY to release the COM port. Install the USB helper dependency once: `py -m pip install pyserial`.
5. Run `py provision_wifi.py --port COM7 --broker 192.168.1.12`, using the actual port and laptop IP. Enter the SSID and **hidden password locally**. It stops the robot, waits for saving, verifies settings in flash and reboots. No credentials belong in chat or Git. Optional `--settings wifi_settings.local.json` accepts a local ignored JSON with `ssid`, `password`, `broker`.
6. Reopen PuTTY and press `n`. Look for `configured=1 stack_ready=1 mqtt=1 subscribed=1`. Browser broker-connected and **car online** are separate. Allow the normal Windows private-network prompt if needed; the helper does not change firewall policy automatically.

The delivered network-enabled UF2 has **no embedded credentials**. Normal line following works before setup. CRC-checked device settings survive reboots; repeat USB setup while stopped to change networks. Optional ignored compile-time `wifi_config.local.h` is supported, with saved device settings taking priority. Credentials are not encrypted in flash, but are never printed or included in the published UF2.

If Mosquitto's Windows service is already using only port1883, the launcher reports a conflict. Stop that service in Windows Services before using the supplied broker config, or configure the existing broker for both1883 and9001. The helper verifies both listeners before reporting its own broker ready; it never changes services automatically. Python3 is needed for the helpers; the `.cmd` shortcuts run them from the correct folder.

Windows USB recovery: the Pico intentionally disconnects/reboots after saving settings. The helper reads the short confirmation one byte at a time. If USB disappears before confirmation, it reports an unconfirmed outcome rather than success. Wait for the COM port to return, reopen PuTTY and press `n`. `configured=1` confirms loaded settings; `stack_ready=1` confirms stack initialization, not Wi-Fi association. Only `mqtt=1 subscribed=1` confirms the broker link. An old helper may show `ClearCommError` during this reboot; it does not by itself mean settings failed or require reflashing.

## Calibration, mapping and start

The first flash imports valid front calibration/profile from records1-25. Keep the current mounting and valid calibration; use `w,l,r,k` only if calibration is needed. `1`, then `k` selects the usual profile. Existing USB `g`/`f` and bank buttons remain: wait5s, BOTH3s, release, wait1s, tap GP21, then2s countdown. Either button or USB `s` stops.

The dashboard adds Start (2s countdown), Pause, Resume, Stop, Reset and latched network ESTOP. Start retains calibration, released-button, rearm and settling checks. Manual `s` cancels a pending network start. Pause/Resume use Buddy1's coordinator lease; Resume requires measured stop/front line visibility. An interrupted junction may require repositioning/restarting rather than resuming stale geometry. Reset releases network ESTOP only while stopped/settled with both buttons released; it keeps the RAM barcode map.

**Apply the barcode mapping before a junction test.** Select four distinct A-Z letters for Straight/U-turn/Left/Right, then wait for confirmed readback. To reproduce the old mapping use **Straight=Q, U-turn=Z, Left=A, Right=F**. There are no boot defaults: unassigned letters are logged and do not queue movement. Complete Code39 frames remain required in either direction/print scale.

Live whole-map updates affect **subsequent decodes**. A decode captures its letter/action/revision, preserving an already queued or executing action. The next confirmed isolated junction consumes it once. Straight consumes it without turning. The map survives Stop/Reset but clears on power loss/reboot; the browser queries it again and does not send retained movement commands.

Profiles/tuning are inherited unchanged:1 normal45mm/s;2 legacy180mm/s ramp experiment;3 qualified up to135mm/s. Scan/approach remains45mm/s. Begin integration testing with profile1.

## Live and stored logs

Live copied binary motion/line snapshots are20Hz, heartbeat/coordinator status about1Hz, plus actual barcode/coordinator events. The dashboard plots measured/target wheel speed, PWM and levels, with raw ADC, distance/time, pipeline state, timeline and CSV/JSON export. During a run, calibrated levels are the filtered levels used by control; idle placement readings use current ADC/calibration. Request accepted, queued, dispatch and completed remain separate.

Wi-Fi/MQTT loss does not stop or restart the robot. Direct remote steering/drive/turn commands are rejected. Data is timestamped, queues are bounded and loss/stale state is visible. Phone access uses the laptop LAN URL printed by the helper; WebSocket host must be that laptop, not the phone's localhost.

Console: `p` motion/events, `d` drive diagnostics, `j` barcode, `a` raw ADC, `z` coordinator, `n` network. Stop/wait5s before dumping. Dashboard stored-log downloads also require the run ended/settled, fetching rows sequentially with IDs, fragment reassembly and bounded retries. Exports are complete only when all rows arrive. Motion is decoded CSV; all datasets also have binary/hex CSV/metadata exports. Diagnostic rows are format4/58bytes; nav rows31bytes; raw rows10bytes little-endian `us:u32,path100:i32,adc:u16`. Downloaded motion flag128 means **run-relative timestamp**; live timestamps are boot-relative.

To fit networking, onboard motion is **latest384 rows** (~7.7s at20ms, less with added state-change rows), raw ADC **first2048 samples** (~4.1s at2ms). Place a barcode near the start for a complete raw capture. Drive diagnostic256 rows, barcode192 events, coordinator120 entries and64 turn summaries remain. Whole-run time/distance counters are independent of overwritten rows. The laptop records longer received histories.

Record26 imports old calibration, not old history layouts. Older UF2s cannot read it, so downgrading may require recalibration. Raw version2 likewise rejects older raw layouts.

Core0 owns ADC/control/motor GPIO. Core1 alone owns Wi-Fi/lwIP/MQTT through copied bounded queues. Network callbacks never call motion. Stopped-only flash writes park core1 before masking IRQs; no flash writes occur during drive. Asynchronous Wi-Fi/broker retries are on core1. As in Buddy1’s successful bench configuration, power saving is disabled on the network core; `n` reports `pm_off_verified=1` only after successful set/readback.

Host tests/ARM builds are software evidence, not physical connection/course proof. First compare a profile1 straight with networking unconfigured/configured, then a bend, arbitrary letter/action combinations, live remapping and reconnect while running. Send `p`, `d`, `n` and dashboard exports. The V2 UF2 remains the physical comparison baseline.
