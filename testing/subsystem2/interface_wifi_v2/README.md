# Buddy 1/2/3 interface Wi-Fi V2.2

This folder preserves the Continuous V2 driving, barcode and navigation implementation from `interface_wifi_v1` at commit `0285d74`. It improves MQTT recovery and diagnostics, and avoids guessing the laptop broker address from an unrelated network route. V1 is unchanged. This is an independent Pico SDK application: core 1 owns CYW43/lwIP/MQTT; core 0 communicates through bounded copied queues. It does not modify the RTOS MQTT implementation.

## V2.2 driving/network stack separation

A subsequent power-bank test accepted Start and emitted a Running heartbeat at 17:37:02, then broker LWT at 17:37:10. Disabling telemetry did not resolve it; the user reports driving continues and networking does not return after a physical stop. This is distinct from the earlier timer-pool panic, whose fix is retained.

The compiled driving call chain has **4328 bytes of local frames alone**. With the old default linker layout, that reaches into the other core's network stack. V2.2 reserves separate **8 KiB stacks** for both cores in the top 16 KiB of main SRAM, and lowers the heap limit below both stacks. The application uses the same driving and networking algorithms; no dual power supply or USB debugging is required for the retest. [STACK_LAYOUT.md](STACK_LAYOUT.md) contains the measured frames and real ELF checks. This proves a layout defect and removes that collision; it does not yet prove the physical dropout is resolved.

After pulling, flash `buddy123_interface_wifi_v2_2.uf2` for **V2.2**. Use **one power source at a time** for this setup. Start the laptop broker first, boot the car from the power bank, and wait for stable heartbeats. Keep telemetry disabled for the first driving test. Press Start and run on the normal test track for at least 30 seconds while watching heartbeats, then stop with the physical button and check that heartbeats persist. Save the message log. Repeat with telemetry enabled if that passes. If networking still drops, stop physically and save both the dashboard messages and laptop `dashboard_session/broker.log`; do not assume remote Stop is available after disconnection. Physical V2.2 testing remains pending.

## V2.1 timeout-pool fix

The V1.3 diagnostic hardware report identified `sys_timeout: timeout != NULL, pool MEMP_SYS_TIMEOUT is empty`. The network core panicked while polling; this explains why progress/connection timeouts stopped and later settings lockout failed. The diagnostic stack reported 1112/8192 bytes used and an intact bottom canary, so that report did not implicate stack exhaustion.

The copied `lwipopts.h` inherited lwIP's default **core-only** timeout budget. MQTT registers an application cyclic timer in its TCP-connect callback, before CONNACK. V2.1 explicitly reserves `LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4` slots (7 core + 4 spare with this build's options). The real lwIP allocator/list regression reproduces the exact panic with the old default and passes with the fix. These additional slots are bounded headroom for this single-client configuration, not a remedy for arbitrary timer leaks.

Historical V2.1 retest instructions follow; use V2.2 above for the current firmware. Start the broker first, boot the Pico stopped, capture `n` twice 20 seconds apart, then leave it online for two minutes. Poll counts and `last_ms` must advance, `mqtt=1 subscribed=1` and fresh heartbeats must persist, and no panic should appear. Physical retest of V2.1 is pending. Existing settings remain compatible; do not reprovision if the saved laptop IP is already correct. This fix does not by itself validate firewall/hotspot reachability, although the diagnostic panic occurs when the MQTT TCP-connect callback schedules its timer.

## What the initial reported failure means

The supplied PuTTY report has `sdk=3 ip=10.236.162.13`, so the Pico acquired an IP on the 2.4 GHz hotspot. `mqtt=0 subscribed=0 published=0` shows no established MQTT link. In V1, `mqtt_result=0` also represents the asynchronous connect API accepting a request; it does **not** prove that the broker accepted it. The screenshot alone cannot distinguish an incorrect broker address, unreachable TCP port, broker rejection or missing response. Growing dropped-TX counts while offline are expected bounded-queue drops, not successful sends.

The browser connects to WebSockets on port **9001**; the Pico connects to plain MQTT on TCP **1883**. Browser success does not prove Pico connectivity. The broker address saved on the Pico must be the **laptop's IPv4 on the hotspot**, not the phone gateway, Pico address, localhost or an address from another network. Both laptop and Pico must join that hotspot. Hotspot client isolation can still prevent them communicating.

## Teammate setup on Windows

1. Flash [buddy123_interface_wifi_v2_2.uf2](firmware/buddy123_interface_wifi_v2_2.uf2) through BOOTSEL. PuTTY `h` should show `BUDDY123 INTERFACE WIFI v2.2`. Normal flashing preserves existing device settings. Motors remain off at boot.
2. From this folder run `py run_dashboard.py --pico-ip 10.236.162.13` using the **current** Pico IP printed by `n`. The helper prints the laptop interface routed to that Pico. Check this against the Wi-Fi adapter IPv4 in `ipconfig`; a VPN can affect routing. Mosquitto must listen on both 1883 and 9001. The helper starts a broker when available; it does not change services or firewall rules.
3. Press `n` in PuTTY and compare `WIFI_PROGRESS broker=` with that laptop address. If different, close PuTTY, then run `py provision_wifi.py --port COM4 --broker LAPTOP_IP` with the real COM port/address. Enter hotspot credentials locally. The helper stops the car, saves settings and reboots. Open PuTTY again after USB returns. No password belongs in Git or chat.
4. Run `py check_mqtt.py --broker LAPTOP_IP --pico-ip PICO_IP`. It performs MQTT CONNECT/SUBSCRIBE and waits up to 20 seconds for a fresh binary heartbeat; it sends no car commands. A local success proves the laptop broker accepts MQTT, **not** that Windows permits incoming traffic from the Pico.
5. Press `n` again. Success requires `mqtt=1 subscribed=1`, `WIFI_MQTT stage=online`, and a fresh car heartbeat in the dashboard. If it fails, share only `WIFI_LINK`, `WIFI_PROGRESS` and `WIFI_MQTT` lines; no credentials.

If the helper is missing its USB dependency: `py -m pip install pyserial`. A complete source/dashboard/firmware package is [buddy123_interface_wifi_v2_2_bundle.zip](firmware/buddy123_interface_wifi_v2_2_bundle.zip). Existing double-click shortcuts remain available; the explicit route-aware command above is preferable for diagnosis. Ctrl+C stops the dashboard server and any broker started by its helper.

## Reading V2 diagnostics

`WIFI_MQTT` reports stage, SDK internal state, connect API result, callback result, subscription result, last error, initialization result, port and age of the stage. `-32768` means no corresponding result received yet. API result `0` means request started; callback `0` means accepted CONNACK; subscribe `0` means successful SUBACK. Only the combined online state proves the firmware finished its handshake. Result fields reset per attempt; the last error remains visible until a successful subscription.

| Stage / result | Meaning and next check |
| --- | --- |
| `unconfigured` | Provision hotspot and laptop broker settings. |
| `init_failed` / error -1001 | SDK initialization failed; `init` contains its return code. |
| `wifi_or_dhcp` | Check hotspot availability, credentials and DHCP; `WIFI_LINK sdk` has link status. |
| `tcp_connect` / error -1004 | No TCP connection completed within 10 s. Verify saved broker IP, TCP 1883 listener, Windows firewall and hotspot client isolation. |
| `await_connack` / error -1005 | TCP connected but MQTT handshake did not finish within 10 s from attempt start. Check the broker log and service/protocol on 1883. |
| callback 1..5 | Broker refused connection: protocol, client ID, availability, authentication or authorization respectively. |
| `await_suback` / error -1006 | Broker accepted CONNECT but did not acknowledge the command subscription within 5 s. |
| error -1007 | Subscription failed; inspect `subscribe` and broker permissions. |
| error -1002 / -1003 | Broker address parse / MQTT-client allocation failure. |
| `retry_wait` | Bounded exponential retry; inspect preceding error/callback. |
| `online` | CONNECT and command SUBSCRIBE succeeded; confirm fresh heartbeat as well. |

If TCP stalls despite the correct address, inspect the **existing** Windows firewall policy for inbound Mosquitto TCP 1883 on the hotspot's network profile and whether the phone allows communication between connected clients. Do not disable the firewall globally. A second client on the hotspot can test the laptop's inbound path more directly than a laptop connecting to itself.

## Changes and validation boundary

V2 separates API/callback/subscription results, exposes TCP versus CONNACK progress, records initialization failures, retries missing/failed subscriptions, and resets backoff only after a successful subscription. SDK private MQTT state is read on the network-owner core for diagnostics only; public callbacks still control connection state. Pin mapping, motor tuning, steering, optical logic, stored settings and binary wire formats are unchanged. Buddy 4 assistance remains unsupported; default Buddy 5 hooks remain empty. Wi-Fi loss retains the inherited behavior and does not itself stop/restart driving.

[TEST_LOG.md](TEST_LOG.md) records host/build evidence and what remains untested. Hardware connectivity on the teammate's hotspot is **not yet verified**. [BUILD.md](BUILD.md) gives build and artifact checks; [GUIDE_V1_2.md](GUIDE_V1_2.md) retains existing calibration, operation and saved-log details. Original provenance ledgers are historical; the current release ledger is `SOURCE_SHA256.json` and V1 comparison is `WIFI_V2_BASELINE.json`.

Firmware release names must include the complete version, for example `v2_2`. Future versions receive new filenames; do not replace an older version with different firmware under the same name.
