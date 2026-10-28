# Buddy 1 µT-Kernel network proof

This snapshot is the **first phase of the professor micro T-Kernel migration**.
It proves the Pico W network path, Buddy 1 MQTT command bridge, barcode-map
storage/readback, and basic kernel state reporting. The motors are disabled at
boot. This is **not** the full line-following firmware: no physical motion,
line tracing, barcode sensing, IMU, obstacle avoidance, or safety telemetry is
integrated in this phase.

## Windows bench setup

1. Close the old dashboard/broker terminal with Ctrl+C before starting the
   new helper. Leave the robot stopped for this network-only test.
2. On the laptop, join the same 2.4 GHz Wi-Fi network or hotspot you want the
   Pico to use. Start the dashboard and broker by double-clicking
   **`Start dashboard.cmd`**. If Mosquitto is installed, the helper starts it
   and prints the laptop's LAN IPv4 address. The dashboard opens in your
   browser; its local broker URL is `ws://localhost:9001`. If the helper says
   Mosquitto is missing, install the Windows Mosquitto package (or add its
   executable to `PATH`) and start the dashboard helper again.
3. Flash **[`firmware/buddy123_utk_network_v1.uf2`](firmware/buddy123_utk_network_v1.uf2)**
   to the Pico W using BOOTSEL: hold BOOTSEL while connecting USB, release it
   when the `RPI-RP2` drive appears, then copy the UF2 to that drive. The Pico
   reboots after the copy.
4. The previous interface WiFi firmware's saved **WCF1** network settings are
   compatible with this image. If those settings already have the right SSID
   and laptop broker address, keep them; do not provision again. If the boot
   status says `cfg=0`, or the SSID/broker address has changed, run
   **`Configure WiFi.cmd`** and follow its prompts. That helper writes the
   settings page through the RP2040 `RPI-RP2` BOOTSEL drive and then reboots
   the Pico. It does not update the network firmware. Enter the laptop's LAN
   IPv4 from step 2 as the broker address; `localhost` is only for the laptop
   browser.
5. Wait for the dashboard's separate **broker connected** and **car: online**
   indicators. The page requests the active barcode map automatically. A map
   is confirmed only when the page shows a matching source-8 readback and its
   revision. If the car reports no configured map, choose four different
   A–Z letters in Forward, U-turn, Left, Right order and apply the map.

The firmware prints its connection status automatically every **2 seconds**
over USB. A passive serial monitor at 115200 baud is optional for diagnosis;
the setup does not need pyserial, typed COM-port input, or interactive
single-letter commands. In particular, `n`, `g`, `f`, `w`, `l`, `r`, and `k`
from the standalone firmware are not used here. Do not type commands into the
console. The dashboard is the command interface.

## What proves this phase is working

In an optional passive USB console, look for:

```text
motors=OFF cfg=1 ...
mqtt: Operational
mqtt: listening on is28/cmd
```

The dashboard should show a live Buddy 1 heartbeat and the map it reads back
from the Pico. `Operational` means the Wi-Fi/MQTT link is up; a matching map
readback confirms the stored map. The dashboard's broker connection alone
does not prove the Pico is connected.

The active barcode map is held in RAM and starts unconfigured after reboot;
apply it again from the dashboard. Wi-Fi settings persist in flash. The
BOOTSEL helper writes a complete aligned 4 KiB settings sector without
including the application or saved driving-log sectors. It creates the
private settings UF2 in memory and does not save it on the laptop. Credentials
remain unencrypted in the device's flash. The complete-sector layout follows
the [RP2040 UF2 bootloader requirements](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf#page=145).

Successful network, command-ACK, and map-readback checks are integration
evidence for this first phase only. Physical Wi-Fi range, motor hardware,
line-sensor behavior, closed-loop driving, barcode recognition, avoidance,
and full firmware requirements still need separate hardware validation after
the relevant subsystem links are integrated.

## Documents

- [`BUILD.md`](BUILD.md): exact Windows build helper, profile flags, and tools.
- [`DATA_REPRESENTATION.md`](DATA_REPRESENTATION.md): limits and scope of the
  binary data contracts in this proof image.
- [`COMPARISON_WIFI_V2_1.md`](COMPARISON_WIFI_V2_1.md): teammate's timeout-pool
  fix, why this image already covers it, and allocator regression evidence.
- [`dashboard/README.md`](dashboard/README.md): laptop broker/dashboard
  behavior and local browser checks.
