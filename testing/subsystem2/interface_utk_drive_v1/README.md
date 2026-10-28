# Buddy 1/2/3 UTK Drive V1.1

This is the full V2 steering/controller and kernel-network integration for
the micro T-Kernel migration. It combines the validated Continuous V2
controller and navigation coordinator in one core-1 control task with Buddy
1's Wi-Fi/MQTT service and bounded message-buffer bridge on core 0. It does not
start the template kernel navigation or motion tasks. Buddy 5 remains on its
empty hooks; no IMU or obstacle telemetry is claimed.

This image is drive-capable, not the earlier motors-off network proof. Motor
outputs are low on boot and require an explicit start, but **physical driving
on the robot has not yet been validated**. Keep the robot secured, wheels
clear for initial tests, and emergency stop accessible. Do not treat dashboard
telemetry or a successful network check as physical validation.

## V1.1 timing correction and upgrading V1

V1.1 fixes the hardware TIMER clock initialization. The old V1 application
accepted an already-running watchdog tick without checking its divider;
passive hardware telemetry showed application time about 42.7 times too slow.
The V1 UF2 is retained as historical evidence; use the explicit V1.1 UF2.

**Existing V1 dashboard and Python helpers remain compatible. Reflashing only
`buddy123_utk_drive_v1_1.uf2` is enough.** No protocol, dashboard, saved Wi-Fi,
calibration, profile, or log layout changes are required. The V1.1 ZIP is an
optional complete source/helper bundle. Reapply the RAM barcode map after reboot.

Before Start, open PuTTY at 115200 and press `n`. V1.1 reports:

```text
TIMEBASE: PASS cycles=12 enable=1 running=1 ref_selected=0x4 ref_div=0x100 kernel_ms=... timer_us=...
```

The stored boot measurement should show roughly 250 kernel milliseconds and
250000 hardware microseconds (values may differ slightly with scheduling).
If the check fails, no drive task is started and failure diagnostics repeat
every two kernel seconds. Do not issue Start while checking idle timing.
Verify roughly one heartbeat per second, motion updates about every 50 ms,
and uptime increasing by about one second per wall second. Forced heartbeats
from other connected clients can produce additional messages. Then test the
normal two-second Start countdown. Keep steering/calibration settings as before.

See [TIMEBASE_FIX_V1_1.md](TIMEBASE_FIX_V1_1.md) for evidence and regressions.

## First Windows setup

1. Put the robot on a stable surface with the wheels clear. Start the laptop
   broker and dashboard by double-clicking **`Start dashboard.cmd`**. It
   starts Mosquitto when available, serves the dashboard, and prints the
   laptop's LAN address. The browser uses `ws://localhost:9001`; the Pico must
   use the laptop's LAN IPv4 as its broker address.
2. Flash **[`firmware/buddy123_utk_drive_v1_1.uf2`](firmware/buddy123_utk_drive_v1_1.uf2)**
   with BOOTSEL: hold BOOTSEL while connecting USB, release when the
   `RPI-RP2` drive appears, then copy the UF2 to that drive. The Pico reboots.
   The accompanying release bundle is
   **[`firmware/buddy123_utk_drive_v1_1_bundle.zip`](firmware/buddy123_utk_drive_v1_1_bundle.zip)**.
3. The Wi-Fi settings use the existing WCF1 sector at `0x101AA000`. A valid
   saved WCF1 page remains usable across this firmware update. Run
   **`Configure WiFi.cmd`** only if settings are missing/invalid or the SSID
   or laptop broker IPv4 has changed. It writes the separate WCF1 settings
   sector using the `RPI-RP2` BOOTSEL drive; it does not replace the drive
   firmware or calibration. Do not enter `localhost` for the Pico's broker.
4. Wait for the dashboard to report **broker connected** and **car: online**.
   Check the live Buddy 1 state, motion telemetry and active barcode map. A
   map change is complete only after the dashboard confirms the source-8
   readback and revision. Choose four different AÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“Z letters in Forward,
   U-turn, Left, Right order before testing junctions.

On first boot, the front-sensor calibration preset is **white 163/160, black
3457/3700**. A valid calibration record 27 saved by this firmware takes
precedence on later boots. An older record 26 must not replace the preset;
older V2 run logs may still be imported. The existing USB-console `w`, `l`,
and `r` calibration controls remain available. While stopped, with both
buttons released and the controller not settling, `c` restores the known V2
calibration in RAM and cancels any USB or Wi-Fi start countdown. Use `k` to
persist the current calibration/profile and logs so a valid record 27 is used
on the next boot. BOOTSEL `Configure WiFi.cmd` is a separate network-settings
operation.

The starting preset selects profile **1**. A later saved profile is restored
with record 27. Use profile **1** for the first drive check. Start using either the dashboard's **Start** button (two-second
countdown) or USB-console `g` (two-second countdown). Both retain the original
calibration-valid, stopped/settled, rearm, and buttons-released guards. The
existing offline BANK-button start path remains available: wait 5 seconds,
hold both bank buttons for 3 seconds, release them, wait 1 second, tap GP21,
then wait through the two-second countdown. Dashboard **Stop**, USB-console
`s`, or either bank button stops the run. USB connection changes stop an active run. Calibration/profile changes
are rejected while running, matching the original controller guards.

Use a serial terminal at 115200 baud only when you need to observe the USB
console or send the existing profile/start keys. Python `pyserial` is not
required. Stop with the dashboard or controller's normal stop path; do not
test close-range driving until the hardware has been checked.

After a run, stop the robot and wait until the dashboard reports **Off** for
at least 5 seconds. In **Stored data**, request each available dataset in
turn: Motion rows, Diagnostics, Raw sensor rows, and Navigation rows. Each
completed request downloads `.bin`, `.csv`, and `.json` files; an interrupted
or incomplete request is reported and does not produce a completed export.
Diagnostics contains barcode counters/status, while raw sensor rows include
the barcode ADC sample. There is no separate stored barcode dataset: use
Diagnostics and Raw sensor rows for stored barcode measurements, and the live
event CSV for decoded-letter, queue, dispatch, and completion events.
Navigation rows are the stored navigation-event log. The coordinator event
timeline is also captured live during the page session; use **Export event
CSV** for that live event history. **Export telemetry CSV**
and **Export telemetry JSON** save the page-session motion/events capture.
These live exports are separate from the stopped-only stored-data downloads;
the chart's rolling window is not a whole-run archive.

## What this dashboard reports

The dashboard uses the unchanged WiFi V2 MQTT topics and protocol. It shows
received motion/encoder values, line/barcode pipeline, coordinator events,
map readback, Wi-Fi/MQTT health, diagnostics, and actual run summaries. It
provides supervisory Start/Pause/Resume/Stop/Reset/E-stop/Ping and map controls;
it does not expose raw remote wheel steering. Empty panels remain empty until
real packets arrive. IMU and obstacle data are not available in this build.

The firmware build and generated UF2 have passed the software build step.
The image has not yet been flashed or physically validated. Hardware
motor/encoder direction, stopping distance, line-following and barcode turns,
calibration, and full obstacle/safety behavior need independent physical
validation. Do not change speed or gain parameters as part of this first test;
profile 1 is the recommended baseline.

Matching the saved calibration and byte-identical V2 steering/motor code can
help reproduce earlier behavior, but cannot guarantee the same result at a
changed mat height. Verify the actual setup on the robot.

## Build and dashboard references

- [`BUILD.md`](BUILD.md): Windows build helper, exact compile profile, and
  artifact paths.
- [`dashboard/README.md`](dashboard/README.md): local broker and browser
  dashboard operation, telemetry, stored logs, and smoke test.
- [`source/app_program/subsystem2/controller/GUIDE_V1_2.md`](source/app_program/subsystem2/controller/GUIDE_V1_2.md):
  controller calibration/profile and inherited log behavior.
