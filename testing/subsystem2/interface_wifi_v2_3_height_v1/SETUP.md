# Test setup - V2.3_height_v1

## Flash and connect

1. Save any previous run first. Download `firmware/buddy123_interface_wifi_v2_3_height_v1.uf2`. With the car stopped, enter BOOTSEL and copy this UF2 to RPI-RP2. Let it reboot normally. Normal flashing preserves the separate Wi-Fi/broker settings sector.
2. In PuTTY at 115200, press `h`. Confirm `BUDDY123 INTERFACE WIFI v2.3_height_v1` and `CAL FRONT white=191,154 black=1940,2306 valid=1`, profile 1. No recalibration is needed before this first test. Press `1` if needed to select profile 1.
3. Keep your existing Mosquitto/dashboard package, or use this package's `Start dashboard.cmd` / `py run_dashboard.py` (macOS: `python3 run_dashboard.py`). Keep the broker/dashboard terminal running. Existing wire formats are unchanged.
4. Press `n`: expect `mqtt=1 subscribed=1`, stage online, advancing counters and a fresh dashboard heartbeat. Wi-Fi works without USB once configured. Stop and wait five seconds before changing power; USB connectivity changes during a run invoke the inherited stop behavior.
5. Reapply the desired four-letter mapping after boot. There are no boot A/F/Z assignments. Confirm the mapping acknowledgement/readback before a barcode/junction test.

If SSID/password or the laptop broker IPv4 changed, use this folder's `Configure WiFi.cmd` or `py provision_wifi.py --port COM4 --broker YOUR_LAPTOP_IPV4` with V2.3_height_v1 running normally over USB. Close PuTTY first. This serial helper does not use BOOTSEL. `py -m pip install pyserial` installs its dependency. On macOS use python3 and the actual `/dev/cu.usbmodem...` port. Existing WCF1 settings from previous packages remain compatible; do not reprovision unchanged settings.

## First physical comparison

Use the reported fixed raised mounting. Keep the preloaded calibration and profile 1 initially. Test one straight without a barcode, then a bend, then a smooth barcode pass and its next mapped junction. Do not increase speed on the first run. Dashboard Start retains the two-second countdown; `g` is the USB hands-free equivalent. Either physical button stops the run.

The intended result is gentler ordinary corrections with adequate bend response. The software tests cannot confirm this on the actual robot. If the car drifts off, misses a bend or enters repeated recovery, stop physically and keep that run's logs before another run. Do not compensate by weakening line-loss protection or changing the barcode decoder during this comparison.

## Download logs

After stopping, wait five seconds for coordinator Off. Download **Stored data -> Motion rows, Diagnostics, Raw sensor rows and Navigation rows**, each using **Download stopped-only log**. Also click **Export event CSV** and **Save log**. Record a video and state that the firmware is height_v1. Stored buffers have different windows; Diagnostics is typically a bounded capture around 3 s and raw barcode holds the first 2048 samples. These are not all full-run recordings.

For a USB backup: save PuTTY All session output, then press `h`, `n`, `p`, `d`, `j`, `a`, `z` one at a time while stopped. The `d` header prints capture=5 and its units. For this variant, diagnostic GAIN is 0..100% and DAMP is 0..8 mm/s half-speed, not the old format4 interpretation.

## Calibration and reflashing

On first switching to this variant, the measured preset replaces older saved calibration versions. If you later use w/l/r/k, your own valid record29 calibration survives restarts and normal reflashing of the same variant. `c` restores white191/154 black1940/2306 while stopped/settled/buttons released; `k` saves it. Restoring clears previous motion rows in RAM. No captures are locked out.

Original V2.3 and its low-height preset remain in `../interface_wifi_v2_3`. Returning to that firmware restores its original preset because it does not accept height record29. Use a calibration matching the physical mounting.
