# Step4 v0.1.2 — relaxed approach gate

This remains **Step4 of the micro T-Kernel migration**, Buddy1–3.
IMU/ramp remains Step5; Buddy5 is not integrated. Main remains unchanged.

## Change

The prior test exports confirm that both failed runs recognised a cross:
18:01:02 decoded F/Left, confirmed12.90mm then recorded B_GATE_REJECT;
18:04:17 decoded Z/U-turn, confirmed12.42mm then recorded B_GATE_REJECT.
17:55:00 contains the successful F/Left and Z/Right completions. These event
contents differ from the user's initial Test1/Test3 filename labels.

Step4 now evaluates eligibility at cross confirmation. It no longer requires
the extra300ms/3degree heading qualification or both measured wheel speeds
above10mm/s, and no longer permanently vetoes a crossing because eligibility
was false at its leading edge. Ordinary steering corrections may continue.

Retained: verified mapped barcode, independent contrast junction sensing,
fresh4mm white re-arm,2ms debounce,11–30mm stripe,4mm trailing white,110mm
sensor offset, current front-line visibility, lost-line/search protection,
150mm post-turn clearance, nav coordinator/single motor writer, all existing
STOP/E-stop/USB/network supervision. Front calibration,45mm/s profile1,
Continuous V2 gains/PI/PWM, both Code39 decoders, MQTT and dashboard unchanged.
This does not change turn angle/acquisition logic or enable IMU/ramp.

Rejected crosses now report the precise reasons in event9 `flags`:

| Bit | Meaning |
|---|---|
|1|Legacy approach gate; not used in this Step4 profile|
|2|Inside150mm clearance after a completed turn|
|4|Line follower reports line lost|
|8|Neither front normalized level reaches400|
|16|Line search active|

Bits can combine, e.g.12 means line lost plus no visible front line. A reject
keeps the pending action for a later cross; no delayed turn is scheduled for
an already passed cross. SDK/other profiles retain their original gate by
default; the shared-source audit pins the exact reviewed opt-in edit.

## Setup and tests

1. Extract the v0.1.2 bundle into a new folder. Close previous dashboard and
   serial tools. BOOTSEL and flash
   `firmware/robopico_rtos_connected_barcode_v0_1_2.uf2`.
   Wi-Fi and saved front calibration remain. Do not recalibrate. If SSID,
   password and laptop broker IP are unchanged, no Setup Wi-Fi is needed.
2. Run **Start dashboard.cmd**, keep it open, hard-refresh
   http://localhost:8765/ and connect ws://localhost:9001.
3. Close PuTTY and run **Capture connected test.cmd**, choosing the actual
   COM port. Keep it actively reading throughout driving: USB DTR supervision
   remains mandatory. Confirm USB banner **CONNECTED BARCODE v0.1.2**,
   `JUNCTION_GATE policy=confirmation`, CLOCK PASS and network subscribed.
   Turn battery/motor power on for the run; keep cable loose.
4. Apply/read back **Forward=N, U-turn=Y, Left=F, Right=Z**. Repeat the
   successful F-left then Z-right route. Keep the sensor sweep supported.
5. Export as below before another Start. For a second run, repeat F-left
   after swapping the junction mats as in the failed test.
6. For the U-turn run apply/read back **Forward=N, U-turn=Z, Left=F,
   Right=Y**. All four letters must be different. Scan Z then the separate
   cross. Keep GP20/GP21 accessible. Video the full car and sensor path.
   The approach change does not yet prove physical U-turn alignment.

## Export each run before starting the next

Stop promptly after completion/failure and keep Pico powered. Let the capture
reach at least30s total, then Ctrl+C while stopped and wait for serial close.
Run **Export barcode log.cmd**, then **Export line log.cmd**, one at a time.
Do not reboot/reflash/Start between the run and exports. Send:

- `rtos_connected_session`: `_summary.json`, `_usb.txt`, `_mqtt.csv`.
- `rtos_barcode_session`: `_summary.json`, `_usb.txt`, `_events.csv`, `_raw.csv`.
- `rtos_line_session`: `_summary.json`, `_usb.txt`, `_line.csv`.
- Video of the physical junction. Add `dashboard_session/broker.log` if
  heartbeat/network issues occur.

The dashboard still does not download stored RTOS rows. Line history retains
the last6.4s; raw GP27 freezes at the first decode. Later junctions are covered
by barcode events, live USB capture and the stored line trace. Export time is
not run start time; label each set by intended letter/action/mat setup.

Mac equivalents: `python3 tools/dashboard/run_dashboard.py`,
`python3 tools/rtos_migration/capture_connected_line.py`,
`python3 tools/rtos_migration/export_barcode_log.py`, then
`python3 tools/rtos_migration/export_line_log.py`.
