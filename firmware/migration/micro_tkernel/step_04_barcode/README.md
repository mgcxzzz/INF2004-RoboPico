# Step4 v0.1.2 â€” connected barcode and junction sensing

**Latest test image:** `firmware/v0_1_2/robopico_rtos_connected_barcode_v0_1_2.uf2`.
Read [the exact junction retest and log instructions](UPDATE_V0_1_2.md) first.
The original v0.1 UF2/bundle remain in `firmware/` for rollback.

**EXPERIMENTAL. Hardware acceptance pending.** This is Buddy1â€“3 on micro
T-Kernel with the tested Step3B motion baseline, physical barcode decoding and
the network allocation correction. **Buddy4 IMU/ramp is Step5; Buddy5 is not
integrated.** Main and its SDK reference remain unchanged.

The latest bundle contains ONE application image:

| Image in `firmware/` | Purpose |
|---|---|
| `robopico_rtos_connected_barcode_v0_1_2.uf2` | Step4 junction fix: same decoders/network/motion, independent contrast junction sensor, confirmation-time gate,110mm sensor offset;640 line rows plus raw barcode history. |

Keep the previous v0.1.1 Step4 bundle for rollback. Read
[the network investigation](NETWORK_DELIVERY.md) and
[barcode logic](../../../../app_program/barcode_navigation/README.md).
Steering/PI gains, speed45mm/s, calibration priority, PWM limits and Step2's
angle-only junction experiment are retained. Extended-mat L/R and sharp bends
remain outstanding physical checks; do not treat a barcode decode as turn
alignment acceptance.

## Setup once

1. Extract the complete bundle into a NEW folder. Close the previous
   dashboard/server terminal and all USB serial programs.
2. Stop the robot/remove motor power. Enter BOOTSEL and copy the desired
   **connected_barcode_v0_1_2** application UF2 to RPI-RP2.
3. Run **Start dashboard.cmd** from this new folder; keep its terminal open.
   Browser http://localhost:8765/; connect ws://localhost:9001; hard-refresh.
4. Saved Wi-Fi is retained if SSID/password/broker IPv4 are unchanged.
   Otherwise run **Setup Wi-Fi.cmd**, entering BOOTSEL only when prompted.
   It sends settings only; do not reflash the application afterward.
5. Open PuTTY at115200 briefly to check the correct banner, CLOCK PASS,
   LINE flags0x3, NET connected1/subscribed1 and calibration. Expected for
   the currently tested car: **white122,138 / black246,267, origin1**.
   This is imported, not forced. Do not press w/l/r/k/c for this comparison.
6. Close PuTTY and run **Capture connected test.cmd**, select the actual port.
   It holds USB serial supervision open for300seconds and sends no commands.
   Keep the cable loose and the capture actively reading while driving.
   A finished command window does not hold supervision open.

Mac commands: `python3 tools/dashboard/run_dashboard.py`,
`python3 tools/rtos_migration/capture_connected_line.py`,
`python3 tools/rtos_migration/export_line_log.py`,
`python3 tools/rtos_migration/export_barcode_log.py`.
Install pyserial once in your existing Python environment if needed.

## A â€” network check, no motion

With motors off, capture60seconds Telemetry off, then60seconds on. Avoid
repeated Ping/map commands during those intervals. No need to repeat already
passed physical safety tests just to collect this baseline. Send the three
capture files AND `dashboard_session/broker.log`. Check CLOCK remains near
100Hz, NET_MEMORY heap_errors does not increase during the steady phases,
and actual received heartbeat gaps are normally<=2seconds. Do not discard a
failed capture or assume a connected MQTT flag guarantees timely delivery.

Wi-Fi/calibration sectors remain intact. The dashboard firmware badge should
say **Step4**; the USB banner identifies **v0.1.2** specifically. The dashboard
itself is unchanged. A new stationary network comparison is only necessary if
delivery problems recur; the priority for this update is the junction retest.

## B â€” decode first, with no junction in the scan path

1. Apply/read back the dashboard mapping. Suggested first test:
   **Straight=Y, U-turn=Z, Left=A, Right=F**. All four must be distinct Aâ€“Z.
   Mapping is RAM state: reapply after a reboot/reflash. No default letters.
2. Use one known good label, preferably A, on a straight with ample supported
   mat. Use the original low sensor mounting and unchanged front calibration.
   Keep the entire barcode sheet still and flat; keep the USB cable free.
3. Start (2s countdown), follow across the label, then Stop after a decode.
   Expect DECODED then QUEUED A/Left, but no turn until a separate junction.
   An unassigned letter must show UNASSIGNED and continue straight.
4. Keep capture open until at least30seconds total, then Ctrl+C while stopped.
   Keep Pico power connected. Run **Export barcode log.cmd**, then
   **Export line log.cmd**, one at a time. Neither resets the run.
   Do not new Start/reboot/reflash before both exports.
5. Repeat with the label rotated180degrees (reverse print direction while
   driving forward), then F, Z and Y. One label/run gives the clearest
   first-decode raw capture. Do not tune thresholds merely because one
   hardware scan fails: retain its raw capture for replay.

## C â€” mapped junction action and live mapping

Only after B works: label then a physically SEPARATE supported cross, leaving
adequate white gap as in the SDK arrangement. Expect decoded/queued, junction
confirmed, dispatched, then completed with continued line following. Test
Straight first, then left/right/U-turn with the entire sensor sweep on mat.
Have GP20/GP21 accessible. Export/video each attempt separately.

Then change the mapping while running BEFORE scanning the next label and
check its new action. A label already decoded retains its old action/revision
until consumed; a later map change must not silently change that pending turn.
The barcode stop bar must never trigger a turn. Pause midway through a label
must not allow its remaining half to finish after Resume. A fresh complete
label should still work. Pause during a junction requires Stop/reposition/new
Start, as in Step3B. Manual USB L/R/U injection is disabled in Step4; use the
actual mapped barcode for this acceptance.

## Files to send

- Each run: all3 **rtos_connected_session** files (`_usb.txt`, `_mqtt.csv`,
  `_summary.json`), plus broker.log for the network check.
- **rtos_barcode_session**: all4 barcode export files (`_usb.txt`, `_events.csv`,
  `_raw.csv`, `_summary.json`). This uses USB b only, while stopped.
- **rtos_line_session**: all3 line export files. Step4 retains only the last
  6.4seconds at100Hz; USB/MQTT capture covers the whole run. Raw GP27 retains
  ~4.1seconds and freezes on the first decode, or rolls until Stop on failure.
- Video of the full car, barcode sheet, sensor path and junction if tested.
  No video is needed for the stationary network check.

The dashboard still does not download stored RTOS rows: use the two export
helpers. Live graphs are20Hz, not the2ms raw barcode recording. On a failed
scan, stop promptly so a later wait does not overwrite its raw trace. On each
new Start, RAM run logs are replaced. Never send Wi-Fi passwords/settings UF2.
The unchanged line CSV pending/action fields encode canonical A/F/Z turn
codes; use barcode event export for the actual Aâ€“Z letter, action and revision.

Review BARCODE/BARCODE_DECODE/ADC and NET_TX/NET_MEMORY in USB capture. The
export result COMPLETE means all rows arrived, not physical acceptance.
Hardware success, control deadlines and heartbeat stability must be checked
together before Step5 or promoting anything to main.
