# Step4 v0.1.1 — Y decodes, but no junction action

Experimental micro T-Kernel Buddy1–3 checkpoint; IMU/ramp Step5 and Buddy5
are not enabled. Main is untouched. Physical acceptance remains pending.

## What changed

The 2026-11-04 17:17:40 capture decoded reverse Y at run5.649s and froze U-turn
at mapping revision1. Peak decoding succeeded, but the cross detector used
the undecided63-bank preview212. Its minimum raw ADC was234, so it stayed
black and produced no cross candidate. This was before turn dispatch, not
an angle/acquisition failure. The raw trace freezes at decode; the later
junction waveform comes from the100Hz stored line trace.

v0.1.1 keeps both Code39 algorithms/thresholds, Continuous V2 steering, speed,
PI/PWM limits, front calibration priority, networking and safety unchanged.
After a verified frame it creates a separate junction threshold at the
midpoint of measured white and strong-bar levels, with5% contrast hysteresis.
For peak frames the levels come from that decoded frame; for original
consensus they are the run ADC extrema through that acceptance. No fixed
mat ADC threshold is added. This classifier cannot decode letters itself.
The existing decoder's diagnostic threshold may still say212: that is now
separate from `JUNCTION_SENSOR threshold=...`.

After decode or qualified Resume it requires a fresh4mm white interval
before taking any leading edge. It retains2ms debounce, an11–30mm isolated
black run,4mm trailing white and the original optical straight gate. A
return to black before4mm cancels the candidate. The stop bar is not a cross.
Junction confirmation/dispatch still goes through the existing Buddy3 nav
coordinator API and single motor writer. A frozen action/revision remains
unchanged by later mapping edits. No sharp-bend or turn-acquisition changes.

Horizontal GP27 sensing-point-to-axle distance is now110mm rather than92mm:
18mm additional forward approach travel before turn initiation. Wheel track
width110mm and rolling offset55mm are separate measurements and unchanged.
Other migration profiles and the SDK retain their default92mm sensor offset.

## Setup and one primary test

1. Extract this complete bundle into a new folder. Close the previous
   dashboard terminal and PuTTY/capture programs.
2. With motor power off, BOOTSEL and copy
   `firmware/robopico_rtos_connected_barcode_v0_1_1.uf2` to RPI-RP2.
   Saved Wi-Fi and calibration are retained. Do not press w/l/r/k/c. No Wi-Fi
   setup is needed if the network and broker IP are unchanged; otherwise run
   **Setup Wi-Fi.cmd** and follow its BOOTSEL prompt for settings only.
3. Run **Start dashboard.cmd** and leave it open. Hard-refresh
   http://localhost:8765/ and connect ws://localhost:9001. Close PuTTY before
   running **Capture connected test.cmd** and selecting the actual COM port.
   That capture must remain actively reading to keep mandatory USB DTR
   supervision open. It runs300s by default. Mac: use the corresponding
   `python3 tools/dashboard/run_dashboard.py` and
   `python3 tools/rtos_migration/capture_connected_line.py` commands.
4. Confirm USB banner **CONNECTED BARCODE v0.1.1**, CLOCK PASS, LINE flags0x3,
   NET connected1/subscribed1, and the existing front calibration. The last
   tested car was white122,138 / black246,267, origin1. Do not force these
   values onto a different car. `JUNCTION_SENSOR offset_mm=110` must appear.
   Its valid/armed flags may be0 before the first verified label: expected.
5. Apply/read back **Forward=N, U-turn=Y, Left=Q, Right=Z**. Mapping is RAM
   state and must be reapplied after flashing/rebooting. Turn on battery/motor
   power for the physical run; USB alone was insufficient in earlier tests.
6. Repeat the same stationary, flat Y sheet followed by a separate cross.
   Keep the entire sensor sweep on supported white mat and cable loose.
   Click Start, wait the2s countdown. Expect Y decoded/queued, junction
   confirmed, U-turn dispatched, then completion and line following. Keep
   GP20 E-stop/GP21 Stop accessible. Video the full car and sensor path.
7. Stop promptly after completion or failure. Do not reposition and start
   another run before exporting. Keep Pico power connected. Let the capture
   reach at least30s total, then Ctrl+C while stopped. Wait until it releases
   the serial port. Leave the dashboard/broker running.
8. Run **Export barcode log.cmd**, then **Export line log.cmd**, one at a time.
   Mac: `python3 tools/rtos_migration/export_barcode_log.py`, then
   `python3 tools/rtos_migration/export_line_log.py`. No reflash or reboot
   between the run and exports. The dashboard still cannot download stored
   RTOS histories. These exports do not start another run.

## Send back

- `rtos_connected_session`: `_summary.json`, `_usb.txt`, `_mqtt.csv`.
- `rtos_barcode_session`: `_summary.json`, `_usb.txt`, `_events.csv`, `_raw.csv`.
- `rtos_line_session`: `_summary.json`, `_usb.txt`, `_line.csv`.
- Video showing Y, the separate junction, front/side sensor support and turn.
- `dashboard_session/broker.log` if heartbeat delivery stalls or disconnects.

Line history holds the last6.4s, so stop/export promptly. Raw GP27 freezes on
the first decode: it normally covers the label, not the later junction. USB
status plus stored line ADC/side_black and barcode events cover that junction.
The line CSV uses canonical A/F/Z route bits: an actual Y mapped to U-turn can
appear as90 in that field. Barcode event CSV contains the actual Y/action/rev.

If that primary test works, use separate runs for reverse-print Y, then left
Q and right Z on adequate mat, followed by a sharp bend. Do not claim Step4
accepted solely from a decode or software replay. Missing alignment after a
confirmed dispatch is a separate turn/acquisition check.

## Software evidence

`testing/rtos_migration/fixtures/step4_y_junction.csv` preserves the observed
label and later junction ADC/distance; the companion provenance JSON records
input hashes and the100Hz downsampling limit. Production-model replay now
decodes Y once and confirms one cross13.78mm wide, using junction1181 with
hysteresis94 while decoder preview remains212. This proves the prior stuck
classification is removed on those samples; it does not prove real turn
alignment, network latency, sensor support or control deadlines on the car.
