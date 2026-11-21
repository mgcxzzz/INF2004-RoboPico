# System — test log

Tests of the assembled RoboPico. The [circuit record](logs/2026-11-21_run_4/README.md)
is the final physical evidence for the behaviours shown. Run 4 used v0.4.2;
the final v0.4.3 stopped-range tweak and dashboard warning were checked by
[replay](../integration/logs/2026-11-21_v0.4.3/README.md).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
| --- | --- | --- | --- | --- | --- |
| SYS-10 | Supervised Start and autonomous motion | FR01, FR02 | manual | Place the ready robot on tape, issue Start and record command, encoder and video times. | Start enters the nominal 2 s countdown, then the robot drives autonomously. |
| SYS-11 | Supervisory commands and rejection | FR02 | manual | Exercise Pause, Resume, Stop, Reset and a rejected Start. Save ACKs and actual state/output. | The car acknowledges requests, follows the appropriate state change and keeps zero output after an unsafe Start. |
| SYS-12 | Line tracking on straights and bends | FR05 | manual | Drive an unbroken straight and bend at Baseline 45 mm/s; record the path and motion state. | The robot follows the tape around the bend without steering commands. |
| SYS-13 | Physical barcode detection and decode | FR06 | manual | Traverse known labels and compare the printed letter to the accepted decode. | Detection yields the correct letter and mapped pending action. |
| SYS-14 | Right, U-turn and left at a junction | FR07 | manual | Read back mapping; film each mapped turn from approach through settled line tracking. | Right, U-turn and left execute at the junction and the robot resumes following the intended line. |
| SYS-15 | Straight-through barcode at a junction | FR07 | manual | Map a known letter to Straight and film its approach and exit. | The robot goes straight at the qualified junction and resumes ordinary guarding. |
| SYS-16 | One-wheel hump detection and peak reporting | FR08 | manual | Place the left wheel on flat mat and drive the right wheel over a hump. Save video and Buddy 4 events. | The robot crosses and reports an accepted peak in mm and the run's highest accepted value. |
| SYS-17 | Obstacle detection and pre-contact stop | FR11 | manual | Place a box on a straight; film approach and compare the stop to the obstacle trigger. | The robot detects the box and stops with visible clearance before its scan. |
| SYS-18 | Obstacle width report | FR11 | manual | Inspect the width report after a scan. Document the basis and retain raw width records where available. | The dashboard reports an estimate in mm with its basis or labels an unavailable/assumed value. |
| SYS-19 | Contact-free bypass and line rejoin | FR12 | manual | Keep box and original tape in view until avoidance completes, then correlate with handback telemetry. | The robot clears the box, rejoins the original tape and resumes Buddy 3 line following. |
| SYS-20 | Live MQTT telemetry in motion | FR03 | manual | Observe the dashboard during driving and record telemetry and heartbeats received. | The dashboard receives heartbeat, operating state and live subsystem data. |
| SYS-21 | Pickup, inversion and E-stop | FR09, NFR02 | manual | Run separate pickup/inversion and E-stop trials. Capture both wheels, fault latch, reset/restart and timing from qualified event to drive-off. | Motor drive goes to zero, pickup remains latched until eligible Reset, and the measured stop target is within 100 ms. |
| SYS-22 | Stopped run ZIP download | FR10 | manual | Stop before Reset or a new Start; export and inspect the manifest and stored files. | The ZIP includes page history and available motion, diagnostic, raw-sensor and navigation datasets. |
| SYS-23 | Line tracking repeatability and recovery | FR05, NFR05 | manual | Record 30 s of unaided tracking in each of three documented lighting conditions. Test a recoverable line gap and one beyond search reach. | The car tracks under each condition, recovers from the short gap and makes a bounded stop on the longer one. |
| SYS-24 | Barcode repeatability and rejection | FR06, NFR05 | manual | Record 20 traversals with known labels at nominal speed, plus invalid and valid-unmapped labels. | At least 19 traversals decode correctly. Invalid and unmapped labels queue no action. |
| SYS-25 | Hump measurement accuracy | NFR06 | manual | Measure the one-wheel hump and repeat passes. Add a flat control and an intentionally rejected sample. | Record accepted peak error against the 10% target. Keep live and rejected readings distinct from accepted peaks. |
| SYS-26 | Obstacle detection, width and bypass targets | NFR07 | manual | Use measured 50–200 mm targets at 200–400 mm and three materials. Log detections and trigger spacing. Measure ten bypasses and test one blocked route. | Target 20/20 detections, width error within 10%, trigger spacing at least 60 ms, and 9/10 contact-free rejoins within 10 s of clearing. Report a blocked route and stop. |
| SYS-27 | Link quality, loss and reconnect | FR03, NFR03 | manual | Run a five-minute wireless capture with sequence counts. In a separate trial interrupt the supervisory link. | Target at least 95% delivery and a maximum 2 s heartbeat gap. Loss stops the robot and reconnect requires a fresh Start. |
| SYS-28 | Stopped speed, barcode mapping and calibration | FR10 | manual | Change speed and mapping while stopped and verify readback. Check RAM and saved calibration after reboot and the running-state command gate. | Confirmed settings match the car. Saved calibration survives reboot and stopped-state restrictions apply. |
| SYS-29 | Encoder distance and turn accuracy | FR04, NFR04 | manual | Compare odometry with external distance and angle references across ten trials; verify the 45, 60 and 75 mm/s target selections. | Distance and angle are within ±5% and ±5° respectively under the documented conditions. |
| SYS-30 | Pico W task rates and power-bank use | NFR01 | manual | Capture on-device timing or GPIO traces for motor, IMU and safety tasks while power-bank powered. Check the configured FIFO sample rate. | Nominal periods are 10 ms motor, 20 ms IMU FIFO service and 1 ms safety wait. The accelerometer FIFO is configured at 400 Hz. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-21 | SYS-10 | v0.4.2 (manifest) | IS28 team | PASS (functional) | [Events](logs/2026-11-21_run_4/events.csv) | Start ID 54 accepted. First encoder movement appeared about 2.4 s after ACK receipt. |
| 2026-11-21 | SYS-12 | v0.4.2 (manifest) | IS28 team | PASS | [Events and timestamps](logs/2026-11-21_run_4/README.md#recording-and-log-sync) · [Line following](https://youtu.be/DSKYzSl4pN8) | Straight tracking and a bend visible in the recording. |
| 2026-11-21 | SYS-13 | v0.4.2 (manifest) | IS28 team | PASS (examples) | [Decode events](logs/2026-11-21_run_4/README.md) · [Barcode scans](https://youtu.be/gGe0tv3TXEg) | Coordinator accepted Z, Q and A. |
| 2026-11-21 | SYS-14 | v0.4.2 (manifest) | IS28 team | PASS | [Turn events](logs/2026-11-21_run_4/README.md) · [Right](https://youtu.be/u8M4bE7olWs) · [U-turn](https://youtu.be/aMheU_oI-9U) · [Left](https://youtu.be/lvsAOKSMIwc) | Right, U-turn and left completed; the video shows the outgoing paths. |
| 2026-11-21 | SYS-16 | v0.4.2 (manifest) | IS28 team | PASS (functional) | [Hump events](logs/2026-11-21_run_4/README.md) · [Both crossings](https://youtu.be/LSbH6W17-fk) | 12 mm and 16 mm accepted. Both crossings are shown. Reference height was unmeasured. |
| 2026-11-21 | SYS-17 | v0.4.2 (manifest) | IS28 team | PASS | [Obstacle events](logs/2026-11-21_run_4/README.md) · [Approach and stop](https://youtu.be/jYbEuhxsAzg) | Trigger reported at 284 mm. The recorded first obstacle was detected before contact. |
| 2026-11-21 | SYS-18 | v0.4.2 (manifest) | IS28 team | PASS (reporting) | [Width event](logs/2026-11-21_run_4/events.csv) · [Width probe](https://youtu.be/jYbEuhxsAzg) | Estimate reported as 138.7 mm; the trace missed one record. Accuracy needs a reference width. |
| 2026-11-21 | SYS-19 | v0.4.2 (manifest) | IS28 team | PASS | [Bypass events](logs/2026-11-21_run_4/README.md) · [Bypass and rejoin](https://youtu.be/jYbEuhxsAzg) | Robot cleared the first obstacle and rejoined. Handback recorded. |
| 2026-11-21 | SYS-20 | v0.4.2 (manifest) | IS28 team | PASS (functional) | [MQTT receipt summary](logs/2026-11-21_run_4/mqtt.txt) · [Live dashboard](https://youtu.be/v_6Jqraz99s) | 16,749 telemetry frames and 370 heartbeats received in the evidence window; largest observed gap 1.218 s. |
| 2026-11-21 | SYS-22 | v0.4.2 (manifest) | IS28 team | PASS | [Capture record](logs/2026-11-21_run_4/capture.json) | The export contains the four completed stored datasets and page-session history. |

Cases without a dated result above remain open on the final setup. Earlier
runs and their evidence are retained in the [archive](../evidence/README.md).
