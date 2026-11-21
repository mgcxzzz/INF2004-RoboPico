# Tribunal 2 verification — IS28 RoboPico

Use this index with the team's `IS28_PreTri2.pdf`. The FR and NFR numbers
below match that document. It describes the Buddy 1–4 baseline and records
FR11 and FR12 as course requirements. Verify those two on the combined
Buddy 1–5 build. The figures below are test targets. Record the observed
value, release and outcome for each claim.

## Build and repeat the automated checks

Record the exact Git commit, build profile (`APP=rtos_buddy1_5` for the
combined robot), UF2 SHA-256 and dashboard version in the test results. Run
from that commit with a host C compiler and Node.js. Build the firmware using
the [RTOS build instructions](../docs/build.md) for the selected release.
See the [v0.4.3 release verification](evidence/tribunal2/v0.4.3/README.md)
and the [synchronized v0.4.2 hardware run](evidence/tribunal2/run-4/README.md)
for results and recording timestamps.

```sh
python tools/rtos_migration/run_tests.py
node --test dashboard/test/test_b5_rtos.js dashboard/test/test_imu_lift_v023.js dashboard/test/test_wireless_calibration.js dashboard/test/test_source15.js
node dashboard/test/test_release.js
python tools/coding_standard/check.py
python tools/coding_standard/format.py --check
```

Run the C suite first so the dashboard tests can use C-generated packet
fixtures. With Playwright and Chrome available, also run
`node dashboard/test/ui_release.mjs` for the simulated MQTT and run-export
flows. Save the command output with the tested commit. See [existing host
checks](../docs/testing.md), the [application C standard](../CODING_STANDARD.md),
its [documented exceptions](../docs/coding_exceptions.md), and the
[data representation](../docs/data_representation.md) for coding and packing
evidence.

| Automated coverage | Representative checks |
| --- | --- |
| Start, stop and motion coordination | `test_bringup_policy`, `test_release_policy`, `test_network_stop`, `test_nav_steer`, `test_encoder_fault` |
| Line sensors, recovery, barcode and junction | `test_line_model`, `test_rtos_barcode`, `test_barcode_attempt`, `test_junction_sensor`, `test_junction_policy`, and the line-following checks under `firmware/integration/buddy1_4/tests/unit/` |
| Ramp and pickup | `test_rtos_ramp`, `test_rtos_ramp_bridge`, `test_imu_lift_guard`, `test_imu_service_rtos` |
| Buddy 5 ownership, width fallback and rejoin | `test_b5_straight_gate`, `test_b5_mission`, and `dashboard/test/test_b5_rtos.js` |
| Supervisor messages, compact records, calibration and export | `test_mqtt_delivery`, `test_compact_signed`, `test_release_bridge`, `test_release_exports`, and `test_wireless_calibration` |

## Physical test cases

Keep the robot on its demo power supply. Record the mat, lighting, speed
mode, calibration and barcode mapping for each run. Use the case IDs below in
the evidence index and reuse one run across cases where it covers all the
expected results.

| ID | Requirement | Run and expected result | Evidence to save |
| --- | --- | --- | --- |
| T2-01 | FR01, FR02 | Start with the car ready. Check the 2 s countdown from accepted Start to drive. Drive without steering commands. Exercise Pause, Resume, Stop and Reset. Check that a rejected Start supplies a reason and leaves output at zero. | Command ACKs, Pico timestamps, state transitions and wheel PWM. |
| T2-02 | FR05, NFR05 (line) | Drive a straight and both bend directions in Baseline mode. Record 30 s of unaided tracking under each of three documented lighting conditions for the repeatability target. In separate trials, place a recoverable gap and then a gap beyond search reach. Verify recovery in the first trial and a bounded stop in the second. | Motion and diagnostics, with an overhead clip of the bend and the physical line recovery. |
| T2-03 | FR06, NFR05 (barcode) | Run 20 traversals of known printed labels at nominal speed. Count detections and correct decoded letters separately. Target at least 19 correct decodes. Also run invalid and valid-unmapped labels and verify they queue no movement. | Label ground truth, attempts, accepted letters and queued actions. |
| T2-04 | FR07 | Apply and read back a known mapping. Run Straight, Left, Right and U-turn at a qualified crossing, recording each command and the expected outgoing branch. Verify completion and resumed tracking. Check 7–30 mm and the 4 mm white margins with the junction unit tests plus a measured example stripe. On v0.4.1, repeat the turn with a reflector outside its swept path, then exercise Straight with an obstacle ahead. | Decode, pending action, stripe decision, dispatch and completion. Overhead clip shows the branch actually taken. Buddy 5 status covers the turn-versus-straight regression. |
| T2-05 | FR08, NFR06 | Measure the reference hump before testing. Keep it under the right drive wheel and the left wheel on flat mat. Traverse it repeatedly and record peak error against the 10% goal. Run a flat control and a rejected trial. Verify the accepted peak and maximum are retained and distinguish a rejected peak, including the starred display where supported. | Hump events and counts, measured reference height and a side/rear clip showing the wheel placement and crossing. |
| T2-06 | FR11, NFR07 (detection and width) | Place measured obstacles at known distances on a straight. Verify detection and stop clearance, record actual versus reported width, classify estimate versus fallback, and measure sonar trigger spacing. Track the Week 6 targets of 20/20 detections across three materials, width error within 10% for 50–200 mm targets at 200–400 mm, and triggers at least 60 ms apart. | Reference dimensions and placement, trigger range, width trace and basis. Keep assumed widths out of the accuracy pass count and report those trials explicitly. |
| T2-07 | FR12, NFR07 (bypass and rejoin) | Run ten obstacle trials with the original line visible before and after the obstacle. Check no contact, the correct rejoin, and resumed Buddy 3 tracking. Score the 9/10 target using elapsed time from clearing the obstacle to rejoining, with a 10 s limit. Use a separate controlled blocked-route case for the safe-stop outcome. | Obstacle stages and handback plus overhead video showing clearance, which line was rejoined and when. |
| T2-08 | FR09, NFR02 | While moving in a controlled setup, lift and invert the car in slow and brisk trials. Check both wheels cease being driven, the trip stays latched when set down, and Reset then explicit Start is required. Exercise E-stop separately. Check the 100 ms target from accepted E-stop or qualified IMU event to drive-off with synchronized on-device timestamps or a GPIO/PWM capture. | Pickup clip showing the gesture and both wheels, C9 trip, safety state, and motor output. Use captured timing for the numerical latency claim. |
| T2-09 | FR03, NFR03 | Run a 5 min Wi-Fi/MQTT capture with telemetry enabled. Compute delivery from per-source counters or sequence gaps and record the maximum heartbeat gap. Targets are ≥95% delivery and ≤2 s maximum gap. In a separate run cause controlled link loss and check stop, reconnect and the need for a fresh Start. | Capture Wireless Test summary, MQTT/runtime CSVs, and stop/reconnect events. Record unintended monitor gaps separately. |
| T2-10 | FR10 | Stopped, change speed and mapping and confirm car readback. Exercise front-IR capture and apply, check RAM and saved behavior on reboot, and confirm the dashboard respects stopped-state gates. After a driving run press Stop and export one ZIP. | Readbacks, calibration diagnostics and a ZIP with manifest, dashboard data and expected stored datasets. |
| T2-11 | FR04, NFR04 | Verify wheel feedback and single normal motion owner. Compare encoder distance and turns against an external reference across ten documented trials for the ±5% distance and ±5° turn targets. Verify selection of 45, 60 and 75 mm/s as commanded targets. | Measured vs commanded distance and angle, wheel commands and speeds, and ownership events. |
| T2-12 | NFR01, NFR08, NFR09 | Boot the release on the Pico W using the power bank. Check the nominal 10 ms motor task, 20 ms IMU FIFO service, 400 Hz sensor configuration and 1 ms safety wait against the code and available timing data. Review packed widths, masks, byte order, reserved bits, ownership and bounded buffers. Compare RAM and payload size with an equivalent baseline. | RTOS timing or GPIO traces, build/map output, actual C-to-dashboard vectors, and coding checks plus manual exception review. |

The Week 6 obstacle width target covers a wider range than the combined
long-probe display policy, which limits estimates to 100–200 mm and may show
a labelled 150 mm assumption. Record any scope or performance gap explicitly
against the reference measurement. See the [Buddy 5 width policy](../docs/buddy5_rtos_integration.md#width-in-the-mission-long-probe).

## Evidence already available

The [repository archive](evidence/README.md) contains pinned earlier runs and
provenance. Its `current/` folder describes provisional Buddy 1–4 host checks.
The [Step 5 archive](evidence/rtos_buddy1_4/step5_v0_1_5/README.md) includes an
earlier flat control and a 13 mm accepted reading on a nominal 10 mm ramp.
That is evidence of a detected hump and a 30% error against the nominal
height. Keep the functional and accuracy results distinct.

The captures below were supplied during development. The timestamps with
`Z` are UTC, as recorded in the dashboard event CSV. The [selected event
rows](evidence/tribunal2_historical_events.csv) preserve the relevant messages
and their source ZIP hashes. Add the raw archive location when recording a
result in the repository.

| Capture | Verified evidence | Use |
| --- | --- | --- |
| `robopico_run_1_2026-11-21-03-05-42.zip` | v0.3.12, complete ZIP. Width estimate 100.00 mm at 03:00:24Z. Buddy 5 `HANDBACK` at 03:01:26Z. Accepted hump 11 mm at 03:01:58Z. `F → Left` and `A → U-turn` completed at 03:03:20Z and 03:05:13Z. | Historical functional evidence for T2-03 through T2-07 and a complete-export example for T2-10. Physical clearance and measurement accuracy still need the corresponding ground truth. |
| `robopico_run_2_2026-11-21-03-42-39.zip` | v0.3.13. A rejected 10 mm peak at 03:38:42Z and a later accepted 12 mm peak at 03:42:20Z. `HANDBACK` at 03:42:01Z. | Historical acceptance/rejection and handback examples. |
| `robopico_run_4_2026-11-21-07-05-45_PARTIAL.zip` | v0.3.12. Close-obstacle `NOT_STRAIGHT` HOLD at 07:05:26Z. The stored diagnostics download stopped after 170 rows. | Failure and partial-export evidence. Use the relevant event window and note the missing data. |
| `2026-11-19_15-22-40_summary.json` and matching MQTT/runtime captures | v0.2.9. Observer received 186 heartbeats and reported a 15.67 s maximum gap plus several monitor interruptions. | Historical connectivity failure analysis. Repeat T2-09 on the submission build. |

## Videos to record

1. **Line and junction (T2-02, T2-04).** Film overhead from barcode to junction exit. Show the actual printed letter, track layout and outgoing branch. Include a sharp bend and, if testing recovery, a separate short clip of the robot finding the tape again.
2. **One-wheel hump (T2-05).** Show the ruler or caliper and both wheels during the crossing. Keep the hump in view so a kicked or shifted ramp is visible. Pair it with the accepted peak in the same run.
3. **Obstacle and rejoin (T2-06, T2-07).** Show the measured obstacle, approach, clearance and original line. Keep filming until normal line tracking resumes. If scoring the 10 s target, mark when the robot cleared the obstacle and when it rejoined.
4. **Pickup stop (T2-08).** Show the lift, inversion, and both wheels stopping in one continuous shot. If the dashboard is out of view, match the clip to the trip and motor-output evidence from that run.

MQTT delivery, decoder correctness with documented labels, packet packing and
export integrity can be demonstrated with logs or automated results. Use the
[video index](../videos/README.md) to link physical clips as unlisted videos
and reference the test IDs and timestamps in the result rows.

## Record results

For each measured case, add a row like the one below. Start a fresh dashboard
capture for a new evidence run. After each run, press Stop and download the
run ZIP before Reset or another Start. Keep original captures with their
manifest and hash, and publish short excerpts with time ranges and the raw
archive link. Check shared evidence for private network details first.

| Date / tester | Case ID | Commit / UF2 / dashboard | Setup and trials | Observed / expected | Status | Evidence / time range |
| --- | --- | --- | --- | --- | --- | --- |
| | | | | | Pending | |

A case can have a historical result and a separate pending run for the
submission release. For a failure, record the observed outcome and later add
the rerun with its new commit. Retain the original result.
