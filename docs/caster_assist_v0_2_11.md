# RoboPico v0.2.11 caster-assist test build

This `barr-c-cleanup` build adds a short motor assist for the passive caster
catching on a mat seam. It uses Baseline 45 mm/s and the existing tracking
targets. Normal 60 mm/s and Fast 75 mm/s keep their previous behavior.

After at least 2 mm of encoder travel during ordinary tracking, 300 ms with
neither wheel reporting another 2 mm of net travel can trigger one push.
The controller adds up to 55 PWM counts per wheel with a forward target of
at least 25 mm/s and positive base output. Total PWM stays within the existing
420 cap. The boost ends when an encoder reports 2 mm of travel or after
250 ms, whichever comes first. A fresh 2 mm of travel is needed to arm again
after recovery. If neither wheel progresses by the deadline while still
eligible, the motors stop and latch **No progress** until Reset.

Junctions, line search, barcode scan mode and active ramp assistance are
outside this policy. The boost uses the existing motion commit, including
the physical and dashboard stop paths, network supervision and IMU pickup
protection. The existing 1 ms safety task enforces the boost deadline;
the underlying controller excludes the extra PWM from its feedback.
The dashboard uses the existing RTOS stall bit and adds a **No progress**
event. MQTT packet layouts and stored calibration layouts are unchanged.

## Test on the mat

1. Extract the bundle, flash
   `firmware/robopico_buddy1_4_rtos_v0_2_11_caster_assist.uf2` and start the
   bundled dashboard. Confirm firmware v0.2.11 and Baseline 45 mm/s.
   Keep the car flat while the IMU gets ready. Apply the route mapping as usual.
2. Run **Capture wireless test.cmd** to record the test. On macOS, use
   `python3 tools/rtos_migration/capture_wireless.py` from the extracted folder.
   The capture is optional for driving. The robot can run from the power bank.
3. Drive across a flush seam and then the usual troublesome seam at Baseline
   speed. Note the approximate time and whether it crossed or stopped.
   If it stops, leave it powered and capture for another 15 seconds.
   Save the logs before Reset or power cycling.
4. From **Logs & exports**, export the **Event CSV** and the motion telemetry
   **CSV**. After stopping, download **Diagnostics** under **Stored data**
   if the run faulted. Send these files and the wireless capture MQTT CSV and
   summary JSON. The plotted wheel speeds, encoder travel and PWM can show
   the attempt. A latched stall is recorded as fault bit `0x10` and stop
   reason `8`.

The targeted software checks cover pulse timing and cap, a zero-target wheel,
rearm, scan cancellation, the existing ramp path and release packet format.
The ability to clear the mat seam needs a run on the car.
