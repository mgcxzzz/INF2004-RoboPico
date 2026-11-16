# Buddy 1–4 RTOS v0.2.4: correct IMU safety reporting clock

v0.2.3 could display **IMU not ready — motion inhibited** while the actual guard
was healthy. The C9 report compared kernel uptime with hardware-timer sample time.
They run at the correct rate but start at different offsets; unsigned subtraction
could therefore incorrectly mark a fresh sample stale.

v0.2.4 makes the reporting API obtain hardware time itself. C9 uptime, sample time,
trip time and freshness now use one epoch. Callers cannot supply a kernel clock.
No gesture thresholds, calibration, actual motion permission, latches, motor outputs,
line/barcode/junction/bend/recovery logic, IMU/ramp processing, or network guards
are relaxed or retuned by this correction. Buddy5 remains not integrated.

## Update and check

1. Stop and preserve any useful dashboard logs before flashing. Extract the new
   complete bundle. In BOOTSEL copy `firmware/robopico_buddy1_4_rtos_v0_2_4.uf2`
   to RPI-RP2. Saved Wi-Fi/front calibration remain; do not run w/l/r/k/c for this fix.
2. Start the new dashboard helper and hard-refresh the page. Reuse working Wi-Fi
   settings. No USB capture helper is required to start the robot.
3. Keep it flat and still for calibration. After about10 seconds, check fresh IMU
   safety status, armed, upright500ms and reset permitted. A healthy unlatched
   guard should show **No IMU safety stop latched**.
4. A retained event count can survive Reset; it is not proof of a new event.
   Trip uptime0 after Reset means the detector's previous latch was cleared.
   Putting the robot down never automatically starts it; use explicit Start.
5. If Start is still rejected or no motion follows, preserve **Export all frames
   CSV**, **Export event CSV** and Messages **Save log** before another Reset or
   power cycle. Include full IMU safety details and the Start acknowledgment.
   The status correction alone cannot establish every cause of a failed Start.

The [v0.2.3 lift/overturn gesture and physical checks](imu_end_run_v0_2_3.md)
still apply. Its C9 wire layout stays0x23 for compatibility. Within C9, all times
are hardware timer milliseconds; do not assume identical zero points with other
source packets that explicitly use kernel uptime.

## Focused software evidence

The actual service regression first reproduced the failure with a137ms kernel/
hardware offset. It now passes with kernel time both ahead and behind hardware
while keeping the internal guard and C9 freshness/reset flags consistent. The test
also retains the lift latch and fresh/upright Reset checks. Release metadata/export
and four C9 browser-codec checks passed. The ARM image and packaging gates verify
the new reporting API is linked. This is not a new hardware acceptance result.
