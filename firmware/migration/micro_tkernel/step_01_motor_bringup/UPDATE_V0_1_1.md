# Step1 v0.1.1: startup delay and readiness reporting

Date:2026-11-02. This is a Step1 correction, not Step2 line following.

## Evidence from v0.1 hardware tests

The supplied40s capture measured40.008623 laptop seconds,40.01 kernel seconds,
40.010002 TIMER seconds and100.002 control updates/second, with no new late
intervals. Nevertheless, all captured control rows had flags0x2c: ready and
clock-valid plus a latched earlier control-delay fault. The historical maximum
interval was132256us. Later GP20 added an E-stop latch (0x2e).

After explicit x reset, the extended PuTTY log contains three accepted requests:
left45/right0, left0/right45 and both45mm/s. Encoder directions were correct,
invalid-transition counts remained zero, all three ended by the automatic pulse
timeout, and the late counter did not increase. The user separately confirmed
s, GP21, GP20 and e stops worked. These are v0.1 observations; v0.1.1 cold-start
behaviour still requires the brief retest below.

## Correction

- A >=50ms gap while already stopped is retained in the historical counters.
  It inhibits new starts and resets the clock-good streak to zero. Three NEW
  valid one-second windows are required before starts are eligible again.
- No stopped delay automatically clears an E-stop, stall, encoder or existing
  active-run delay latch. x remains necessary for those latched faults.
- A >=50ms gap during an active run still stops and latches. The independent
  Safety Supervisor and generation-checked output gate remain in place.
- The USB analyzer now reports timing_result and motor_readiness separately.
  Stable timing with an older latch reports overall BLOCKED, not PASS. It also
  checks current USB, button and IRQ state and incomplete/stale readiness data.
- No shared controller, nav API, encoder constants, PWM settings or calibration
  sources from the working main reference were modified.

## Brief retest

1. Flash robopico_rtos_motor_bringup_v0_1_1.uf2. Fully remove USB and battery
   power, then reconnect normal USB with the robot stationary and wheels raised.
2. Do not press x. Run the new bundle's Run timing check.cmd. Require overall
   result PASS, timing_result PASS and motor_readiness READY. Send the output
   files if any differs. An earlier stopped delay may still appear in max_dt_us
   and late, but flags should recover to0xc after three fresh good windows.
3. Open PuTTY at115200, enable session logging, enable normal motor power and
   press3 once. Both wheels should move and stop automatically after3s.
   During one additional pulse, press GP20: starts must remain rejected until
   it is released and x is pressed, followed by1s rest. This verifies that the
   startup correction has not made emergency stops auto-clear.

After this passes, proceed to Step2 optical line following. Wi-Fi, barcode,
IMU and Buddy5 application tasks remain disabled in this checkpoint.
