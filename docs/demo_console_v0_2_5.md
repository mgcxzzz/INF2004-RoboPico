# Demo console and drive modes v0.2.5

**Provisional Buddy 1–4 micro T-Kernel SMP release. Buddy 5 is not integrated.**

This release retains the tested Step 5 baseline and the subsequent production
barcode, junction, bend and recovery changes. The new dashboard uses the approved
Demo overview layout. Its values come from car telemetry; the design preview's
simulated values are not used in production.

## Install

Download and extract the complete
[v0.2.5 bundle](../firmware/integration/buddy1_4/artifacts/v0_2_5_rtos/robopico_buddy1_4_rtos_v0_2_5_bundle.zip).
Stop the robot, use BOOTSEL, and copy
`firmware/robopico_buddy1_4_rtos_v0_2_5.uf2` to RPI-RP2.
Run the new **Start dashboard.cmd** and hard-refresh the page. Both the firmware
and dashboard changed: flashing only the UF2 leaves the old UI without the new
mode controls and diagnostic decoders.

Saved Wi-Fi and front-sensor calibration are preserved. Do not use w/l/r/k/c
just for this update. Keep the Pico flat and still for startup IMU calibration,
confirm car-online and fresh IMU data, then apply and read back the barcode map.
Existing working Wi-Fi settings do not need provisioning again. See
[setup](setup.md) for Windows/Mac commands and broker configuration.

No PuTTY, USB serial connection or Capture connected test helper is required to
drive. The laptop broker/dashboard and motor supply must be running. Powerbank
operation, the two-second Start countdown and explicit Stop/Reset behavior remain.

## Overview

The top contains run state, controls, the drive-mode selector and six cards:

- **Line and motion:** tracking/recovery, target and measured speeds, left/right
  PWM, and whether bounded load compensation is active.
- **Barcode and junction:** detection, decoded letter, queued action and completed
  action are separate. A decode is not proof that a turn completed.
- **Hump and ramp:** live height estimate is separate from accepted hump count,
  last accepted height, maximum accepted height and ramp assistance.
- **Obstacle and avoidance:** detection, width and bypass/rejoin are visibly
  unavailable until Buddy 5 is integrated.
- **Wi-Fi and MQTT:** broker connection, car-online and heartbeat freshness are
  separate indicators.
- **IMU and pickup stop:** armed/current pose, recorded trip, and reported motor
  output are separate evidence. Armed does not mean a pickup stop was demonstrated.

The mapping editor, all telemetry graphs, diagnostics, messages and exports remain
available below. Missing or stale data must not be interpreted as a successful
zero reading. The overview is evidence for the demo, not an automatic rubric grade.

## Speed modes and load compensation

| Mode | Nominal cruise | Intended use |
| --- | ---: | --- |
| Baseline | 45 mm/s | Default on power-up; begin here |
| Normal | 60 mm/s | Trial after a successful Baseline circuit check |
| Faster trial | 75 mm/s | Optional trial after Normal is controlled |

The mode is held in RAM. A power cycle returns to Baseline. Change mode only after
Stop and completion of any history download; a paused run or pending Start is
not eligible. Wait for car readback to confirm the applied mode. A command ACK
alone does not confirm application. Unsupported older firmware cannot be changed
using this selector.

These remain Profile 1 controllers with a separate cruise setting. Barcode
scanning/approach remains at its bounded scan speed; junction turns and line
recovery keep their existing motion policy. Qualified bends retain their slowdown
and steering response. This is not activation of the old experimental Profile 3.

When a wheel remains below its own commanded speed, the controller may gradually
add bounded PWM reserve. It releases the reserve as speed recovers. A deliberately
slower inside wheel is judged against its own target, and a zero target does not
receive forward assistance. Existing output cap, slew, stop guards and ramp
assistance still apply. This cannot guarantee that a physical obstruction will
be crossed; a stuck robot can still trigger the existing safety guards.

## Pickup stop correction

The calibrated raw accelerometer guard now qualifies either:

- tilt of at least **45 degrees for 100 ms**, with a valid gravity norm; or
- upward acceleration at least **400 mg above its calibrated baseline for 30 ms**.

The guard latches a stop through the existing safety/coordinator path for all
motion, including turns and ramp assistance. Returning the robot to the mat does
not restart it. Place it upright/still, use an eligible Reset, wait for calibration
and readiness, then issue a new Start.

Freshness checks now read the clock after taking the protected sample snapshot.
This prevents a sample arriving during lock acquisition from appearing to be in
the future. The 100 ms freshness requirement is retained.

A new diagnostic snapshot retains the greatest tilt evidence and dwell peaks
**since explicit Reset**, including rejected candidates. It is diagnostic history,
not proof of a trip during the current run. Last nonzero candidate and last
norm/invalid rejection are retained independently; they need not refer to the
same sample as the greatest valid tilt. A very slow, level lift may still be
indistinguishable from resting; an IMU does not measure ground contact directly.

## Focused hardware checks

These changes require physical acceptance. The earlier 50 Hz IMU/ramp results
do not certify the new gesture threshold or faster cruise modes.

1. Start at Baseline 45 on a clear supported mat. Confirm ordinary straight
   tracking and Stop. Check that a routine turn does not trigger pickup stop.
2. With a clear area and access to GP20/GP21, make a deliberate supported lift
   and tilt beyond 45 degrees for at least a quarter second. The wheels should
   stop and remain stopped when replaced on the mat. Use Stop/GP20 immediately
   if they continue. Record a video showing the lift and wheels.
3. Confirm Start cannot bypass the latched stop. Place it flat/still, Reset,
   wait for fresh calibration/readiness, then Start explicitly.
4. Run one normal ramp to check for a false pickup stop and confirm ramp
   assistance and accepted height remain observable.
5. After Stop, select Normal 60, wait for applied readback, and test a straight
   and bend. Only try 75 after 60 is controlled. Check smoothness, barcode
   scanning and junction actions before using a faster mode for the demo.

For a fault, stop and keep the robot/broker powered. Export **all frames CSV**,
**event CSV**, **Messages → Save log**, and stopped **Diagnostics** and
**Navigation** datasets. Add **Raw sensors** for barcode failures and **Motion
rows** for motion issues. Optional **Capture wireless test.cmd** records the full
test passively and is not a driving prerequisite. Download before another Start
or reboot; retained car histories are bounded RAM buffers. State the selected
mode, barcode mapping, failed action and whether any cable/hand contact occurred.
