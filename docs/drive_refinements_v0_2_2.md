# Test Buddy 1–4 RTOS v0.2.2

This provisional release adds the three requested drive refinements to v0.2.1.
Buddy 5 is not integrated. Software checks do not establish physical bend or
stationary-search success; these are the next hardware acceptance checks.

## What changed

- A mapped pending barcode action may qualify a **7–30 mm** black stripe.
  The detector still requires 4 mm fresh white before and after the stripe.
  Layout assumption: the decoded label is followed by its junction before
  another barcode. A 7 mm width alone cannot exclude every wide barcode bar.
  Threshold learning, barcode acceptance, junction geometry and turn logic stay
  unchanged. Dashboard C7 reports the latest measured width and its result.
- Sustained same-side optical demand can qualify bend assistance. Candidate
  persistence is 100 ms; stronger authority additionally needs 180 ms demand
  and 80 ms correctly signed encoder turning. Candidate speed scales toward
  80%; confirmed steering authority can reach 110% of the existing request.
  Entry/exit blend and ordinary wheel slew limits remain bounded. These are
  provisional first-trial constants, not tuned or guaranteed circuit values.
- Confirmed line loss uses bounded counter-rotating wheel searches instead of
  forward rolling arcs. Wheels settle before direction reversals and before
  returning to forward following. Targets are +/-24 mm/s with a 300 PWM cap,
  80 ms encoder-still dwell (600 ms settle timeout), +/-10 then +/-20 degree
  waypoints, a hard +/-25 degree bound, 120 mm accumulated path limit,
  1.2 s no-motion limit and 5.25 s total search budget. The existing <=150 ms
  brief-gap bridge and two-second loss confirmation precede this search.
  With no credible direction memory, the bounded sweep starts left and then
  covers the opposite side; this is a symmetric search, not inferred line location.
  No extra reverse permission is granted to
  ordinary tracking. Dashboard C8 reports bend and recovery state.

No recalibration or higher global PWM cap is introduced. Saved Wi-Fi/calibration,
Profile 1 straight speed (45 mm/s), both barcode decoders, IMU/ramp logic,
Micro T-Kernel tasks, shared APIs/coordinator, network supervision and stop
guards are retained. The weak-bar recovery idea was not added to the decoder.

## Setup

1. Extract the complete v0.2.2 bundle into a new folder. Stop the robot and
   finish any previous downloads before flashing.
2. In BOOTSEL mode copy `firmware/robopico_buddy1_4_rtos_v0_2_2.uf2` to RPI-RP2.
   Existing saved Wi-Fi and calibration remain. Keep the robot flat and still
   for IMU startup calibration.
3. Close the previous dashboard helper with Ctrl+C. Run **Start dashboard.cmd**
   from the new bundle and hard-refresh the page (Ctrl+F5). Keep it open.
   On Mac run `python3 tools/dashboard/run_dashboard.py` from the bundle root.
4. If network name/password/broker IP are unchanged, no Wi-Fi setup is needed.
   Otherwise use **Setup Wi-Fi.cmd**, or on Mac
   `python3 tools/rtos_migration/provision_network.py`, and follow its
   settings-only BOOTSEL instructions. Do not reflash the application afterward.
5. Confirm car online, firmware v0.2.2, Profile 1 and IMU ready. Keep the same
   saved calibration for this comparison (tested car: white 122/138,
   black 246/267; actual readback is authoritative). Apply/read back mappings.

Powerbank operation is supported. No USB capture or connected-test helper is
required to start. Use the working motor supply as well as Pico power.
Dashboard Start retains the two-second countdown; Stop/GP21 and GP20 ESTOP remain.

## Purposeful physical checks

Keep Telemetry on. Video is especially useful for the bend and search tests.
Use enough mat under the sensors/wheels throughout each movement.

1. **Straight baseline:** run a short straight, then Pause/Resume and Stop.
   Check smoothness against v0.2.1; brief alternating corrections should not
   repeatedly show confirmed bend assistance.
2. **Sharp bend:** run one known troublesome bend, then Stop. State whether
   it stayed on the line, ran wide, stopped, or entered recovery. Include the
   bend direction and approximate point where the behaviour changed.
3. **Stationary recovery:** in a supported, clear area allow the front sensors
   to lose the line. Check that the search counter-rotates rather than advancing
   through rolling arcs. Observe both a successful return and Stop during a
   search. If a wheel does not move, preserve the failed attempt rather than
   increasing power or repeatedly restarting. Some slip/drift is physically
   possible despite zero intended centre velocity.
4. **Junction:** use the decoded-F/previously missed junction route. Check
   **Latest junction measurement** for width/result and whether the turn was
   dispatched. Barcode stop bars must not be reused as junctions.

No repeat ramp or exhaustive network reconnect test is required for this change.
Those algorithms and network guard limits are unchanged. A new physical failure
is evidence to review, not permission to relax a guard.

## Preserve and send the logs

After each useful failure, Stop and download before another Start, Reset or
power cycle. Keep Pico power on until downloads complete. The histories are RAM.

- **Export all frames CSV**, **Export event CSV**, and Messages **Save log**.
- Stored data: **Diagnostics**, **Navigation rows**, and **Raw sensor rows**,
  one at a time; wait for Complete and keep CSV/JSON/BIN together.
- Include the video and a short description of the route/outcome. Add the active
  `dashboard_session/broker.log` if a network loss also occurred.
- ZIP large files without trimming. The optional **Capture wireless test.cmd**
  passively records MQTT; it is not needed for permission to drive.

C7/C8 are small binary copied snapshots. Stored-export JSON freezes the latest
received snapshots at download request, rather than asserting exact per-row
association. All-frames/events provide their time history. The 640 controller
rows cover about 6.4 seconds; preserve a failed bend promptly.

## Calibration reminder

`w/l/r` change front calibration in RAM; `k` saves it. For a reversible trial,
leave `k` unpressed and power-cycle to reload the existing save. `c` restores
the older built-in white 163/160, black 3457/3700 preset, not the current
122/138, 246/267 save. No preset values are forced by this release.
