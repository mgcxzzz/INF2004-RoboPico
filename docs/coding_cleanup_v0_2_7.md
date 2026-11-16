# Buddy 1–4 RTOS v0.2.7 cleanup candidate

Branch: **barr-c-cleanup**. This is a provisional coding-cleanup candidate based
on tested main v0.2.6, not the promoted main release. Buddy 5 is not integrated.

## What changed

The adopted policy is [RoboPico C Standard 1.0](../CODING_STANDARD.md), based on
Barr-C:2018 with [documented exceptions](coding_exceptions.md). Formatting uses
80 columns, four spaces, Allman braces and explicit control bodies. Application
types, variables and declarations are clearer; module contracts and include
guards are consistent. Logical grouping and constant-first comparisons are
explicit. Upstream dependencies and historical release bundles remain intact.

Signed bit-fields in lift state/packets, sweep direction, bend direction,
drive diagnostics, network-stop snapshots and IMU events use unsigned compact
storage with explicit signed conversion. Their bit widths, negative ranges and
wire encodings remain. Seven native layout size/alignment comparisons compile
on the host and Arm target. No calibration or control threshold was retuned.

RTOS task priorities, periods, locks, coordinator ownership and safety paths are
retained. Both barcode decoders, 7 mm junction detection, 45/60/75 speed modes,
bend assistance, stationary recovery, right-wheel ramp processing and pickup
latching remain. Baseline 45 mm/s is still the default. Network supervision and
powerbank operation remain; capture scripts are optional.

## Install

1. Keep your working v0.2.6 UF2 available for rollback.
2. With motor power off, hold BOOTSEL while connecting USB, then copy
   `robopico_buddy1_4_rtos_v0_2_7_cleanup.uf2` to the Pico drive.
3. Disconnect the flashing cable and power the robot normally. Existing saved
   Wi-Fi and front calibration remain; do not recalibrate solely for this test.
4. Start your broker/dashboard as usual. The full bundle includes the existing
   setup, dashboard and optional logging helpers. The previous dashboard remains
   protocol-compatible; the bundled dashboard displays v0.2.7.
5. Confirm the firmware identifies v0.2.7, wait for fresh calibrated IMU status,
   choose Baseline 45 and apply/read back your barcode mapping after power-up.

Flashing the new UF2 is sufficient if the current dashboard/setup already work.
There is no USB serial/DTR requirement during normal powerbank driving.

## Short hardware smoke test before promotion

Use the same sensor placement, saved calibration and mats as the working build.

1. **Boot and commands:** motors remain off at boot; dashboard Start observes the
   existing countdown. On a short flat section, confirm straight tracking,
   Pause/Resume and Stop. GP21 Stop and GP20 ESTOP must still remove output.
2. **Direction and barcode:** use sufficient supported mat for a left turn,
   right turn and U-turn/reacquisition. Include a bend in each direction if
   available. The compact direction fields must preserve both signs.
3. **Ramp:** cross the same right-wheel-only hump once, then continue level for
   approximately two seconds before Stop. Check accepted count/height and ramp
   assistance. Repeat only if the result differs from v0.2.6.
4. **End run:** perform the previously successful lift/tilt gesture. Motors must
   stop and remain latched after replacing the robot. Explicit eligible Reset
   and a new Start are required; there must be no automatic restart.

Please report these four results. Keep the dashboard open from before Start to
after Stop. Use **Export run** and retain its downloaded files. If anything differs,
also save **Logs & exports → Export all frames CSV**, event CSV, Messages log and
the stopped Diagnostics/Navigation rows; Motion rows help with a driving issue.
A short video is useful for a failed turn, bend, ramp or pickup; not required for
an uneventful pass. No full five-minute network/reconnect repetition is required
for this cleanup alone.

## Software evidence and its limits

The formatting/naming/declaration stage rebuilt to the exact v0.2.6 UF2 SHA-256
`9e69291564ccccb3c70359cf331e5df3f884cce514b2972412105dccc9248208`.
Explicit-expression cleanup also reproduced its preceding-stage UF2 exactly.
These comparisons cover the release configuration, not every alternate profile.

Existing C regression programs cover the retained configurations. Focused new
checks exercise all compact 2/3/9/11/13-bit values, 31-bit arithmetic boundaries,
negative IMU projections/pitch, mirrored bend/search signs and seven native
layouts. Codec fixtures compare against the preceding working representation.
The release manifest records linked RAM, stack frames and artifact/source hashes.

Passing the mechanical checker and formatter is not certification of every
semantic Barr rule. Per-function documentation depth, all casts/promotions,
nesting/complexity and exhaustive whole-program concurrency analysis remain
human review responsibilities. The adopted standard and exceptions explicitly
separate these from the enforced subset. No percentage-compliance claim is made.
