# V1.1 startup fix — 2026-10-27

Original Interface V1 can print `FOLLOW accepted` while the coordinator immediately enters FAULT and holds both targets at zero. Start authorization and coordinator output permission are separate stages; that message alone did not prove motor commands were being forwarded.

## Confirmed cause

The hardware main captures `CircuitInput.now` before computing its optical demand and publishing wheel targets. Publication sets `nav_since` using the live microsecond port clock, so it is slightly newer than the input snapshot. Original V1 tested `feedback.now_us - nav_since > 100000` with unsigned 64-bit subtraction. A newer request by even one microsecond underflows to a huge value and wrongly raises `NAV_FAULT_CONTROL` (0x100), mode 6, targets 0/0. The session could remain active, making this particularly confusing.

The same mixed-clock subtraction could incorrectly time out a recent takeover request against an older feedback snapshot.

## Reproduction and fix

A new test was run against the unmodified V1 coordinator first. With a request one microsecond newer than the snapshot it failed with:

```text
REGRESSION: request newer than snapshot by1us causes mode=6 fault=0x100 zero targets=0,0
```

V1.1 evaluates request ages using a fresh reading of the same monotonic port clock used to timestamp those requests. Snapshot timestamps remain unchanged for sensor/encoder feedback and settling observations. The 100 ms stale-navigation and two-second failed-settling checks remain enabled; no threshold was extended or removed.

New regressions cover 1, 2, 7, 50, 500, 1000 and 5000 microseconds of snapshot/publication lag for 14,000 active ticks, the exact 100 ms heartbeat boundary, actual stale-request failure, takeover between snapshot and update, same-snapshot resume, and actual two-second failed settling. Packed and unpacked builds pass. The earlier V1 tests used identical fake timestamps and therefore missed this hardware ordering; their passing result did not establish physical startup correctness.

## Release and test

Use `firmware/buddy23_interface_v1_1.uf2`. Keep the existing calibration; verify `h` says **BUDDY23 INTERFACE v1.1** and `CAL FRONT ... valid=1`. Place the front sensors over the stripe, select profile `1`, and start normally. For a USB test, `g` starts after two seconds; `s` or either button stops.

`v` now also prints:

```text
NAV_COORD: mode=0 owner=3 session=1 ... fault=0x000 targets=...
```

Mode 0 is AUTO; mode 6 is FAULT. Nonzero targets/PWM without wheel movement point to a different motor-power/drive issue and require the live `BARCODE_CIRCUIT` and `NAV_COORD` lines. Stop and wait five seconds before changing power; `z` exposes coordinator fault evidence. The original V1 would show fault 0x100 for this bug.

This patch changes coordinator time comparisons and adds visibility. It does not change V12.2 optical steering, barcode decoding, motor control, PWM caps, profiles, junction geometry, calibration, API signatures or saved-record layouts. The CMake target remains `buddy23_interface_v1` for Buddy5 integration compatibility; the downloadable fixed artifact has the distinct V1.1 filename. If Buddy5 has already added code, update the coordinator source and rebuild that combined image rather than replacing it with the default no-ultrasonic image.

Host and ARM builds verify the regression fix, not physical motor supply or tracking on the user's robot. Test this release on the robot next. Original V1's artifact/build report remain archived for comparison; do not use its UF2 for new tests.
