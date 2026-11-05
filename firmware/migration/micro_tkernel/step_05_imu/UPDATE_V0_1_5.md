# Step5 v0.1.5 — bounded odometry-gap qualification recovery

Experimental micro T-Kernel SMP Buddy1–4 migration build. Buddy5 remains a
blank integration point; main is unchanged. Parent v0.1.4 commit:
241546c2be22aaf8e009fef7da4d6344db92ced0.

## Evidence and scope

v0.1.4 flat/ramp motion retained about50Hz IMU and100Hz control with no new
motion-window acquisition, late-gap or FIFO-overrun faults. Assistance ran
1959ms and ended on descent. Live height reached9mm in sampled telemetry,
but accepted humps/max stayed0. Level-ready was absent during the climb;
right invalid encoder transitions rose19->22->31. Decimated/aggregated
telemetry cannot prove the exact failed20ms sample. Dirty odometry is a
supported hypothesis, not a proved electrical encoder fault.

The current change allows a limited pause in qualification rather than
blindly treating uncertain odometry as valid. It does not force9/10mm into
the maximum or claim that the physical crossing already met acceptance.

## Policy

Only APP_RTOS_IMU builds opt in. Default SDK preprocessing remains unchanged.
After valid level readiness has been established, isolated odometry-quality
loss may hold readiness/candidate state for up to100ms. No uncertain sample
contributes to height peak, rise/return confirmation or trusted travel.
New candidates require current trustworthy sensor/odometry data; an unready
robot cannot become ready by waiting through invalid samples.

The new odometry tolerance shares the existing consecutive-uncertainty limit
with dispersion/acceleration skips, so alternating categories cannot reset
that limit. Candidate skipped time shares the existing25% total exclusion
budget. Existing rise/return/travel/peak/time guards remain. Known turn,
reverse, shock, sensor/timing faults, prolonged uncertainty and Stop/Reset
still cancel. Recovery rebases distance/rate bookkeeping without crediting
outage travel. Gross movement across a gap remains a rejection check.
After recovery, a bounded heading window preserves the existing200ms turn-rate
meaning while velocity history re-primes. It allows no unchecked turn period:
large yaw still fails during warm-up, and repeated gaps cannot reset the yaw
anchor to evade the check. The full8s profile timeout still applies during holds
and before a recovered sample can complete a candidate.

IMU still owns I2C/estimator work on core0 priority2. Motor/navigation/safety
remain on core1. The qualification path is bounded pure computation; it adds
no task sleep, I/O, PWM write or network call. Requests still pass through
the shared navigation coordinator. IR calibration, Continuous V2 steering,
barcode/junction logic and ramp-assist policy/limits are unchanged.

## Diagnostics and USB output

IMU_HUMP is a copied USB snapshot, not a new binary MQTT packet:

- flags bits1/2/4/8:level-ready/candidate/confirmed-rise/returning.
- holds: uncertainty episodes that pause established readiness/candidate state.
  recoveries: fresh odometry returns after eligible gaps, including gaps while
  not yet ready. disarms: cleared readiness/candidates/partial level proofs;
  early_clears: candidates cleared before confirmed rise. These are uint32
  modulo counters reset by physical IMU initialization; compare run deltas.
- hold_reason:0 none,1 odometry,2 dispersion/acceleration,3 mixed.
- clear_reason:0 none,1 odometry gap too long,2 candidate exclusion share,
  3 turn,4 reverse,5 shock,6 Stop/Reset,7 height fell before rise confirmation,
  8 other invalid quality,9 profile timeout,10 mixed uncertainty too long.
- failure_reason: latest non-Stop/Reset clear reason using the same map; retained
  across Stop/new runs so a normal Stop cannot hide a short failure.
- gap_ms: current held odometry-episode duration; last_gap_ms: latched last
  episode duration. Both are capped127ms; reason counters retain expiry.

Latched reasons/counters make short holds and unconfirmed candidate clears
visible without flooding USB at50Hz. Captures retain first/last snapshots
and changes. Detailed raw USB remains authoritative; the summary does not
claim to reconstruct every20ms sample.

Periodic low-priority status output yields10ms between groups to let the
existing4096-byte USB ring drain. Manual reports remain unpaced for command
response. Producers remain nonblocking and whole-record drops remain counted.
This does not promise loss-free output when the host stops reading or a user
floods manual reports. No ring size/IRQ masking or task priority is increased.

## Representation and compatibility

New context hold/clear/failure reasons use2/4/4bits, two gap/heading flags1bit each, latched
millisecond duration7bits(capped127), bounded gap accumulation17bits(capped
131071). Four diagnostic counters use their full32-bit modulo domains.
The heading anchor uses a full signed16 wrapped heading and uint32 microsecond
modulo timestamp. The cross-task snapshot packs flags4, reasons2/4/4 and
durations7/7bits.

Only the IMU owner changes context/diagnostics. It publishes a snapshot under
the existing short copy lock. Readers copy before formatting. Packed fields
are not concurrently modified by other tasks/ISRs. No sensor/network I/O or
formatting is performed under the copy lock. Binary packet layouts and sizes
are unchanged. Static RAM comparison is recorded in BUILD_MANIFEST; this is
not a claim of complete whole-firmware Tribunal compliance.

## Validation and physical test

Use START_HERE.md: one flat negative control must stay at0 humps, followed
by one complete10mm one-sided ramp crossing with qualified level approach
and return. Expect one accepted hump/nonzero maximum, correct assist release,
healthy50/100Hz cadence and no new diagnostic drops in the measured windows.
Stop safety remains dominant. Hardware behavior is unverified until tested.

Selected software checks cover actual production qualification code, service
integration/Reset, ramp coordinator safety, nonblocking USB records, capture/
exports, and ARM build/source/flash/RAM/stack/bundle integrity. Unchanged
barcode/steering suites are not broadly rerun for this checkpoint.

Six selected strict C programs and25 Python capture/export tests passed. The
production qualification fixture includes a full accepted10mm profile with
short gaps before ascent, during elevation and during return; excluded70mm
samples do not inflate its maximum. It also covers zero-hump flat travel,
long/mixed uncertainty,25% rejection,8s timeout, unconfirmed early clears,
cross-gap reverse/turn checks, bounded heading recovery, Stop and an actual
processor shock cancelling a held candidate. The real service/reset fixtures
also passed. These are software evidence, not a substitute for the ramp run.

ARM build and source/flash/RAM/stack gates passed. Default SDK preprocessed
tokens match main for imu_proc.c/.h, imu_service.c, b4_config.h and the driver.
Maximum checked application/shared-IMU/libtm stack frame is944bytes; these
static frames do not establish a runtime call-chain high-water mark.

| Same ARM build profile | v0.1.4 | v0.1.5 | Change |
| --- | ---: | ---: | ---: |
| Static BSS bytes |189576|189632|+56|
| Linker static RAM end |0x2002e6c8|0x2002e700|+56 bytes|
| RAM before dynamic kernel allocations |71992|71936|-56 bytes|
| Existing binary MQTT packet growth |baseline|unchanged|0 bytes|
