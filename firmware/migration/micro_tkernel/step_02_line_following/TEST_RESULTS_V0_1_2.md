# Step 2 v0.1.2 — software checks and hardware test status

Date: 2026-11-03. Branch: `codex/micro-tkernel-migration`.
**Experimental junction policy; hardware acceptance pending.**

## Evidence motivating the change

The user's v0.1.1 stationary capture (`2026-11-03_22-48-05`) passed approximately
70seconds of clock/acquisition observation: control99.991Hz, no new intervals
above15ms, zero genuine conversion errors/timeouts. The calibration stayed
white122/138, black246/267. White poses read approximately143/177 initially and
141/175 at the end; isolated black read approximately1500–1800. These are actual
IR measurements, not replacement calibration values.

For that saved calibration, the existing normalized <200 rule needs filtered
left <=146 and right <=163. The measured right white surface fails it. Prior
right/U-turn captures ended in J_BRANCH_MISSING; the left test was manually
stopped early. This identifies a failed prerequisite, not proof that removing
it will produce a correct physical branch acquisition.

## Passed offline checks

- 15 strict production-C regression programs, including both angle-only and
 original white policies. Test both >10deg/30ms departure and distinct30ms branch
 dwell, qualification reset,75..110deg L/R branch window,165deg U-turn window,
195deg U-turn failure limit, turn timeout, fault outputs and L/R mirroring.
- Ordinary tracker, Continuous V2 steering, calibration import/CRC/storage,
 power envelopes and post-turn acquisition parity tested with both policy
 settings. Shared coordinator/stop/stale-command/commit and ADC tests pass.
- Junction acceptance is logged at L/R acquisition entry and U-turn stage2 ->3.
 Tests cover no-white acceptance, qualified-white acceptance, event pulse versus
 latched flags, per-turn reset and angle/steering-field packing.
- 38 Python tests pass:15 timing/readiness,13 old/new stopped-log export and10
 ADC-readiness analyzers. New exporter preserves version1 log interpretation
 and requires policy metadata for version2.
- Step1 ARM build with original/default policy and final Step2 ARM build pass.
 One expected linker RWX-segment warning reflects existing RAM-resident flash
 code. Application C compilation uses Wall/Wextra/Werror.
- Shared baseline audit verifies101 files byte-for-byte against main7843f45;
 exactly2 turn headers are permitted to differ, and their entire content must
 match the mechanically reconstructed opt-in-only change. Other drift fails
 packaging. Default policy remains0.

See `logs/v0_1_2/host_tests.txt`, `step1_build.txt` and `arm_build.txt` for raw
software results. `firmware/BUILD_MANIFEST.json` contains final source/image
hashes, UF2 bounds, linked symbols, RAM flash-code and static stack-frame checks.

The RAM row remains56bytes x1536. Six new diagnostic bits and a signed13-bit
angle use spare aux bits; no per-row RAM increase. This is a local representation
audit, not certification of minimum-bit compliance for all later subsystems.

## Required car checks

Follow [the exact test guide](UPDATE_V0_1_2.md): unchanged calibration/mounting,
short straight, then independent left/right/U-turn attempts with enough mat,
stopped logs and video. A complete export or host test is not a driving pass.
Sharp bends remain unresolved. Wi-Fi/MQTT, physical barcode decoding, IMU/ramp
and Buddy5 are not active in this checkpoint.
