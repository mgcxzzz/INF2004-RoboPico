# V2.1 follow-up

Based on V2 commit `c8155de`. User-supplied V1.3 diagnostic output identifies a MEMP_SYS_TIMEOUT-empty panic. V2.1 changes `lwipopts.h` to reserve four application timeout slots, plus the firmware banner and test/release material. Current source ledger and verifier preserve the original V1 baseline at `0285d74`, allowing the exact timeout option insertion and banner substitution. No motion code or settings schema changes.

# V2 update provenance

V2 starts from all tracked V1 source files at commit `0285d74` (excluding published V1 firmware). `WIFI_V2_BASELINE.json` records their hashes; `SOURCE_SHA256.json` records the current package. V1 remains untouched in its original folder. V2 changes networking recovery/diagnostics, PC helper diagnostics, tests, release metadata/docs and the firmware banner; the motor/navigation controller is preserved. No hardware validation is claimed.

## Historical V1 provenance

# Provenance and integration boundaries

Continuous V2 base: `mgcxzzz/INF2004-RoboPico`, branch `codex/buddy2-left-motor-test`, commit `1263390b069ae9617c412e47212177e77807c505`.

Buddy1 input: same repository, branch `buddy1`, commit `b09403074939f45cf4d7c11b7606148f6c2a4e73` (27 October 2026, “Implement Buddy1 MQTT dashboard and record bench tests”). Reviewed its mapping/protocol/network APIs, task, team telemetry schema, MQTT service, dashboard and test evidence. The compact protocol/mapping algorithm/browser foundation are adapted with attribution. Its RTOS scheduler, coordinator and template applications are not merged into this standalone SDK harness.

Buddy1's evidence supports networking/mapping bench work, not physical motor timing or course completion. This artifact has separate software evidence and needs its own hardware test. Historical V12.2/V1 manifests copied with source describe earlier components; `WIFI_BASELINE.json`, `WIFI_PATCHES.json`, `BUILD_METRICS.json`, `BUILD.md`, and `verify_firmware.py` describe this build.

V2 motor, steering and optical gains/thresholds are byte-checked. Routing replaces fixed A/F/Z selection with a captured action/revision, extends the canonical route index to5bits, and retains existing L/R/U geometry. Storage shortens history and uses record26/RAW2 formats, retaining CRC/stopped saves and importing old front calibration only.

No Buddy4 or Buddy5 policy is added. Default `buddy_hooks.c` remains empty; inherited optional examples are not linked. No credentials are in tracked source/UF2.
