# Current WiFi V1 integration note

This folder is based on Continuous V2 with a Buddy1 network adapter. Default Buddy5 hooks remain empty; optional examples below are not linked. Existing takeover/finite move/resume APIs and sign conventions are retained. All normal movement still passes through the same coordinator and motor writer. Barcode routes now accept A-Z and resolve a captured runtime mapping; do not assume A/F/Z defaults. Network Pause is a Buddy1 lease, and Buddy5 retains its existing preemption priority. Wi-Fi callbacks only queue data and never invoke motor APIs. Core1 is reserved for the network owner, so extension hooks stay cooperative on core0. Every stopped flash operation uses the core1 lockout wrapper.

The historical interface handoff follows for the unchanged APIs. Use this folder's README/BUILD for its current version, recording depth and setup.

---

# Current target: Interface Continuous v2

Default Buddy5 hooks remain empty. Use this folder to build the retained APIs/harness with the V2 steering test; README.md lists the current changes. The following V1 guide is inherited integration reference.

# Current test target: INTERFACE CONTINUOUS v1

Build from this folder to include the continuous ordinary driving test. Use the same APIs, startup and hook instructions below. The inherited V12.2 behavioural claims describe the baseline; README.md in this folder lists the new ordinary-drive changes. Profile1 is the first physical test.

# Buddy5 with V1.1 HANDOVER1

This folder preserves the InterfaceV1.1 nav API, standalone circuit harness and startup fix. It retains saved diagnostics and adds the narrowly guarded ordinary cruise-exit compensation transfer described in README.md. Use [README.md](README.md) for the UF2/start/save/dump procedure. `p` is normal motion, `d` detailed drive state, `z` coordinator requests/results. Dumps occur only after stopping. Diagnostic bank0x1ab000..0x1af000 must be reserved alongside the existing log banks.

Build with this folder's CMake project and optionally set `BUDDY_EXTENSION_SOURCE` to the absolute path of Buddy5's source file. Strong `buddy_extension_init`/`buddy_extension_poll` hooks replace the weak no-op hooks; see examples/buddy5_extension.c. All buddies use nav_api.h; no direct motor GPIO/PWM writes. Buddy5 retains ownership throughout avoidance and line search, and explicitly resumes after measured stop and front line confirmation. API fields/signatures/ownership rules are V1.1. `nav_report_terrain` is telemetry only; speed assistance remains UNSUPPORTED. The default UF2 contains no ultrasonic driver.

Core0 cooperative scheduling and existing synchronized commit/stop behavior are retained. The additional diagnostic snapshots run on core0, outside motor commit IRQ masking. Any RTOS/multicore integration still needs flash lockout and timing validation. Obtain `p` and `d` before changing calibration/profile or beginning another run. Keep DIAG1 at949d4b7 and its original UF2 as the direct comparison baseline; V1.2/V1.3 physical drive regressions are not active here.
