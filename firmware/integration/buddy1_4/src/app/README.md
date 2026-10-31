# Application lifecycle and scheduling

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

`main.c` is the only application entry point; it no longer includes another main.c through version-numbered folders. It owns initialization, USB/button commands, run guards, the regular ADC/control schedule, stopped saves and ordered feature polling.

Core0 owns sensing/navigation/motion. Core1 owns the copied network transport. Application-private adapters connect the retained state machines to their explicit public services and the one PWM commit path. They are internal glue headers, not buddy-facing APIs; their filenames state their feature rather than an inherited firmware version. Pure numeric trackers/controllers remain header functions to preserve their existing host-testable implementation; separately compiled navigation, transport, IMU and motion-executor modules remain distinct translation units.

Run order matters: sample the barcode readers, compute optical navigation, submit Buddy3 targets, update coordinator feedback, choose motion controller, request/apply approved ramp assistance, compute drive output, generation-check the PWM commit, then poll bounded external/network/IMU work. IMU polling prefers a gap just after the barcode sample. Do not add blocking display or I/O work to the moving path.

Start validates calibration, settled/rearm state and released buttons, then performs the existing countdown. Stop/ESTOP invokes immediate outputs-off and invalidates old command generations. Stop and save are different phases; wait5s before removing power or requesting long stored dumps. Resume resets the original and peak partial readers.

The startup banner and dashboard explicitly identify this as PROVISIONAL Buddy1–4 integrationv0.1. Buddy5's default extension hooks are empty. The structural/API changes need hardware acceptance despite the old combined baseline's successful recorded runs.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
