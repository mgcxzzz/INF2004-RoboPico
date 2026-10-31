# micro T-Kernel migration — experimental checkpoints

The course project requires C, the Pico C SDK **and micro T-Kernel RTOS**.
The working Buddy1–4 SDK-only integration on main at `7843f45fbe45d5a774bc18c93be9bb11084d1b64`
is the reference for driving behaviour; it is not the completed RTOS migration.
Development is isolated on `codex/micro-tkernel-migration`. Buddy5 is not integrated.

| Step | Port and hardware check before progressing | Status |
|---|---|---|
| 0 | Establish and measure hardware TIMER, kernel clock and actual task cadence against laptop wall time. | Included in Step1; hardware acceptance pending. |
| 1 | 10ms motion task, motor/encoder port, navigation API, safety stop, GP20/21 and USB console. Motors off at boot; verify each wheel, encoder sign, stops and automatic timeout. | [Current test package](step_01_motor_bringup/README.md); hardware acceptance pending. |
| 2 | Continuous V2 steering/drive, front IR calibration storage, line recovery and junction turns. Compare a straight and bend with the SDK reference at profile1 speed. | Not ported. |
| 3 | Kernel-owned Wi-Fi/lwIP/MQTT, bounded TX/RX message buffers and dashboard. Verify heartbeat and supervisory controls during driving, then broker/AP reconnect. | Not ported. |
| 4 | Original plus peak barcode decoders, distance/sampling schedule, runtime mappings and separate-junction action. Test mapped letters in both directions. | Not ported. |
| 5 | Buddy4 IMU detector6 and coordinated ramp assist. Test hump acceptance and crossing against the reference. | Not ported. |
| 6 | Buddy5 obstacle API integration with its owner; bench test then slow bypass/reacquisition. | Placeholder; teammate integration later. |

Keep gains, decoder logic and calibration values fixed while changing execution
contexts. Re-run motion timing with each added task. A checkpoint passing host
tests or compiling is not a claim that its hardware test has passed.

The kernel Wi-Fi service remains the only permitted owner of lwIP/MQTT in Step3,
as specified in TEAM.md and the professor's integration guidance.
