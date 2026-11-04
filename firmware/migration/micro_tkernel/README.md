# micro T-Kernel migration — experimental checkpoints

The course project requires C, the Pico C SDK **and micro T-Kernel RTOS**.
The working Buddy1–4 SDK-only integration on main at `7843f45fbe45d5a774bc18c93be9bb11084d1b64`
is the reference for driving behaviour; it is not the completed RTOS migration.
Development is isolated on `codex/micro-tkernel-migration`. Buddy5 is not integrated.

| Step | Port and hardware check before progressing | Status |
|---|---|---|
| 0 | Establish and measure hardware TIMER, kernel clock and actual task cadence against laptop wall time. | v0.1.1 user capture PASS: 99.978Hz, both clocks match laptop time; READY. |
| 1 | 10ms motion task, motor/encoder port, navigation API, safety stop, GP20/21 and USB console. Motors off at boot; verify each wheel, encoder sign, stops and automatic timeout. | [v0.1.1 user hardware acceptance passed](step_01_motor_bringup/HARDWARE_ACCEPTANCE.md). |
| 2 | Continuous V2 steering/drive, front IR calibration storage, line recovery and junction turns. Compare a straight and bend with the SDK reference at profile1 speed. | v0.1.2 straight and one U-turn worked. L/R inconclusive: sensors left available mat. Extended-mat L/R and sharp bends pending; steering/calibration frozen. |
| 3 | Kernel-owned Wi-Fi/lwIP/MQTT, bounded TX/RX message buffers and dashboard. Verify heartbeat and supervisory controls during driving, then broker/AP reconnect. | [Step3A steady baseline and functional broker reconnect passed](step_03_wifi_mqtt/HARDWARE_ACCEPTANCE.md). [Step3B connected line v0.1.2](step_03_connected_line/README.md) adds USB-supervised motion and dashboard controls; physical acceptance pending. |
| 4 | Original plus peak barcode decoders, distance/sampling schedule, runtime mappings and separate-junction action. Test mapped letters in both directions. | [Step4 v0.1](step_04_barcode/README.md) builds both reference decoders, runtime mapping and stopped raw/event export; physical acceptance pending. |
| 5 | Buddy4 IMU detector6 and coordinated ramp assist. Test hump acceptance and crossing against the reference. | Not ported. |
| 6 | Buddy5 obstacle API integration with its owner; bench test then slow bypass/reacquisition. | Placeholder; teammate integration later. |

Keep gains, decoder logic and calibration values fixed while changing execution
contexts. Re-run motion timing with each added task. A checkpoint passing host
tests or compiling is not a claim that its hardware test has passed.

The kernel Wi-Fi service remains the only permitted owner of lwIP/MQTT in Step3,
as specified in TEAM.md and the professor's integration guidance.
