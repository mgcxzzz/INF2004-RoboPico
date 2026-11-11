# Software evidence: provisional Buddy 1–4 RTOS v0.2.3

No robot was connected during this work. Physical lift/overturn discrimination,
motor/wheel stop latency and no-false-positive ramp acceptance remain pending.

## Purposeful checks performed

Seven native C programs passed:

- Detector:30ms upward and50ms steep-tilt boundaries, single-spike/ramp exclusions,
  sticky reason/counter,500ms upright eligibility, invalid batch dwell reset,
  bounded integer arithmetic and packed metric saturation.
- C9 codec:actual C-generated30-byte binary consumed by the browser tests.
- Supervisor policy:IMU latch/inhibition, Start/Resume/Reset/output gates.
- Actual production IMU service/task:boot and Reset calibration, live FIFO hook
  independent of accepted height, no production Impact state, latched lift,
  raw-reset bypass refusal, fresh/upright revalidation after a gap, and single-queue
  reset handoff that remains output-inhibited through calibration.
- Existing release bridge and exports:coordinator Stop/Pause/Resume compatibility,
  cancellation/queue handling and stopped row/export identity.
- Retained legacy IMU service/ramp:calibration/mission/fault reset and synthetic
  nominal10mm ramp acceptance, with default legacy collision behavior preserved.

Four Node tests passed for C9 decoding, reserved/range/version/truncation rejection,
and the eighth C3 supervisor bit. The actual C-to-browser release/runtime test
passed. A headless browser using simulated MQTT passed lift/overturn/unready/stale
labels, release metadata and stopped binary/CSV/JSON exports with no page errors.
No live broker or robot commands were sent by these browser checks.

The final ARM release build passed: text498920,data0,BSS195432 bytes. The inherited
RWX-segment linker warning remains. Static RAM end0x2002fda8 is24 bytes above
v0.2.2;66136 bytes remain before dynamic kernel/task allocation. This is not a
measured stack high-water margin or end-to-end real-time latency measurement.

Packaging verifies flash limits, RAM flash routines, required RTOS/safety symbols,
stack-frame limits, exact protected source fingerprints and ZIP hashes.89 protected
executable files remain unchanged from the Step5 comparison; nine existing files
have explicitly reviewed release fingerprints, including retained v0.2.2 changes.
The existing decoder/calibration/ramp-control/network limits are preserved.

C9 adds30 binary frame bytes/s at1Hz and remains enabled with ordinary telemetry
off. New bounded application fields use bitfields and explicit masks; task-shared
packed data use short locks/copied requests. The scoped representation/RAM evidence
does not claim completion of the whole-firmware Tribunal audit.

See [the setup/test guide](imu_end_run_v0_2_3.md). The v0.2.2 source and artifacts
remain available for rollback. Buddy5 is not integrated.
