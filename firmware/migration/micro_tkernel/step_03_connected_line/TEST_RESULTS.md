# Step3B v0.1 software verification — 2026-11-04

Hardware acceptance is **pending**. No car was driven by these checks.

| Check | Result |
|---|---|
| Strict host C regression programs |20passed, with warnings as errors. Real shared coordinator/API/executor and production bridge/controller dispatch are used with deterministic kernel/hardware stubs. |
| Python capture/export/provisioning checks |49passed. New capture cannot automatically certify a physical driving test. |
| Dashboard protocol/compatibility tests |27passed, including original SDK/RTOS layouts and optional source15/ramp payloads. |
| Dashboard headless Chrome checks |48passed with simulated MQTT. Includes connected RTOS identification and Ready/Paused/E-stop display, preserved SDK/RTOS behaviour, exports and mobile overflow checks. |
| Arm builds |Step2 `rtos_line`, Step3A `rtos_network`, Step3B `rtos_line_wifi` all compile/link sequentially. Existing published Step2/3A binaries are not overwritten. |
| Linked RAM |Static end0x2002d2e8;77080bytes remain before dynamic kernel/task allocation. Largest individual application stack frame600bytes; these are static frames, not measured call-chain stack high-water marks. |
| Flash transaction routines |RAM0x20000270/0x200002a8; no direct flash-fetch calls in copied RAM code. |
| Reference source audit |Packaging checks all shared source/config/interface files against the SDK baseline; only the two existing opt-in junction departure edits are allowed. |

The connected bridge test covers command age/connection epochs/barriers,
wraparound, queue-full Stop delivery, malformed-prefix rejection, priority TX
and stale telemetry expiry, early reconnect Start rejection, actual coordinator
Pause/stillness/Resume handback, and delayed Reset following physical E-stop.
Existing tests cover interrupted junction takeover, stale commit rejection,
ownership/leases, ADC recovery and continuous steering/drive parity.

The package verifies UF2 family/flash bounds, required network/control symbols,
absence of barcode/IMU/Buddy5 startup, source hashes and archive integrity. See
the generated BUILD_MANIFEST.json and SHA256SUMS.txt for exact release details.

Still required on the car:100Hz timing/clock agreement and received heartbeat
gaps while moving, Start/Pause/Resume/Stop/E-stop, USB loss, broker/AP loss and
reconnect without movement, plus Step2 L/R and sharp bends with adequate mat.
Radio power-save is disabled and read back by the kernel owner for Step3B;
that setting alone is not proof that prior recovery jitter is resolved.
