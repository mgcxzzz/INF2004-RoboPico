# Step1 software verification

Updated:2026-11-02 for v0.1.1. Reference main:`7843f45fbe45d5a774bc18c93be9bb11084d1b64`.

**v0.1.1 user hardware acceptance passed on 2026-11-02.** See
[acceptance measurements and scope](HARDWARE_ACCEPTANCE.md). The checks below
record the preceding software/build verification; the acceptance note records
the subsequent cold-start, both-wheel, encoder and stop evidence.

- Production policy regression: independently rejects the previous~42.7x slow
  hardware TIMER and~1.525x clock disagreement, frozen clocks, incomplete startup,
  unreleased buttons, USB disconnect, stop latches and stale input; exercises
  wraparound and the same encoder transition table used by the real port.
  Added production-policy cases distinguish idle revalidation from active-run
  latching at the50ms threshold, including an existing emergency latch.
- Existing ContinuousDrive tests pass: feedforward/PI, bounded startup aid,
  physical PWM envelope and slew. No algorithm source was changed.
- Existing shared-interface tests pass: ownership, stop invalidation of stale
  commits, leases, compact request roundtrips and audit.
- Existing clock-ordering tests pass: newer requests cannot underflow age;
  stale100ms targets and2s settling timeout remain enforced.
- Existing coordinated-assist tests pass. These establish reuse of the API;
  the IMU/ramp application is not running in Step1.
- Fifteen Python analyzer tests pass: normal timing, both clocks slow together,
  timer/kernel disagreement, wrap, missing/short captures, new late intervals,
  dropped USB output, internal validation failure, active motor capture and
  duplicate status queries.
  Added cases cover pre-existing latches despite stable timing, retained old
  delay counters without a latch, missing/invalid/stale readiness, buttons,
  IRQ latching and USB DTR. Replaying the original user capture now yields
  timing_result PASS and overall BLOCKED with the exact late-loop fault.
- Full Arm GCC image builds; new application files use `-Wall -Wextra -Werror`.
  The linker enforces a1MiB code ceiling, below the reference's saved sectors.
- Packaging checks UF2 family/block indices/flash bounds, required/forbidden
  ELF symbols, source hashes, ZIP integrity, and byte parity of the preserved
  shared integration source/config/interface files against main.

See `logs/host_tests_v0_1_1.txt`, `logs/arm_build_v0_1_1.txt`,
`logs/replay_original_capture_v0_1_1.json` and `firmware/BUILD_MANIFEST.json`.
The original v0.1 build/test logs remain alongside them.
The v0.1.1 cold-power-up capture and pulse/GP20 retest requested in UPDATE_V0_1_1.md
were subsequently accepted. No shared main controller/config/interface source changed.
