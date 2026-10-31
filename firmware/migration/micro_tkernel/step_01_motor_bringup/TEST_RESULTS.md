# Step1 software verification

Date:2026-10-31. Reference main:`7843f45fbe45d5a774bc18c93be9bb11084d1b64`.

**Hardware acceptance has not been performed.** All results here are offline
software/build evidence, not a claim of successful robot motion or stop latency.

- Production policy regression: independently rejects the previous~42.7x slow
  hardware TIMER and~1.525x clock disagreement, frozen clocks, incomplete startup,
  unreleased buttons, USB disconnect, stop latches and stale input; exercises
  wraparound and the same encoder transition table used by the real port.
- Existing ContinuousDrive tests pass: feedforward/PI, bounded startup aid,
  physical PWM envelope and slew. No algorithm source was changed.
- Existing shared-interface tests pass: ownership, stop invalidation of stale
  commits, leases, compact request roundtrips and audit.
- Existing clock-ordering tests pass: newer requests cannot underflow age;
  stale100ms targets and2s settling timeout remain enforced.
- Existing coordinated-assist tests pass. These establish reuse of the API;
  the IMU/ramp application is not running in Step1.
- Ten Python analyzer tests pass: normal timing, both clocks slow together,
  timer/kernel disagreement, wrap, missing/short captures, new late intervals,
  dropped USB output, internal validation failure, active motor capture and
  duplicate status queries.
- Full Arm GCC image builds; new application files use `-Wall -Wextra -Werror`.
  The linker enforces a1MiB code ceiling, below the reference's saved sectors.
- Packaging checks UF2 family/block indices/flash bounds, required/forbidden
  ELF symbols, source hashes, ZIP integrity, and byte parity of the preserved
  shared integration source/config/interface files against main.

See `logs/host_tests.txt`, `logs/arm_build.txt` and `firmware/BUILD_MANIFEST.json`.
The next evidence must be the user's read-only40s USB timing capture and
lifted-wheel encoder/stop tests described in the checkpoint README.
