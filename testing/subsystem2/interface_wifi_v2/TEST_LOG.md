# Wi-Fi V2 / V2.1 / V2.2 validation — 2026-10-28

Baseline branch: `codex/buddy2-left-motor-test`, pulled at `0285d74`. V1 preserved. All changes and evidence for this request are contained in `testing/subsystem2/interface_wifi_v2`.

| Check | Result | Evidence / scope |
| --- | --- | --- |
| Full host suite | PASS | `logs/host_tests.txt`: 34 C test executables, 10 JavaScript schema tests, 17 Python helper tests; frozen algorithm comparison passed. |
| Actual transport worker with SDK/lwIP fakes | PASS | Included in the full suite: asynchronous API versus callback, accepted CONNECT/SUBACK, invalid address/allocation failure, missing SUBACK retry, TCP/CONNACK timeout distinction and sleeping initialization failure. These are mocked boundaries, not a radio test. |
| Final PC helper tests | PASS | `logs/python_helpers_final.txt`: 20 tests, including full CONNECT/SUBACK/fresh-heartbeat handling, broker refusal and TCP failure. |
| Host fixture correction | Resolved | `logs/host_tests_initial_failure.txt` records a compile failure from a harness clock dependency; moved elapsed-time calculation to the transport snapshot. Full suite then passed. |
| ARM network ON, Release, warnings as errors | PASS | `logs/arm_wifi_on_build.txt` and configuration log. No local credential header. |
| ARM network OFF, Release, warnings as errors | PASS | `logs/arm_wifi_off_build.txt` and configuration log. |
| V1 unchanged / controller preservation | PASS | Current release verifier compares preserved source hashes, allowing only the firmware banner in the nested baseline main. |
| UF2/source/package verification | PASS | `logs/release_verification.txt`; current hashes in `firmware/SHA256SUMS.txt`. |
| Teammate hotspot, TCP 1883, real heartbeat | NOT TESTED | Screenshot establishes Wi-Fi IP-ready only; run the README checks with V2 and the actual laptop hotspot IP. |
| Real motor/barcode/navigation regression | NOT TESTED | Controller code preserved, but no hardware test or flash performed here. |

The PC helper unit tests use socket/process mocks. Their printed broker-start messages are simulated; they do not mean a live broker or Pico was tested. No browser screenshot was taken. Existing V1 build metrics, baselines and protocol attribution documents retained in this folder are historical evidence, not additional V2 test results.

## Hardware result to save later

Save the V2 banner, two `n` samples a few seconds apart (`WIFI_LINK`, `WIFI_PROGRESS`, `WIFI_MQTT`), PC diagnostic outcome and fresh dashboard heartbeat evidence under `logs/`. Record laptop/Pico IPs and whether TCP 1883 was permitted, but omit SSID/password. Record failed attempts too. Do not mark the physical test passed until `mqtt=1 subscribed=1` and fresh heartbeat are observed.

## V2.1 follow-up: confirmed timeout-pool panic

User hardware evidence is saved in `logs/hotspot_stall_2026-10-28.txt` (V2 stall) and `logs/timeout_pool_panic_2026-10-28.txt` (V1.3 diagnostic panic). The latter explicitly identifies an exhausted MEMP_SYS_TIMEOUT pool. The original copied lwIP options left no slots beyond lwIP core timers; MQTT's TCP-connect callback adds an application timer. The networking failure is therefore no longer only an unlocalized stall. A larger diagnostic stack did not resolve it.

V2.1 changes only the timeout-pool option plus banner/release material/tests. The original V1 folder remains unchanged by this fix. V2 networking recovery and packet/controller implementations are retained.

- `logs/lwip_timeout_pool_regression.txt`: real SDK `memp.c`/`timeouts.c`, production lwIP feature options, native platform/clock stubs and no-op unrelated protocol callbacks. The core-only default reproduces the exact allocation assertion. The fixed 11-slot pool supports all 7 core timers plus the application timer, 100 callback reschedules and 1000 add/remove cycles. These are allocator tests, not MQTT/radio tests. Two benign native-linker section-alignment warnings are retained in the raw output; ARM builds use their own target layout.
- `logs/host_tests_v2_1.txt`: full host suite passes (34 C test executables, 10 JavaScript tests, 20 Python tests). The real-lwIP regression above is a separate pair of executables.
- `logs/arm_wifi_on_build_v2_1.txt` and `logs/arm_wifi_off_build_v2_1.txt`: ARM Release builds pass.
- `logs/release_verification_v2_1.txt`: updated source/UF2/bundle and precise timeout-option-only provenance check pass.
- Physical V2.1 retest: **PENDING**, do not mark overall hardware connectivity passed from host/build results.

## V2.2 follow-up: driving stack collision

- Saved actual telemetry-off Start/Running/LWT log: `logs/start_heartbeat_loss_2026-10-28.txt`. No USB+power-bank test was performed or requested for this retest.
- Old application compiler frames and actual ELF direct calls prove a driving path consuming at least 4328 bytes, intruding into the core1 stack under the default layout: `logs/arm_stack_usage_v2_1.txt`, `logs/stack_layout_baseline_v2_1.txt`. This is a proven layout defect; the physical corruption event itself was not captured.
- V2.2 links separate 8192-byte stacks with heap limited below both. `logs/stack_layout_v2_2.txt` validates the real linked layout and call-frame budget; `logs/arm_stack_usage_v2_2.txt` records current frame sizes.
- ARM Release network ON/OFF builds pass: `logs/arm_wifi_on_build_v2_2.txt`, `logs/arm_wifi_off_build_v2_2.txt`.
- Firmware/source/package checks pass: `logs/release_verification_v2_2.txt`, including boot SP and current stack/heap bounds.
- V2.1 host and real-lwIP pool tests above remain the latest algorithm/schema regression results. They were not rerun for this linker-only change plus banner, documentation and new static verification script.
- Physical Start / driving / heartbeat retest of V2.2: **PENDING**. No claim of complete worst-case stack usage or resolved hardware dropout until retested.
