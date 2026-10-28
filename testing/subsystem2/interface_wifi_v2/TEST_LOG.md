# Wi-Fi V2 validation — 2026-10-28

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
