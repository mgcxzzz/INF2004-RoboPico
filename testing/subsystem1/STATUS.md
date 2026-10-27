# Buddy 1 bench status — 27 October 2026

Branch: `buddy1`. Changes and evidence are uncommitted; no push performed.
Latest stable laptop image: `Embedded/bench_tools/buddy1/buddy1_mqtt.uf2`.
SHA256: `d27c36e9751898f870a9cdaec482ce76da97e49094209c0d89bc0497df3714ce` (user-reported flash, no device checksum readback).

## Verified this session

- Commands/state transitions, rejection handling, remote ESTOP/reset and board buttons (routing/state only).
- Ping latency; telemetry off/on; mapping SET/GET, idempotence, revision changes, RESET retention and power-cycle clearing.
- Live malformed/oversized mapping rejection and mapping SET/GET while telemetry off.
- Broker loss/recovery and fresh cleared Safety mask after Stop/Reset.
- Last Will: 6.583–6.634 s after bracketed USB disappearance.
- Finite 1024-GET burst: counted drops, heartbeats and subsequent Ping/GET; unchanged map. Not every request accounted for; shared drop counter has no per-queue attribution.
- Pico boots while broker absent, retries and later connects/subscribes; initial heartbeat is Ready without any state-changing command.
- Final firmware full 300-second baseline: 300/300, no sequence gaps, maximum dashboard gap 1.155 s; native internal maximum 1.155431 s. Only WiFi source observed; Ready state throughout.
- Live dashboard SET with mapping readbacks withheld: no false confirmation; independent GET proves car applied map; normal readback recovers. Standard broker URL restored; active map Z/A/Q/M revision 3.
- Host: 51 C checks and 20 dashboard protocol checks pass. Earlier UI simulation has 16 checks. On-target TEST=1 self-test suite was compile-checked earlier, not executed.

Tests span documented image revisions; latest code adds initial state seeding to the prior Safety-mask refresh and PM-off image. Exact evidence and image hashes are in TEST_LOG.md. Original failures are retained.

## Deferred / limits

- S1-U09 is supported by the saved user-observation report. A later note
  claimed a separate dashboard export existed; the final file audit could
  not find it. That claim is corrected by an appended TEST_LOG.md note;
  no raw dashboard export is being invented or substituted.
- Template metadata is unfinished: Student ID is still a placeholder.
  Tests ran on documented uncommitted working trees, a deviation from
  main's commit-before-testing instruction. Base commits, source manifests,
  patches and image hashes preserve available provenance. No commit/push
  has been performed; code and test records still need committing together.
- Actual Wi-Fi outage/recovery: explicitly deferred to keep the user's network available. Broker restart is a separate test.
- Track-distance testing and integrated subsystem-load delivery remain for later.
- Buddies 2, 3, 4 and 5 remain unintegrated. No physical motor timing, barcode A–Z/bidirectional scan or junction action has been verified by these tests.
- Large fragmented inbound processing is supported by live oversized-message checks and SDK parser source; callback counts were not instrumented on target.

Broker and local dashboard server remain running. Test proxy and dedicated captures were stopped after saving evidence.
