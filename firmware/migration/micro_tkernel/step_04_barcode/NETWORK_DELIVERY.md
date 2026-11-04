# Network delivery investigation — 2026-11-04

Baseline: Step3B v0.1.1 at ba7cfe0. User supplied stationary/straight USB and
MQTT captures and broker.log. Motion timing stayed~100Hz with no new>15ms
intervals or ADC errors. User subsequently reports the remaining stop,
emergency stop, reset and reconnect/no-auto-resume checks passed.

The passive captures had heartbeat arrival gaps6.96s stationary and5.31s
driving. Observer reconnects inflated some gaps, but the broker itself also
logged3–5s heartbeat gaps. Its Pico timeouts outside those captures cannot be
classified as accidental failures because the user also performed deliberate
outage/end-of-test checks. Kernel polling advanced, Nagle was off, and copied
NET_TX showed many free TCP bytes while MQTT requests/ring backed up.

## Concrete allocation issue

The pinned Pico SDK2.2.0 lwIP `src/core/tcp_out.c` function
`tcp_pbuf_prealloc` has a special branch for LWIP_NETIF_TX_SINGLE_PBUF=1:
`alloc = max_length`. With default TCP_OVERSIZE=TCP_MSS, a small MQTT write
therefore reserves roughly a full1460-byte MSS despite the pbuf's visible
length being trimmed to its actual payload. Nagle disable does not bypass
this SINGLE_PBUF branch. The lwIP heap is16KiB. Increasing MQTT request slots
to16 can thus exhaust heap while `tcp_sndbuf` still reports ample byte room.
The old NET_TX did not measure those allocations.

This is a verified configuration interaction, and a plausible contributor to
the recorded stalls. Its exact causal share on this car is NOT established
without the new heap counters/hardware comparison.

## Correction

Only APP_RTOS_NETWORK sets TCP_OVERSIZE=0 and LWIP_NETIF_TX_SINGLE_PBUF=0.
The combination is required: lwIP rejects SINGLE_PBUF=1 with OVERSIZE=0.
The unmodified CYW43 `cyw43_ll_send_ethernet` already handles chains using
`pbuf_copy_partial`, flattening into its bounded SPI buffer. MQTT uses copied
TCP writes. No upstream SDK/driver source is edited.

`testing/rtos_migration/test_tcp_allocation.py` compiles the actual pinned
upstream allocation helper under both configurations. Fourteen60-byte writes
reserve20440 payload bytes under the old configuration versus840 under the
new one (allocation metadata/headers are additional). This experiment tests
the allocation interaction, not RF throughput or real heartbeat latency.

The existing16-request/1024-byte MQTT ring, ACK/HB headroom, Nagle disable,
latest live snapshots, non-blocking MBFs and extra four timeout slots remain.
1Hz heartbeat,5s MQTT keepalive, dashboard stale limits and safety-loss behavior
are unchanged. Exact-size allocation does not hide gaps by extending timeouts.

NET_MEMORY now copies heap used/max/capacity/failures and TCP queue/retry/RTO
state from the sole kernel owner. Bounded fields use masks/bitfields; copied
access uses the bridge spinlock. No subsystem reads live lwIP structures.
These diagnostics allow the next capture to distinguish heap pressure from
remaining TCP/RF/server delays. They are not broker-delivery acknowledgements.
The passive capture observer waits6s before reconnecting instead of3s, to
avoid losing buffered packets during a short stall. Actual timestamp gaps
are still recorded; this changes no firmware/dashboard timeout or the2s
baseline delivery expectation.

## Tribunal1 scope

Minimum-bit application representation and packet-buffer allocation are
separate concerns. Existing compact binary MQTT formats remain unchanged;
no JSON/text telemetry is introduced. New raw rows explicitly encode ADC12,
time32 and signed distance32 in10 bytes. New copied heap fields use15 bits
for0..16384, queue6 for0..32, retries4 for0..12; the error counter retains its
source16-bit range. Recent encoder cause/wheel remain2-bit fields. Packed
shared accesses are copied under locks, never unsynchronized read-modify-write.

This change is not a declaration of whole-firmware Tribunal1 compliance.
The Week13 field inventory, full RAM/message-size comparisons and functional
performance evidence still need a separate complete audit. Decoder RAM also
grows in Step4; its line-history ring is explicitly reduced to640 rows to
retain the reference63 decoder banks and2ms raw history within the RP2040's
RAM budget. Link/stack/source/UF2 checks are recorded in each build manifest.
