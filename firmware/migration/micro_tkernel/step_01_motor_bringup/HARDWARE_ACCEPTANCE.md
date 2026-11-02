# Step 1 v0.1.1 — user hardware acceptance

Accepted 2026-11-02 before starting Step 2. Firmware source/artifact commit:
`9ed783de88039d6a5fd375699b78167233e84748`.

The submitted `2026-11-02_16-01-33_summary.json` reports:

| Measure | Result |
|---|---|
| Overall / timing | PASS / PASS |
| Motor readiness | READY; flags 12 (`0xc`); no blockers |
| Laptop elapsed | 40.010716 s |
| Kernel elapsed | 40.000000 s |
| Hardware TIMER elapsed | 40.000004 s |
| Control rate | 99.978 Hz |
| New intervals over 15 ms | 0 |
| Independent samples | 41 |

The accompanying PuTTY record shows a successful both-wheel automatic pulse,
left negative/right positive forward encoder counts, zero invalid transitions,
and GP20 during motion producing a reason 3 emergency latch (`0xe`). Requests
were rejected while latched; `x` returned readiness (`0xc`) and a subsequent
pulse succeeded. The earlier user report also confirmed `s`, GP21, GP20 and `e`.
The user then explicitly confirmed the test worked.

The historical maximum idle gap/late counter was retained, as intended; it
did not keep motor requests permanently inhibited after fresh clock validation.
Historical USB drops outside the clean timing capture are not a physical
stop-latency measurement. Stop timing at the wheels was confirmed functionally
by the user, not measured with an instrument.

Scope: Step 1 timing/start/encoder/stop acceptance only. This does not certify
IR tracking, junctions, barcodes, networking, ramps or obstacles under RTOS.
Raw user attachments remain outside the repository; this records the pertinent
results without copying unrelated session output.
