# Navigation coordinator and request routing

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Ownership and execution

All buddy-facing calls enter through `include/interfaces/nav_api.h`, which names the same shared facade implemented by `motion_api.c`. Calls are non-blocking requests, not a second motor driver. `nav_codec.c` round-trips each request through a validated canonical bit stream before policy handles it.

At a normal run start, Buddy3 owns AUTO navigation. Line following, barcode recognition and junction turning remain Buddy3's navigation work. Buddy4 requests a bounded ramp modifier while Buddy3 remains owner. Buddy5 is not implemented in this image; its future avoidance hook may request a takeover and receive a lease.

A takeover removes output immediately, then waits for measured stillness before a finite move is allowed. Every external drive/turn/resume carries the current lease. Buddy5 may pre-empt an existing owner; stale/incorrect leases and other owners' moves are rejected. Finite motion reports RUNNING, DONE, CANCELLED or FAILED with a command ID. Resume requires the existing stopped/line-present confirmation; an interrupted junction requires explicit abandonment. It clears partial decoder state while preserving a completed pending route according to policy.

`motion_update` computes the approved targets. `motion_commit` validates its generation immediately before the one output owner writes PWM; a stop/takeover that invalidates a calculated command prevents that old command being committed.

## Ramp and physical barcode additions

`nav_request_ramp_assist` accepts only Buddy4 and bounded boost/push/hold values, with TTL1–200ms. Application controller-selection context determines whether assistance is permitted. New requests normally expire after50ms unless renewed. Stop, takeover, leaving eligible following or expiry clear them. These requests preserve the navigation owner and generate audit records.

`nav_accept_barcode` authorizes a verified physical A–Z result while AUTO and navigation is not busy. The Buddy3 service then maintains its own counters, mapping revision and pending queue. `nav_submit_barcode` remains the API-injected route path and does not increment physical decode counters. Both decoders share physical acceptance; conflicting reads cancel the pending action.

Request kinds10/11 identify physical barcode approval and ramp assistance. Ramp audit goal/reason0=clear,1=accepted/update,2=Stop/takeover,3=expired/ineligible on read,4=following context removed. Requests rejected for mode/owner/bounds keep their result. Accepted per-tick ramp renewals are audit-rate-limited to100ms while start/clear transitions remain recorded.

## Audit and tests

Press `z` while stopped/settled for requester, owner, request, result and target history. Dashboard navigation export carries the saved31-byte audit rows. Ramp effective-target telemetry is separate, because a requested optical target differs from the constrained target and the slewed motor reference.

New tests verify TTL/bounds/serialization, Buddy3 ownership during Buddy4 assistance, wrong requesters, takeover clearing and physical barcode authorization. Existing tests retain lease, finite manoeuvre, stop-generation and resume checks. Calls are normal cooperative/task-context calls, not ISR entry points. This image remains the working Pico SDK architecture, not the separate micro T-Kernel firmware.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
