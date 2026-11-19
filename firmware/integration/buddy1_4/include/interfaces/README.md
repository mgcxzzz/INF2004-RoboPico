# Public buddy interfaces

Include `nav_api.h` from a buddy module. It exposes the shared motion request
facade. The [coordinator guide](../../src/navigation/README.md) describes
ownership, leases, non-blocking completion and assist expiry.

Buddy 3 supplies wheel targets and verified physical barcodes. Buddy 4 supplies
an expiring bounded ramp modifier. Buddy 5 requests a takeover, submits finite
distance and angle commands under its lease, then requests line hand-back.
See [the active interface table](../../../../../docs/interfaces.md) for the
combined profile. Normal PWM writes remain with the motion task. Invoke the
motion API from task context.
