# Public buddy interfaces

Include `nav_api.h` from a buddy module. It exposes the same request facade as `motion_api.h`, never a second motor writer. Read `../../src/navigation/README.md` for ownership, leases, non-blocking completion and assist expiry. `buddy_hooks.h` is the optional cooperative Buddy5 extension boundary.

Buddy3 supplies wheel targets and verified physical barcodes. Buddy4 supplies an expiring bounded ramp modifier. Buddy5 will request a takeover, then distance/angle commands and an explicit line handback. Its implementation is absent by default. Do not include application adapter headers from a buddy, modify another buddy's globals, call motion APIs from an ISR, or write motor PWM from a subsystem.
