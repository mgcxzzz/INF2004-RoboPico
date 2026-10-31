# Buddy5 integration point — NOT IMPLEMENTED

This provisional image contains no ultrasonic detection/avoidance implementation. The default `buddy_extension_init/poll` hooks in src/app/buddy_hooks.c are no-ops. The example here is host-tested interface demonstration code and is not compiled by default.

Buddy5 should poll without blocking, request `nav_request_takeover`, wait for PAUSED/stopped, execute bounded distance/angle commands with the returned lease, and inspect command completion. It should find the line and stop over it before `nav_resume_line`; use explicit abandonment for an interrupted junction. Takeover clears Buddy4 ramp assistance; Buddy3 cannot overwrite the lease. Add a real extension with CMake's BUDDY_EXTENSION_SOURCE option only after integration testing.
