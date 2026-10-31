# RoboPico Buddy1–4 integration v0.1

**PROVISIONAL — Buddies1–4 implemented; Buddy5 NOT integrated; full-system hardware acceptance pending.**

This is the structured successor to combinedv0.5 at ac161d3. It preserves Continuous V2 steering, the original plus peak barcode readers and ramp-assistv0.6 settings. New behavior is coordinator-approved bounded ramp requests, common physical barcode approval, partial-frame reset on handback, and explicit ramp requested/effective-target telemetry.

- [Status](STATUS.md)
- [Setup and log download](../../../docs/setup.md)
- [Build](../../../docs/build.md)
- [Architecture/interfaces](../../../docs/architecture.md)
- [Evidence index](../../../testing/evidence/README.md)
- [Artifacts](artifacts/)

Feature logic is documented in each src feature README. Public APIs are in include/interfaces. There is one application entry point; versioned experimental trees are not part of the active build. Source origins are pinned in SOURCE_ORIGINS.json. The existing RTOS project is a separate build.
