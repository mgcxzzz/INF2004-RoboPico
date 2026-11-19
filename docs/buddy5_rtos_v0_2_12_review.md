# Buddy5 RTOS candidate 0.4 hardware handoff

The branch is based on the Buddy 1–4 BARR-C v0.2.12 release at `81a994f57e3828364492cc2806fcbea124dfa18a`. Build `APP=rtos_buddy1_5` with the existing [integration instructions](buddy5_rtos_integration.md) and use the v0.2.12 dashboard.

Candidate 0.3 stopped soon after Start on an empty straight. Its last completed sonar result was a clean no-return, 322 ms old at the first `SENSOR` hold status. Candidate 0.4 lets the motion loop service a pending sonar transaction during guard, in addition to the regular Buddy5 worker. It preserves the 300 ms freshness limit and the hold after three consecutive faults. The ARM build and mission simulation passed. A host check using the sonar driver and simulated echo interrupts received a result every 80 ms or less with only the motion loop servicing pending pings.

On hardware, candidate 0.4 followed a clear track. With the demo obstacle on a section the operator identified as straight, the car stopped at a reported front range of 295 mm. The dashboard recorded `NOT_STRAIGHT` with gate reason `BEND_ENTRY` (module 5). Width and direction probing did not start. This leaves a straight-section false block to investigate.

`rtos_line_straight_sample()` reports both a pending bend and a candidate bend to the gate. The dashboard code combines them under `BEND_ENTRY`. The status from this run cannot distinguish which signal closed the gate or how long it was active. Review the Buddy3 bend inputs and the encoder window at the point the close echoes are counted. Keep the stop protection on an actual curve while checking the straight placement.

The width estimator, direction probe, and bypass have no behaviour changes in 0.4. The end-to-end obstacle run is still unverified.
