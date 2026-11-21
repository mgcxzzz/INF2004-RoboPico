# Buddy 5 obstacle algorithms

The [v0.4.3 combined release](../../../../releases/v0.4.3/) links these obstacle algorithms in `APP=rtos_buddy1_5`
in this directory through the [RTOS obstacle adapter](../../../../../app_program/obstacle_navigation/README.md).
The adapter services sonar and servo, coordinates mission stages, and invokes
the shared navigation lease and motion APIs.

The reviewed modules provide width probing and estimation, direction probing,
the automatic route, and line reacquisition. The reviewed SDK-derived files
are covered by [coding exception CEX-08](../../../../../docs/coding_exceptions.md)
to preserve source parity. See the
[integration and provenance notes](../../../../../docs/buddy5_rtos_integration.md)
for the port details and wire formats. The
[v0.2.13 test guide](../../../../../docs/buddy1_5_v0_2_13.md) records software
validation and the remaining physical checks.
