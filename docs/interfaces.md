# Interfaces

See [public declarations](../firmware/integration/buddy1_4/include/interfaces/nav_api.h) and the [coordinator logic](../firmware/integration/buddy1_4/src/navigation/README.md).

| Caller | Requests |
|---|---|
|Buddy3|nav_submit_navigation; nav_accept_barcode for physical results; nav_submit_barcode for API-injected routes|
|Buddy4|nav_request_ramp_assist; optional bounded nav_request_speed_assist; nav_get_ramp_assist readback|
|Buddy5 (future)|nav_request_takeover; nav_drive_distance; nav_turn_relative; nav_get_command; nav_resume_line|
|Any buddy|nav_request_stop; status/command queries; bounded fault reporting|

Requests are normal cooperative/task-context calls. Accepted finite motion is not completed motion. Leases protect ownership, generations protect the hardware commit and ramp TTL protects temporary modifications. Generic takeover manoeuvres and the actual Buddy5 extension still require physical acceptance.
