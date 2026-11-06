# Buddy 1–4 micro T-Kernel architecture

Buddy requests -> shared navigation coordinator -> motion controller -> one
generation-checked normal PWM commit. Buddy 3 owns ordinary line AUTO; Buddy 4
requests bounded expiring ramp modifiers. Buddy 5 is not linked.

| Task / owner | Core | Priority | Responsibility |
|---|---:|---:|---|
| IMU/Buddy 4 |0|2|Sole I2C1 GP2/3 owner; 50 Hz FIFO processing and copied qualification state|
| USB service |0|3|TinyUSB only; optional for production driving|
| Kernel Wi-Fi |0|4|Sole CYW43/lwIP/MQTT owner; nonblocking copied TX/RX|
| Flash parking worker |0|5|Stopped calibration save cooperation|
| Safety |1|6|Buttons, urgent stops, control/network freshness; output reductions only|
| Navigation command task |1|10|Shared coordinator command boundary|
| USB command reader |0|12|Optional console commands|
| Motor/ADC/Buddy 2+3 |1|14|100 Hz encoders/control; front ADC plus2ms side polling; one normal output writer|
| Network application bridge |0|16|Copied commands, mapping/telemetry; never calls lwIP directly|
| Diagnostics |0|18|Paced optional-host text reports|

The network-supervised production profile allows USB absent. Start/Resume and
output commit still require valid clocks, released buttons, no latched fault,
fresh controller/network service and usable MQTT subscription. Recent TCP
publish-ack activity is limited to3s; heartbeat/release metadata remain enabled
when ordinary telemetry is off. Stale/lost service stops; reconnect never starts.

Stopped GET_LOG requests are routed to the core1 data owner, not read from a
network callback. A bounded5s refreshed lease blocks Start while transferring.
Run/dataset/count identity prevents mixing histories; nav last-sequence checks
reject a shifted ring. Data are explicitly encoded and copied into bounded TX
buffers. Each row uses at most two64-byte packets; browser retries/validates
parts and never calls an incomplete transfer complete. No host waits or new
sensor/motor work is added to the network service context.

Existing binary formats are retained. Source15 subtypeC2 adds compact release/
calibration identity; C3 adds runtime counters for passive wireless evidence.
The same source 15 sequence allocator is shared with peak/ramp/target packets.
See [data representation](data_representation.md) for widths/concurrency.

Feature logic and historical SDK adapters remain under
firmware/integration/buddy1_4/src/. Active RTOS task adapters are documented in
app_program/{rtos_bringup,line_navigation,barcode_navigation,network_bridge,imu_navigation}/.


v0.2.1 adds owner-latched encoder/network-stop snapshots and a latest-barcode
capture with bounded pre-roll. They are observation paths, not new command
owners. Network task reads copies and publishes C4/C5/C6 using the shared
source15 sequence allocator. GPIO/ADC/IMU ownership, priorities, coordinator
APIs and all safety/decoder thresholds are retained. See diagnostics_v0_2_1.md.
