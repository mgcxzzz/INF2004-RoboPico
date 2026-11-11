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


v0.2.2 keeps the same tasks, priorities and owners. The core1 tracker can emit
signed counter-rotating search requests through Buddy3's existing coordinator
API. Only the explicit stationary-search envelope permits their signed motor
outputs; ordinary forward tracking and U-turn envelopes keep their boundaries.
Wheel-direction changes cross a zero-output encoder-still dwell. No subsystem
writes motor pins or calls lwIP directly.

C7 width decisions freeze their measured threshold/hysteresis and result across
later label learning. C8 is live bend/search state; both are copied under the
existing owner lock and encoded outside it, published with ordinary telemetry.
The controller rows retain their 56-byte ABI and use two spare bits for the
full-rate bend phase. Binary fields and source IDs remain compatible.

## IMU end-of-run safety (v0.2.3)

Core0 acquisition feeds the calibrated raw FIFO to a bounded integer guard.
Its single-writer state is copied under the IMU spinlock; calculations and I2C
are outside that lock. Core1 safety polls the sticky result, cancels countdown/
line requests, and invokes Buddy4 Stop through the shared motion API/coordinator.
Commit/output gates also inhibit unsafe/stale IMU data. No new normal PWM writer,
RTOS task, I2C client or lwIP caller is introduced.

Core1 Reset revalidates fresh upright evidence under the copy lock while the
supervisor latch is protected, and queues a reset instead of editing a concurrent
core0 calculation. Pending reinitialization inhibits outputs/Start immediately.
The acquisition task consumes that request and recalibrates. No automatic restart
occurs. Historical collision classification remains only in legacy profiles.
