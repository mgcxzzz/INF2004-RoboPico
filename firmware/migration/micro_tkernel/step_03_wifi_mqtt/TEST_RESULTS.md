# Step 3A network bench — software checks

Date:2026-11-04. These are the original pre-hardware software checks. Subsequent
[stopped baseline and functional broker-reconnect results](HARDWARE_ACCEPTANCE.md)
are recorded separately. No live Pico, broker or car commands were issued while
preparing the software checkpoint.

- 19 strict production-C programs pass, covering existing clock/start/stop gates,
 coordinator ownership and stale commits, line/ADC/junction regression tests,
 packet formats and the new motor-disabled command policy.
- 47 Python tests pass, covering existing captures, WCF1 provisioning and network
 evidence analysis. Tests reject frozen worker counters, heartbeat uptime drift,
 large gaps and disconnected monitors. Provisioning tests validate the exact
 aligned4KiB UF2 sector and reject wrong BOOTSEL targets/invalid fields.
- Step1, Step2 and Step3A ARM profiles build after the change. The network image
 retains1ms kernel ticks and compiles the common clock/control/encoder/stop tasks.
- Shared feature source is unchanged from Step2 v0.1.2. Against main7843f45,
101 shared files remain byte-identical and the same two opt-in junction-header
 edits remain the only reviewed differences. They are inactive in this bench.
- Packaging checks kernel CYW43/MQTT and copied bridge symbols, absence of the
 line/calibration writer/physical barcode/Buddy5 code, UF2 family/address bounds,
 static RAM headroom, and the output callback disassembly calling only OFF.

The application task constructs the existing binary heartbeat/mapping/ACK
packets. It never calls lwIP/MQTT. The kernel owner uses TMO_POL for all bridge
messages, bounded polling loops, restored subscriptions, finite connect/SUBACK
waits and the existing +4timer-slot reserve. ACKs have a separate TX buffer;
overflows/truncation and stale messages have explicit rejection/drop behaviour.

GPIO/encoder/clock foundation still requires a real clock comparison when the
radio is running. The new capture records actual broker-received heartbeats
alongside USB counters; neither a local broker listener nor compiling certifies
the Pico-to-broker path. Broker reconnect, steady radio performance and dashboard
confirmation require the README bench test. Video is unnecessary for this test.

Step2 L/R and sharp bends remain open for extended-mat testing. Step3B must add
motion telemetry, dashboard driving supervision and moving-heartbeat tests before
Step3 is accepted. Barcode decoders and IMU/ramp remain later migration steps.
