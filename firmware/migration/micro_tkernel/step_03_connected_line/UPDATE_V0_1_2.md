# Step3B v0.1.2 — network allocation comparison

This image is the barcode-free comparison for Step4. The steering, front
calibration, motor power, recent encoder guard, USB supervision and1536-row
line history remain as in v0.1.1. It adds exact-size TCP allocations and copied
NET_MEMORY heap/TCP diagnostics. Hardware verification is pending.

The same source also builds Step4 with physical barcode decoding enabled.
See [Step4 setup and tests](../step_04_barcode/README.md) and its
[network investigation](../step_04_barcode/NETWORK_DELIVERY.md).

User reports v0.1.1 safety tests passed; no need to repeat all of them merely
to isolate the new network allocation setting. Test stationary Telemetry off
for60seconds, then on for60seconds, keeping the USB capture and broker open.
Keep one dashboard/robot, avoid extra commands within the steady intervals,
and send the three capture files plus dashboard_session/broker.log. The new
heap counters must be assessed alongside received heartbeat gaps and reconnects.
The existing~2s baseline delivery expectation has not been relaxed.

Normal flashing retains Wi-Fi/calibration sectors. Reuse Wi-Fi for the same
SSID/password/broker IP. Expected current car calibration is white122,138 /
black246,267; check actual readback and do not recalibrate for the comparison.
The complete Step4 bundle includes this UF2 beside the Step4 UF2. Only one
application image is flashed at a time. The accepted v0.1.1 package is retained
for rollback. Main is unchanged; this is not final Buddy1–4/5 acceptance.
