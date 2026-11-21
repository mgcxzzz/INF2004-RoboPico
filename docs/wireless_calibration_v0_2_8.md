# Buddy 1–4 RTOS v0.2.8: wireless front IR calibration

This guide records the v0.2.8 introduction of wireless front IR calibration. For the current matched firmware and dashboard, see the [v0.4.3 release](../firmware/releases/v0.4.3/) and [calibration overview](calibration.md).

## Install and operate

Use the new complete bundle, because the dashboard needs `calibration_ui.js` and
the matching schema as well as the new UF2. Keep your old bundle/UF2 for rollback.
Flash `robopico_buddy1_4_rtos_v0_2_8_wireless_calibration.uf2` using BOOTSEL as usual.
Existing Wi-Fi and saved calibration remain intact; flashing alone never captures
or applies a new calibration. Start the bundled dashboard helper and confirm
firmware v0.2.8. Normal powerbank operation requires no USB terminal/capture helper.
Mac: `python3 tools/dashboard/run_dashboard.py` from the extracted bundle.

1. Export any previous run you want to retain. Click **Stop**, release both
   physical buttons, and wait for the car to confirm stopped status. Finish any
   stored-log download. Paused/countdown/fault-latched states cannot calibrate.
2. Open **Diagnostics → Front IR calibration**, near the bottom. The panel shows
   live **raw ADC**, active white/black references, and separate pending samples.
3. Click **Begin calibration**. This blocks Start until application, discard or
   timeout. Keep the normal sensor height/angle throughout the procedure.
4. Put **both front sensor windows on plain white mat**, comfortably inside its
   edges and away from seams/shadows. Hold still. Click **Capture white** and wait
   for **Capture complete**. Each capture averages 32 fresh frames (~0.32 s).
5. Put the **left front sensor on the black line**. Hold still, click
   **Capture left black**, wait for completion.
6. Put the **right front sensor on the black line**. Hold still, click
   **Capture right black**, wait for completion.
7. Review the four staged ADC references. Both black-minus-white spans must be
   at least 50 ADC, matching the existing calibration-validity rule. An unstable
   relevant channel (spread >128 ADC) must be recaptured. Repeating white clears
   both pending black captures. Repeating a black capture invalidates its old
   value until a new successful capture completes.
8. Choose **Use in RAM** for temporary use until reboot, or **Save and use** for
   persistence. Wait for **Saved and verified** and the active-value readback.
   ACK acceptance alone does not prove sampling/application/saving succeeded.
   Save uses the same CRC-protected flash record and verified writer as USB `k`.
   It only applies the candidate after successful verification. Failure leaves
   the active values unchanged and reports failure; a new explicit retry is needed.
9. **Discard pending samples** abandons staging without undoing a previously
   applied/saved calibration. An unused session expires after two minutes.
   Stop, loss of eligibility or ADC failure during capture prevents completion.

The existing `w/l/r/k/c` USB commands remain. A USB calibration command cancels
wireless staging first. `c` still restores the compiled original low-height preset
in RAM; it is not an undo of a particular dashboard calibration. Wi-Fi credentials
and the barcode sensor's separate calibration are not changed by this wizard.

## Short hardware acceptance test

Use sufficient mat beneath the entire turn/sensor sweep. A sensor over the table
edge can falsely look black even with good calibration.

- Capture the three positions above. Before applying, verify the active values
  stayed unchanged. Try Discard once, then perform a complete capture.
- Use in RAM; verify normalized readings approach 0 on white and 1000 on black
  for each channel. Verify Start is rejected while an unfinished session is open.
- Save and use, reboot on the same mat/setup, confirm the four active references
  survived. They should report saved-record origin 2.
- At Baseline 45, test a short straight, Pause/Resume/Stop, and one U-turn with
  full mat coverage. Recalibration alone does not prove turn alignment fixed.

Send the four saved references, three static-position screenshots, **Logs &
exports → Export all frames CSV**, events CSV and stopped Diagnostics/Motion
rows for the run. Include a short video if turn alignment still fails. No need
to repeat the full network/ramp test matrix just for this feature.

## Implementation and wire contract

CONFIG command 7: low argument byte `3`, high byte operation `0..7`: Read, Begin,
White, Left black, Right black, Apply RAM, Save/use, Cancel. Unknown operations
are rejected. Existing telemetry and speed-mode CONFIG keys retain their encoding.
Fresh requests pass the existing connection-epoch, queue-age and Stop barrier.
Mutations run only on the control owner and require its complete-stopped gate,
released buttons, no latched stop, no countdown/export and a good fresh ADC.
Read and Cancel do not require fresh ADC. Network code only queues commands.

Capture consumes the owner's existing ADC0/ADC2 acquisition, without a new ADC
task, blocking sample loop or priority change. Start readiness includes no open
calibration session. Status is pre-encoded into a 40-byte protected mailbox;
network readers copy it without reading live calibration state. No flash I/O
occurs under a copy lock. Duplicate operation/ID within five seconds cannot
restart capture or repeat saving.

Source 15 CC/28 is 40 bytes including the normal three-byte IS28 header. Payload:

| Payload bytes | Meaning |
| --- | --- |
| 0–1 | `CC 28` subtype/codec |
| 2–5 | Uptime ms, little endian |
| 6–8 | Revision uint16 and last modifying request ID |
| 9 | Operation low 3 bits; bit 3 reserved; result high 4 bits |
| 10 | Session/valid/ADC-ready/can-change/applied/saved bits 0–5; 6–7 reserved |
| 11–13 | Captured mask bits 0–2 and capture selector bits 3–4; sample count; origin |
| 14–19 | Active white L/R and black L/R, four explicit 12-bit fields |
| 20–25 | Staged white L/R and black L/R, same packing |
| 26–28 | Live raw L/R, two explicit 12-bit fields |
| 29–32 | Last good owner-frame timestamp, little endian |
| 33–36 | Last capture spreads L/R (12 bits each), then last request ACK |

All reserved bits are zero. Wire bytes are explicitly packed, never a native
struct image. The unsigned compact native state and 40-byte mailbox add **80
static bytes** compared with v0.2.7. Status is emitted after commands and at 1 Hz,
including when ordinary live telemetry is disabled. No automatic calibration
command is sent by the page. Stale (>2.5 s) or disconnected status disables changes.

## Software validation and limits

Targeted production-C tests cover sample ordering/averaging, failed recapture,
quality rejection, ADC loss, Stop serial, timeout/wrap, button/latch gates,
RAM application, verified save failure, duplicate flash suppression and codec.
Actual bridge tests cover owner queuing, stale and post-Stop request rejection;
protocol tests cover the new operation range and existing CONFIG encodings.
Node decodes the real C-generated packet and rejects invalid layouts. A local
Chrome test with simulated MQTT verifies the actual panel's stopped/fresh gates,
ACK/readback distinction, capture, save/failure, stale/disconnect and responsive
layout. The existing release dashboard/export test is also run.

The ARM release build, mechanical C policy/format gates, linked retained-feature
symbols, stack-frame/RAM budget and bundle hashes are checked. Native code tests
do not measure physical flash timing, sensor noise or motor behavior. Hardware
acceptance above is pending; this candidate is not promoted to main.
