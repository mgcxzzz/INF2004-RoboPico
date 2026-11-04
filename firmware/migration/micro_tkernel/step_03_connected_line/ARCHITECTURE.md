# Step3B execution and interface boundary

- Core0 kernel CYW43 service owns radio, lwIP, MQTT and timers. Application
  code never calls these APIs. MEMP_NUM_SYS_TIMEOUT keeps the existing
  LWIP_NUM_SYS_TIMEOUT_INTERNAL+4 provision and compile-time check.
- Core0 application bridge parses copied bounded RX messages, queues commands
  to the core1 owner, supplies actual-decision ACKs and publishes copied
  telemetry. All kernel message-buffer operations are TMO_POL. It never writes
  PWM. Start/Resume/Reset are executed by the control task, not MQTT callbacks.
- Core1 control owns front/side ADC, calibration, line planner and PI state.
  Nominal10ms control and2ms side polling are unchanged; safety runs at1ms.
  Normal motion is Buddy3 -> motion API -> coordinator -> generation-checked
  commit -> one motor writer. Buddy1 Pause uses a coordinator lease and output
  invalidation; Resume checks stillness/line/recovery before returning Buddy3.
- Stop/E-stop out-of-queue bits are set only after full protocol validation.
  The safety task consumes them even when ordinary RX/command buffers are
  full. A command barrier, connection epoch and received TIMER timestamp reject
  requests queued before a stop, from an earlier connection or over500ms old.
  Full/oversized messages are counted and never executed as a valid prefix.
  Saturation can lose an ACK; absence of ACK does not imply Stop was ignored.
- Fresh dashboard Start requires1s after observing this connection; an early
  queued Start at resubscription is rejected. The kernel owner discards all
  retained command publications before RX copying. The public lwIP callback
  omits RETAIN; the port reads its pinned mqtt_priv.h fixed-header byte during
  pub_cb, where mqtt.c also reads QoS. No upstream library is edited and no
  private client pointer reaches application tasks. Commands from the supplied
  dashboard are non-retained. Reconnect does not resume a session.
- Pause keeps the session active for supervision/logging while output is zero.
  The line planner is not advanced while paused; side edges are not processed.
  Resume of ordinary tracking restarts acquisition from current pose, clears
  controller/envelope history and preserves pending route/log. Interrupted
  navigation requires explicit Stop/reposition/Start; no silent abandonment.
- USB DTR remains mandatory. Network poll/app stale250ms or reported transport
  loss cancels motion/countdown. The250ms check follows software evidence of
  failure, not the unknowable instant of RF failure. Local GP20/21 remain.
- Three independent output queues reserve ACK and heartbeat capacity ahead of
  ordinary telemetry. Telemetry queued/pending older250ms is discarded; ACK/HB
  expire2s. At most4 publishes/owner poll,4 RX/app pass,2 control commands/tick.
  A failed publish can briefly hold one packet until its expiry; this is a
  bounded non-blocking bridge, not a reliable delivery/zero-loss guarantee.
- Kernel disables radio power saving for this connected-drive profile and
  prints set/readback results. Step3A retains its prior radio configuration.
  This addresses possible sleep batching; hardware must confirm actual gaps.
- Front calibration import/capture/save and flash sectors are unchanged.
  Stopped k uses the existing two-core RAM parking protocol; no XIP fetches or
  kernel calls occur while flash is unavailable. Clock readiness is rechecked
  after an idle control gap. Never save while moving; Start is blocked during
  a RAM log export. Wi-Fi provisioning remains BOOTSEL settings-only.

## Compact messages and telemetry

The copied RX/control envelope is25bytes: kernel received-ms LE32@0,
length@4, payload prefix8bytes@5, barrier LE32@13, connection epoch LE32@17,
hardware received-us LE32@21. Length9 is an invalid/oversized sentinel. Critical
packed words and copied status use short IRQ-safe spinlocks; no formatting,
blocking operation or subsystem API runs under the bridge's data lock.

Existing protocol source1 heartbeat is1Hz, source9 motion48bytes is20Hz and
source11 coordinator26bytes is4Hz. No change to their lengths, units or offsets.
Each source has its own sequence counter, including mapping versus heartbeat.
**Source9 flags bit6 (0x40) explicitly identifies the RTOS connected runtime**;
bit7 keeps the prior run-relative stored-timestamp meaning. The shared dashboard
uses bit6 to identify this profile instead of guessing SDK from the source ID.
Older SDK packets retain their original display. Source9 normalized barcode
level stays unavailable; physical barcode decoding is not claimed.

Heartbeat uses dashboard states Ready0, Running1, Paused2, SafeStop3,
E-stop5, Fault6. Countdown is Ready, not Running; disconnected/off status never
implies permission to drive. Source11 continues to report the actual coordinator
owner/mode/generation/fault independently. Calibration/steering/turn acceptance
have no network-specific thresholds.

Mapping replacement/readback remains RAM-only across broker reconnect. There
is no physical barcode routing until Step4. Manual Drive/Turn/Steer remain
prohibited. CONFIG telemetry off/on is supported; heartbeat and safety keep
running. GET_LOG is unsupported here; full-rate RAM export is USB d.
