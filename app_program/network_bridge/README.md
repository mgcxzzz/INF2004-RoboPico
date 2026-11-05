# Application-to-kernel network bridge

Main release APP=rtos_buddy1_4 enables the tested Buddy1–4 adapters plus
network-supervised powerbank operation and stopped wireless exports. See
[main setup](../../docs/setup.md) and [architecture](../../docs/architecture.md).
The checkpoint history below describes the port and USB-supervised test profiles.

**Step4 update:** `APP=rtos_barcode_wifi` uses the same connected command and
safety foundation, adding the Buddy3 ADC-owner decoder adapter. Live mapping
crosses cores as a locked snapshot. Source11 status/events share one sequence
allocator; only STATUS is coalesced. See the
[Step4 guide](../../firmware/migration/micro_tkernel/step_04_barcode/README.md).
Step3B v0.1.2 and Step4 use exact-size TCP allocations plus copied heap/retry
diagnostics; [investigation and evidence](../../firmware/migration/micro_tkernel/step_04_barcode/NETWORK_DELIVERY.md).
IMU/ramp and Buddy5 remain deferred.

**Step3B update:** `APP=rtos_line_wifi` adds the Step2 controller to this bridge;
`APP=rtos_network` remains the motor-disabled Step3A bench. See the complete
[connected execution/command specification](../../firmware/migration/micro_tkernel/step_03_connected_line/ARCHITECTURE.md)
and [setup/test guide](../../firmware/migration/micro_tkernel/step_03_connected_line/README.md).
`APP_RTOS_NETWORK` selects the kernel robot transport; `APP_RTOS_NET_BENCH`
adds only the bench motor/start inhibition. Barcode is disabled in those two
profiles; it is enabled only in the separate Step4 profile.

Current buffers reserve TX1024bytes, ACK256, HB128, RX256 and (Step3B) owner
commands256. Copied RX/control envelopes are25bytes; out-of-queue validated
Stop/E-stop/Pause bits survive ordinary queue saturation. Full/malformed
messages are counted. Telemetry expires250ms; ACK/heartbeat2s. All operations
are non-blocking TMO_POL and all mutable packed snapshots have short locks.

In Step3B the control owner executes Start/Pause/Resume/Reset through the shared
API/coordinator before returning the actual decision ACK. USB supervision and
detected network loss stop the session; reconnect never resumes it. Latest
copied source9 motion is20Hz and source11 coordinator4Hz, with source1 heartbeat
1Hz. CONFIG telemetry off/on leaves heartbeat/safety running. Source9 flag0x40
identifies this RTOS runtime to the reused dashboard. Stored logs use USB d.

The following describes the original Step3A scope; its published artifact is
retained unchanged for rollback.

Step3A is a **stopped network bench**, not the completed Wi-Fi driving port.
`APP=rtos_network` runs the verified1ms kernel tick and100Hz clock/encoder/stop
foundation. Both command rejection and the common output callback inhibit motor
starts. It does not acquire front ADC, load/save calibration, decode barcode,
execute junctions, read IMU or activate Buddy5.

## Ownership and flow

1. Core0 kernel `cyw43_utk_task` (priority4,8KiB stack) alone owns the CYW43439,
   lwIP, DHCP/TCP timers and MQTT. Its library implementation remains under
   `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, selecting the robot bridge
   include only for this profile. The example/demo implementation remains intact.
2. Core0 `rn_app_task` (priority16,4KiB stack) parses application commands,
   constructs the existing compact Buddy1 heartbeat and mapping/ACK frames, and
   exchanges copied messages with the owner. It never calls lwIP or MQTT.
3. Separate TX kernel message buffers reserve ACKs, heartbeat and telemetry.
   An RX buffer is256bytes. Every buffer operation uses TMO_POL.
   Oversized messages are rejected; full buffers increment copied drop counters.
   TX cleanup and owner publish work have bounded loops of four operations.
4. MQTT callbacks classify the known topic and copy up to8payload bytes before
   returning. Announced/received lengths must agree; truncation becomes an invalid
   command length, never a valid command prefix. The topic pointer is not retained.
5. All bridge objects exist before the release/acquire ready flag is set. The
   kernel task may start before usermain, but cannot access bridge buffers until
   ready. Mutable counters/status snapshots use the same IRQ-safe SMP spinlock
   pattern as the verified motion boundary, with no kernel calls under that lock.
6. Stop/Pause/E-stop requests reach the shared navigation/motion API as Buddy1
   via `bringup_stop_from`; the network owner never writes motor outputs.

## Protocol and reconnect

The existing `wifi_protocol.c` and `wifi_packets.c` are reused unchanged. MQTT
topics remain is28/cmd, is28/ack, is28/hb and is28/tlm. Source1 heartbeat and
source8 mapping packets are binary. No source9/10/IMU data are invented in this
bench. Start/Resume/Reset/config/log-download are explicitly unsupported here;
manual Drive/Turn/Steer remain prohibited. Mapping is RAM-only and restored to
the dashboard after a broker reconnect, but not after reboot.

Periodic heartbeat uses kernel milliseconds at1Hz. Forced Ping responses do not
reset/postpone that deadline. MQTT keepalive is5s; failed connections retry after
3s, connecting and SUBACK waits are bounded. Subscriptions are re-established on
every accepted connection. One client owns one mqtt.c internal timer; the
existing `LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4` pool is retained and build-checked.
There is no extra self-rearming heartbeat timeout chain.

TX messages older than2s are discarded, including a retained ERR_MEM retry.
RX commands older than500ms are rejected using wrap-safe elapsed time. Queue
pressure/drop behaviour is observable; this bench does not claim driving-time
STOP latency or serve as the final supervisory safety design.

## Settings and storage

The kernel owner reads the validated WCF1 page at0x1aa000. The host provisioning
helper writes only this aligned4KiB sector through BOOTSEL, with CRC validation,
without saving passwords/private UF2s locally. It does not overwrite the RTOS
calibration sector0x1a9000 or SDK history0x1d0000. The public firmware contains no
SSID/password and needs no `wifi_secrets.h` for this profile. Other example
profiles retain their existing build-time configuration.

Heartbeat fields use the existing compact schema:2-bit network state,3-bit
operating state,4-bit command,3-bit result,14-bit saturated uptime and bounded
counters. Mapping uses four5-bit letter indices plus configured/revision bits.
Bridge storage uses minimum byte containers with explicit masks in the existing
codecs; the messages are fixed-size/bounded and the topic is a small enum.
The full project-wide minimum-bit audit remains a later integration task.

See the checkpoint README for hardware tests. Kernel worker progress plus USB
clock capture and actual broker-received heartbeat timestamps must all pass;
host tests and linked-size checks alone cannot establish radio performance.
