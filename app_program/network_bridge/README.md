# Application-to-kernel network bridge

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
3. Two TX kernel message buffers separate ACKs(256bytes) from telemetry/heartbeat
   (1024bytes). An RX buffer is256bytes. Every buffer operation uses TMO_POL.
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
