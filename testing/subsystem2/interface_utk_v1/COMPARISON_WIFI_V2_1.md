# WiFi V2.1 versus the professor-style kernel build

Checked against branch commit `10d7886` on 28 October 2026. The teammate's
`interface_wifi_v2` folder is preserved unchanged.

| Item | Teammate WiFi V2.1 | UTK Network V1 |
| --- | --- | --- |
| Runtime | Standalone Pico SDK, network worker on core 1 | micro T-Kernel, kernel Wi-Fi service on processor 1 / physical core 0 |
| MQTT owner | Standalone `wifi_transport.c` worker | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, called by `cyw43_utk_task` |
| Application bridge | Bounded copied SDK-side queues | Buddy 1 TX/RX kernel message buffers; network calls use `TMO_POL` |
| Timeout allowance | `LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4` = 11 | Same formula = 10 with this build's different feature configuration |
| Scope | Full standalone line/motion harness | Motors-off network, heartbeat, mapping and coordinator-state proof |

## The actual failure and the fix

The captured V2 panic says `pool MEMP_SYS_TIMEOUT is empty`. lwIP's default
pool contains only the enabled core protocol timers. The MQTT library adds
its own cyclic timer after TCP connects, before the MQTT CONNACK arrives.
With no spare slot, that allocation asserts and stops the network owner.
This explains an IP-ready Pico with frozen network polls and no MQTT session.
The old `mqtt_result=0` was acceptance of the asynchronous connect request,
not evidence that the broker completed the handshake.

The new kernel snapshot **already inherited the four-slot allowance** from
Buddy 1 commit `b09403074939f45cf4d7c11b7606148f6c2a4e73`. It did not need a
second increase. Its configuration comment has been corrected: the current
Buddy 1 bridge polls heartbeat/retry deadlines rather than allocating extra
lwIP timers for them. A compile-time guard in `mqtt_utk.c` now refuses a
configuration with fewer than four additional slots.

The earlier diagnosis missed this timeout-pool setting. Switching to the
kernel-service architecture satisfies the professor's integration model;
that architecture change alone is not the explanation for the allocator
panic. Both architectures need enough timer entries.

## Regression evidence

`run_lwip_timeout_test.py` adapts the teammate's actual-allocator test. It
compiles the real SDK `memp.c` and `timeouts.c` against this kernel build's
`lwipopts.h`. Clock/platform operations and unrelated timer callbacks are
test stand-ins. The timeout-list and allocation implementations are real.

Results in `logs/lwip_timeout_pool_regression.txt`:

- Removing only the pool override reproduces the exact empty-pool assertion.
- With the shipped override, core timers occupy six slots and capacity is ten.
- The application timer completes 100 callback reschedules.
- 1,000 add/remove cycles pass without exhaustion.

The test ran with 32-bit MinGW GCC 9.2.0, matching the target's pointer size
and four-byte allocator alignment. Host tests do not establish radio or
broker operation. The user reports that their teammate physically tested
V2.1 successfully; the kernel image still needs its own Pico/broker test.

## What is deliberately not copied

V2.1's standalone worker is not inserted into the kernel build. Doing that
would create a second networking owner instead of following the professor's
model. The existing full-course V2.1 driving image remains available in its
own folder. No line-following thresholds, steering gains or motor PWM
settings were changed by this comparison.

The first kernel test proves connection, continuing heartbeats, command
ACKs and confirmed barcode-map readbacks. Physical motion integration is
the next phase after that test succeeds.
