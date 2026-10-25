/**
 * @file    b1_net.h
 * @brief   WiFi and MQTT (Buddy 1): the bridge between the network side
 *          and the Buddy 1 task.
 *
 * TEAM.md section 3: lwIP and MQTT may only be called from the kernel's
 * WiFi service task (cyw43_utk_task). So Buddy 1 is split in two:
 *
 *   network side   lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c, runs in
 *                  cyw43_utk_task: connects, publishes, receives.
 *   task side      subsystem1/task_wifi.c, the Buddy 1 task: checks and
 *                  dispatches commands, owns the outgoing queue, counts.
 *
 * The network side only calls the functions below. They are implemented
 * in task_wifi.c with kernel calls that never wait (TMO_POL), so the WiFi
 * service task is never blocked. This header uses plain C types only: the
 * network side cannot include tk/tkernel.h next to the lwIP headers.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#ifndef B1_NET_H
#define B1_NET_H

#include <stdbool.h>
#include <stdint.h>

/* MQTT topics (all under one prefix, so one subscription sees them all). */
#define B1_TOPIC_TLM            "is28/tlm"  /* car -> PC: every buddy's frame  */
#define B1_TOPIC_HB             "is28/hb"   /* car -> PC: Buddy 1 heartbeat    */
#define B1_TOPIC_ACK            "is28/ack"  /* car -> PC: command answers      */
#define B1_TOPIC_CMD            "is28/cmd"  /* PC -> car: supervisory commands */

/* Outgoing message kinds (which topic a queued message goes to). */
#define B1_OUT_TLM              (0U)
#define B1_OUT_ACK              (1U)

/* Largest message body in the outgoing queue, bytes. */
#define B1_OUT_MAX              (40U)

/* Largest command accepted from MQTT (longer ones are cut here and then
 * rejected as B1_ACK_FORMAT, because a command is exactly 4 bytes). */
#define B1_CMD_MAX              (8U)

/* Network side timing. */
#define B1_HEARTBEAT_MS         (1000U)     /* NFR-WIFI-02: about 1 s     */
#define B1_KEEPALIVE_S          (5U)        /* broker loss seen in ~7 s   */
#define B1_CONNECT_TIMEOUT_MS   (10000U)    /* bounded connect attempt    */
#define B1_PUBLISH_PER_POLL     (4U)        /* bounded work per 2 ms poll */

/* ------------------------------------------------------------------------
 * Called by the network side (cyw43_utk_task). Never block.
 * --------------------------------------------------------------------- */

/** Next queued message, if any: kind (B1_OUT_*), body and length. */
bool     b1_net_take(uint8_t * p_kind, uint8_t * p_body, uint32_t * p_len);

/** Raw bytes received on B1_TOPIC_CMD (len may exceed B1_CMD_MAX). */
void     b1_net_command(uint8_t const * p_msg, uint32_t len);

/** Build the Buddy 1 status frame (heartbeat); returns its length. */
uint32_t b1_net_heartbeat(uint8_t * p_frame);

/** True once after the operating state changed or a PING arrived: the
 *  network side sends a heartbeat at once instead of waiting. */
bool     b1_net_heartbeat_now(void);

/** The network side's view of the link (b1_net_t). */
void     b1_net_set_state(uint8_t net);

/** One publish attempt finished: true = handed to TCP, false = lost. */
void     b1_net_count(bool b_published);

#endif /* B1_NET_H */
