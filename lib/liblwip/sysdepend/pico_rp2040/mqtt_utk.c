/*
 *	mqtt_utk.c
 *	MQTT client for micro T-Kernel (RP2040, NO_SYS=1) -- network side of
 *	the IS28 WiFi and MQTT subsystem (Buddy 1).
 *
 *	Changed from the template's heartbeat demo (TEAM.md section 3: the
 *	"cloud / reporting" subsystem adds a queue that application tasks
 *	fill, and this file reads it with TMO_POL and publishes it). Agreed
 *	as interface IF-25 in TEAM.md section 5.
 *
 *	Everything here runs on cyw43_utk_task, the single owner of the radio
 *	and lwIP (docs/RELEASE.md safety rule 6). cyw43_utk.c calls
 *	mqtt_utk_start() about every 2 ms while the WiFi link is up; each call
 *	does a bounded amount of work and returns. Buddy 1's task side
 *	(app_program/subsystem1/task_wifi.c) is reached only through the
 *	b1_net_*() functions, which never block (b1_net.h).
 *
 *	Statechart 6.1 (Week 6), network part:
 *	  Disconnected --link up--> Connecting --CONNACK--> Operational
 *	  Connecting --refused / timeout--> Reconnect Wait
 *	  Operational --broker or link lost--> Reconnect Wait
 *	  Reconnect Wait --back-off (1, 2, 4, 8 s)--> Connecting
 *	Operational: heartbeat on is28/hb every B1_HEARTBEAT_MS (and at once
 *	when the operating state changes), queued frames on is28/tlm and
 *	command answers on is28/ack, commands received on is28/cmd.
 *	Losing the link never changes the car's operating state.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it is not subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#include "../../../../app_program/subsystem1/b1_net.h"
#include "../../../../app_program/subsystem1/b1_proto.h"

#define MQTT_UTK_GAP_MS     500U    /* no poll for this long = link was down */
#define MQTT_UTK_LWT_TOPIC  "is28/lwt"

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		net_state = B1_NET_DISCONNECTED;
static u32_t		retry_at;
static u32_t		connect_started;
static u32_t		attempts;
static u32_t		last_poll;
static u32_t		next_hb;

/* One message taken from the queue but not yet accepted by mqtt.c (its
 * output buffer was full): kept and offered again on the next poll. */
static u8_t		pend_kind;
static u8_t		pend_body[B1_OUT_MAX];
static u32_t		pend_len;
static u8_t		pend_valid;
static u8_t		hb_frame[B1_OUT_MAX];
static u32_t		hb_len;

/* Incoming command staging: a publish arrives as one topic callback and
 * one or more data callbacks. */
static u8_t		rx_is_cmd;
static u8_t		rx_buf[B1_CMD_MAX];
static u32_t		rx_len;
static u32_t		rx_total;

static void set_state(u8_t state)
{
	if(state != net_state) {
		net_state = state;
		b1_net_set_state(state);
		tm_printf((UB*)"mqtt: %s\n", b1_net_name(state));
	}
}

static void wait_and_retry(u32_t now)
{
	attempts++;
	retry_at = now + b1_backoff_ms(attempts);
	set_state(B1_NET_RECONNECT_WAIT);
}

static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;
	rx_is_cmd = (u8_t)(strcmp(topic, B1_TOPIC_CMD) == 0);
	rx_len = 0;
	rx_total = tot_len;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;
	if(!rx_is_cmd) {
		return;
	}
	for(i = 0; i < len && rx_len < B1_CMD_MAX; i++) {
		rx_buf[rx_len++] = data[i];
	}
	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		/* Copy into Buddy 1's bounded RX buffer before returning.
		 * Preserve oversize detection; valid commands are 4 or 5 bytes. */
		b1_net_command(rx_buf, rx_total);
		rx_is_cmd = 0;
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: listening on " B1_TOPIC_CMD "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	u32_t	now = sys_now();
	err_t	err;

	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		attempts = 0;
		set_state(B1_NET_OPERATIONAL);
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		err = mqtt_subscribe(client, B1_TOPIC_CMD, 0, subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}
		next_hb = now;			/* heartbeat at once */
	} else {
		/* Refused, timed out or dropped (keepalive). mqtt.c does not
		 * retry by itself; do it from the poll, after the back-off. */
		tm_printf((UB*)"mqtt: connection status %d\n", (int)status);
		wait_and_retry(now);
	}
}

static void start_connect(u32_t now)
{
	struct mqtt_connect_client_info_t	info;
	static const u8_t			offline = 0;
	err_t					err;

	/* Close whatever is left (a half-open connect after a link drop
	 * would otherwise make mqtt_client_connect() return ERR_ISCONN).
	 * A no-op when already closed. */
	mqtt_disconnect(client);

	memset(&info, 0, sizeof(info));
	info.client_id   = MQTT_CLIENT_ID;
	info.keep_alive  = B1_KEEPALIVE_S;
	/* Last will: the broker tells the dashboard at once if the car
	 * vanishes without disconnecting (power, WiFi out of range). */
	info.will_topic  = MQTT_UTK_LWT_TOPIC;
	info.will_msg    = (const char *)&offline;
	info.will_msg_len = 1;
	info.will_qos    = 0;
	info.will_retain = 0;

	tm_printf((UB*)"mqtt: connecting to " MQTT_BROKER_IP " ...\n");
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &info);
	if(err == ERR_OK) {
		connect_started = now;
		set_state(B1_NET_CONNECTING);
	} else {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n",
			  (int)err);
		wait_and_retry(now);
	}
}

/* Heartbeat, then up to B1_PUBLISH_PER_POLL queued messages. QoS 0: a
 * message counts as published once mqtt.c has taken it for TCP. ERR_MEM
 * (output buffer full) keeps it for the next poll. */
static void pump(u32_t now)
{
	u32_t	n;
	err_t	err;

	if(hb_len == 0 && (b1_net_heartbeat_now() || (s32_t)(now - next_hb) >= 0)) {
		hb_len = b1_net_heartbeat(hb_frame);
		next_hb = now + B1_HEARTBEAT_MS;
	}
	if(hb_len != 0) {
		err = mqtt_publish(client, B1_TOPIC_HB, hb_frame, (u16_t)hb_len,
				   0, 0, NULL, NULL);
		if(err != ERR_MEM) {
			b1_net_count(err == ERR_OK);
			hb_len = 0;
		}
	}

	for(n = 0; n < B1_PUBLISH_PER_POLL; n++) {
		if(!pend_valid) {
			if(!b1_net_take(&pend_kind, pend_body, &pend_len)) {
				break;
			}
			pend_valid = 1;
		}
		err = mqtt_publish(client,
				   (pend_kind == B1_OUT_ACK)? B1_TOPIC_ACK: B1_TOPIC_TLM,
				   pend_body, (u16_t)pend_len, 0, 0, NULL, NULL);
		if(err == ERR_MEM) {
			break;			/* try again next poll */
		}
		b1_net_count(err == ERR_OK);
		pend_valid = 0;
	}
}

/*
 * Called by cyw43_utk_task about every 2 ms while the WiFi link is up
 * (the template's name is kept so cyw43_utk.c needs no change).
 */
void mqtt_utk_start(void)
{
	u32_t	now = sys_now();

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL || !ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
			tm_printf((UB*)"mqtt: no client or bad MQTT_BROKER_IP\n");
			client = NULL;
			return;
		}
		IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);
		retry_at = now;
	}

	/* Not called while the link was down: start a fresh session rather
	 * than wait for the keepalive to notice the dead TCP connection. */
	if(last_poll != 0 && (now - last_poll) > MQTT_UTK_GAP_MS &&
	   net_state != B1_NET_RECONNECT_WAIT) {
		tm_printf((UB*)"mqtt: WiFi link was down, reconnecting\n");
		mqtt_disconnect(client);
		attempts = 0;
		retry_at = now;
		set_state(B1_NET_RECONNECT_WAIT);
	}
	last_poll = now;

	switch(net_state) {
	case B1_NET_DISCONNECTED:
	case B1_NET_RECONNECT_WAIT:
		if((s32_t)(now - retry_at) >= 0) {
			start_connect(now);
		}
		break;
	case B1_NET_CONNECTING:
		if((now - connect_started) > B1_CONNECT_TIMEOUT_MS) {
			tm_printf((UB*)"mqtt: connect timed out\n");
			mqtt_disconnect(client);
			wait_and_retry(now);
		}
		break;
	case B1_NET_OPERATIONAL:
		if(!mqtt_client_is_connected(client)) {
			wait_and_retry(now);
		} else {
			pump(now);
		}
		break;
	default:
		break;
	}
}
