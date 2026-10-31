#ifndef BUDDY_WIFI_LWIPOPTS_H
#define BUDDY_WIFI_LWIPOPTS_H

#define NO_SYS 1
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0
#define LWIP_RAW 0
#define LWIP_DHCP 1
#define LWIP_DNS 1
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_MQTT 1
#define MEM_ALIGNMENT 4
#define MEM_SIZE 12000
/* lwIP's default only budgets core timers. The MQTT cyclic timer is an
 * application timer, started after TCP connects; reserve bounded headroom. */
#define MEMP_NUM_SYS_TIMEOUT (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)
#define MEMP_NUM_TCP_SEG 32
#define MEMP_NUM_TCP_PCB 8
#define MEMP_NUM_TCP_PCB_LISTEN 4
#define MEMP_NUM_NETCONN 0
#define TCP_MSS 1460
#define TCP_WND (4 * TCP_MSS)
#define TCP_SND_BUF (4 * TCP_MSS)
#define TCP_SND_QUEUELEN ((4 * TCP_SND_BUF) / TCP_MSS)
#define LWIP_NETIF_STATUS_CALLBACK 1
#define LWIP_NETIF_LINK_CALLBACK 1
#define LWIP_NETIF_HOSTNAME 1
#define LWIP_STATS 0
#define LWIP_DEBUG 0

#endif
