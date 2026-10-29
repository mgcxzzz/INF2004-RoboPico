#ifndef TEST_LWIP_IP_ADDR_H
#define TEST_LWIP_IP_ADDR_H
#include <stdint.h>
typedef struct { uint32_t addr; } ip4_addr_t;
typedef struct { ip4_addr_t ip4; int type; } ip_addr_t;
#define IPADDR_TYPE_V4 0
#define ip_2_ip4(ip) (&(ip)->ip4)
#define IP_SET_TYPE(ip, value) ((ip)->type = (value))
int ip4addr_aton(const char *text, ip4_addr_t *address);
uint32_t lwip_ntohl(uint32_t value);
struct netif { ip4_addr_t ip_addr; };
const ip4_addr_t *netif_ip4_addr(const struct netif *netif);
#endif
