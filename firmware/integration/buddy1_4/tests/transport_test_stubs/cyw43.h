#ifndef TEST_CYW43_H
#define TEST_CYW43_H
#include "tests/transport_test_stubs/lwip/ip_addr.h"
typedef struct {
    struct netif netif[1];
} cyw43_t;
extern cyw43_t cyw43_state;
#define CYW43_ITF_STA 0
#define CYW43_AUTH_OPEN 0
#define CYW43_AUTH_WPA2_AES_PSK 1
#define CYW43_NONE_PM 0
#define CYW43_LINK_DOWN 0
#define CYW43_LINK_JOIN 1
#define CYW43_LINK_NOIP 2
#define CYW43_LINK_UP 3
int cyw43_wifi_pm(cyw43_t *, uint32_t);
int cyw43_wifi_get_pm(cyw43_t *, uint32_t *);
int cyw43_tcpip_link_status(cyw43_t *, int);
#endif
