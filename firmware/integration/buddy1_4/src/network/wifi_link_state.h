/**
 * @file wifi_link_state.h
 * @brief Wifi link state.
 * @details Network helpers validate and explicitly encode bounded application
 * records. Transport calls belong to the configured SDK or RTOS owner, not
 * these codecs.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef WIFI_LINK_STATE_H
#define WIFI_LINK_STATE_H
/* Called only by the core1 network owner after cyw43_arch_poll().
 * The radio-only cyw43_wifi_link_status reports JOIN, never IP-ready UP.
 * MQTT requires the TCP/IP query, which also checks DHCP/netif readiness. */
static inline int wifi_network_link_status (void)
{
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
}
#endif

/* End of file. */
