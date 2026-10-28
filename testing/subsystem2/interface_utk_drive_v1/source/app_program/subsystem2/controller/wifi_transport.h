#ifndef WIFI_TRANSPORT_H
#define WIFI_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "wifi_transport_status.h"

#define WIFI_PACKET_MAX 64u

typedef struct {
    char ssid[33];
    char password[65];
    char broker[16];
    char client_id[32];
} WifiTransportConfig;

/* Runtime serial configuration is disabled; the service reads BOOTSEL-loaded WCF1 settings. */
bool wifi_transport_configure(const WifiTransportConfig *config);
/* Initialize the controller-side bounded bridge adapter. */
void wifi_transport_init(void);
/* Copies a packet into a bounded queue; never waits for the network. */
bool wifi_transport_send(unsigned topic, const uint8_t *data, size_t len);
/* Copies at most eight bytes; original_len reports the complete MQTT payload size. */
bool wifi_transport_receive(uint8_t data[8], size_t *original_len);
WifiTransportStatus wifi_transport_status(void);
/* Call on core0 before disabling IRQs for flash operations. */
bool wifi_transport_flash_begin(void);
/* Call on core0 after IRQs have been restored. */
void wifi_transport_flash_end(void);

#endif
