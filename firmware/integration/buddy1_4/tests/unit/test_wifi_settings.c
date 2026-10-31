#include <assert.h>
#include <stdio.h>
#include "src/network/wifi_settings.h"
int main(void) {
    WifiTransportConfig a = {.ssid = "TEST-NET",
                             .password = "",
                             .broker = "192.168.1.12",
                             .client_id = "is28-test"},
                        b;
    uint8_t page[256];
    assert(wifi_settings_encode(page, &a));
    assert(wifi_settings_decode(&b, page));
    assert(!memcmp(&a, &b, sizeof(a)));
    page[180] ^= 1;
    assert(!wifi_settings_decode(&b, page));
    assert(!wifi_ipv4("1.2.3.256") && !wifi_ipv4("1.2.3") && !wifi_ipv4("1.2..3"));
    assert(!wifi_ipv4("1.2.3.4.5") && wifi_ipv4("10.0.0.1"));
    memset(a.ssid, 'x', sizeof(a.ssid));
    assert(!wifi_settings_encode(page, &a));
    puts("PASS runtime settings CRC, bounds, IPv4 validation and open-network password");
    return 0;
}
