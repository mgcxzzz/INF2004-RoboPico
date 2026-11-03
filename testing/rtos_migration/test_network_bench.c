#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_program/network_bridge/network_policy.h"
#include "lib/libwifi/sysdepend/pico_rp2040/network_settings.h"

int main(void) {
    uint8_t raw[8] = {0};
    uint32_t mapping = 0;
    WifiCommand cmd;
    for (unsigned code = WP_CMD_START; code <= WP_CMD_TURN; ++code) {
        raw[0] = (uint8_t)((code << 4) | WP_VERSION);
        RnCommandPlan p = rn_bench_command(raw, 4, 100, 110, &mapping, &cmd);
        if (code == WP_CMD_STOP || code == WP_CMD_PAUSE || code == WP_CMD_ESTOP) {
            assert(p.result == WP_ACK_ACCEPTED && (p.flags & RN_PLAN_STOP));
            assert(!!(p.flags & RN_PLAN_ESTOP) == (code == WP_CMD_ESTOP));
        } else if (code == WP_CMD_PING || code == WP_CMD_GET)
            assert(p.result == WP_ACK_ACCEPTED);
        else
            assert(p.result != WP_ACK_ACCEPTED && !p.flags);
    }
    raw[0] = (WP_CMD_MAP << 4) | WP_VERSION;
    uint32_t word = 0 | (5 << 5) | (24 << 10) | (25 << 15);
    raw[2] = (uint8_t)word;
    raw[3] = (uint8_t)(word >> 8);
    raw[4] = (uint8_t)(word >> 16);
    assert(rn_bench_command(raw, 5, 100, 110, &mapping, &cmd).flags == RN_PLAN_MAP);
    uint32_t saved = mapping;
    assert(rn_bench_command(raw, 5, 100, 601, &mapping, &cmd).result == WP_ACK_BUSY &&
           mapping == saved);
    assert(rn_bench_command(raw, 9, 100, 110, &mapping, &cmd).result == WP_ACK_FORMAT &&
           mapping == saved);
    assert(rn_bench_command(raw, 5, UINT32_MAX - 5, 5, &mapping, &cmd).result == WP_ACK_ACCEPTED);
    assert(mapping == saved); /* Idempotent mapping does not advance revision. */
    uint32_t last = 100;
    assert(rn_heartbeat_due(200, &last, true) && last == 100);
    assert(!rn_heartbeat_due(1099, &last, false));
    assert(rn_heartbeat_due(1100, &last, false) && last == 1100);
    last = UINT32_MAX - 500;
    assert(rn_heartbeat_due(500, &last, false) && last == 500);

    NetworkSettings config = {.ssid = "TEST-NET",
                              .password = "",
                              .broker = "192.168.1.2",
                              .client_id = "test"},
                    decoded;
    uint8_t page[256];
    memset(page, 0xff, sizeof(page));
    memcpy(page, "WCF1", 4);
    memcpy(page + 8, &config, sizeof(config));
    uint32_t crc = utk_network_settings_crc32(page + 8, 248);
    for (unsigned i = 0; i < 4; ++i)
        page[4 + i] = (uint8_t)(crc >> (i * 8));
    assert(utk_network_settings_decode(&decoded, page) && !strcmp(decoded.broker, config.broker));
    for (unsigned i = 0; i < 256; ++i) {
        page[i] ^= 1;
        assert(!utk_network_settings_decode(&decoded, page));
        page[i] ^= 1;
    }
    puts(
        "PASS network bench: Start/Resume/manual drive prohibited, stop plan, mapping/CRC, stale commands/wrap and independent heartbeat deadline.");
    return 0;
}
