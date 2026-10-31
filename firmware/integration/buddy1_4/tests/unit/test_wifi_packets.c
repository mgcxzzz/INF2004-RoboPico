#include "src/network/wifi_packets.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned u16(const uint8_t *p) {
    return (unsigned)p[0] | ((unsigned)p[1] << 8);
}
static unsigned u32(const uint8_t *p) {
    return (unsigned)p[0] | ((unsigned)p[1] << 8) | ((unsigned)p[2] << 16) | ((unsigned)p[3] << 24);
}
int main(void) {
    uint8_t b[WIFI_PACKET_MAX], outlen;
    WifiHeartbeat hb = {0};
    WifiMotionPacket m = {0};
    WifiLinePacket l = {0};
    WifiNavPacket n = {0};
    WifiSummaryPacket s = {0};
    WifiDiagnosticsPacket d = {0};
    const uint8_t data[58] = {0};
    hb.seq = 255;
    hb.net = 2;
    hb.state = 7;
    hb.last_cmd = 9;
    hb.last_result = 5;
    hb.uptime_s = 16383;
    hb.published = 0x1234;
    hb.lost = 0xabcd;
    hb.accepted = 4;
    hb.rejected = 5;
    hb.connects = 6;
    hb.fill_pct = 100;
    assert(wpk_heartbeat(&hb, b));
    assert(b[0] == 0x12 && b[1] == 12 && b[2] == 255);
    assert(u32(b + 3) == (2u | (7u << 2) | (9u << 5) | (5u << 9) | (16383u << 12)));
    assert(u32(b + 7) == 0xabcd1234u && u32(b + 11) == 0x64060504u);
    m.seq = 7;
    m.uptime_ms = 0x78563412u;
    m.elapsed_ms = 70000;
    m.run_id = 0x1234;
    m.speed_left_0p1mm_s = -1;
    m.heading_cdeg = -18000;
    m.calibrated_level_left = 1000;
    m.calibrated_level_right = 900;
    m.distance_mm = 0x01020304u;
    m.adc_left = 11;
    m.adc_right = 22;
    m.adc_barcode = 33;
    assert(wpk_motion(&m, b));
    assert(b[0] == 0x92 && b[1] == 45 && b[2] == 7);
    assert(u32(b + 3) == 0x78563412u && u32(b + 7) == 70000u && u16(b + 11) == 0x1234u);
    assert(u16(b + 17) == 0xffffu && u16(b + 46) == (uint16_t)-18000 && u16(b + 39) == 0xffffu);
    assert(!wpk_motion(NULL, b));
    m.calibrated_level_left = 1001;
    assert(!wpk_motion(&m, b));
    m.calibrated_level_left = 0xffffu;
    assert(wpk_motion(&m, b));
    m.calibrated_level_left = 1000;
    m.heading_cdeg = 18001;
    assert(!wpk_motion(&m, b));
    m.heading_cdeg = -18000;
    l.seq = 4;
    l.uptime_ms = 100;
    l.adc_left = 1;
    l.adc_right = 2;
    l.adc_barcode = 3;
    l.calibrated_level_left = 500;
    l.calibrated_level_right = 600;
    l.last_decoded_letter = 'Z';
    l.pending_letter = 'A';
    l.pending_action = 'S';
    l.pending_revision = 8;
    l.executing_action = 'R';
    l.executing_revision = 7;
    assert(wpk_line(&l, b));
    assert(b[0] == 0xa2 && b[1] == 21 && b[2] == 4);
    assert(u16(b + 8) == 1 && u16(b + 10) == 2 && u16(b + 12) == 3 && u16(b + 14) == 500 &&
           b[18] == 'Z' && b[19] == 'A' && b[20] == 'S' && b[22] == 'R');
    l.pending_action = 'X';
    assert(!wpk_line(&l, b));
    l.pending_action = 'S';
    l.calibrated_level_left = 0xffffu;
    assert(wpk_line(&l, b));
    n.seq = 1;
    n.uptime_ms = 900;
    n.event = WPK_EVENT_DISPATCH;
    n.letter = 'A';
    n.action = 'S';
    n.map_revision = 4;
    n.command_id = 0x12345678;
    n.generation = 0x87654321;
    n.fault_bitmap = 0x03ff;
    n.owner = 2;
    n.mode = 3;
    n.command_state = 5;
    n.detail = 0xbeef;
    assert(wpk_nav(&n, b));
    assert(b[0] == 0xb2 && b[1] == 23 && b[9] == 'S' && u32(b + 11) == 0x12345678u &&
           u32(b + 15) == 0x87654321u && u16(b + 19) == 0x03ff && u16(b + 24) == 0xbeef);
    s.seq = 2;
    s.uptime_ms = 1;
    s.run_id = 2;
    s.result = 3;
    s.flags = 4;
    s.elapsed_ms = 0x12345678;
    s.distance_mm = 0x23456789;
    s.line_events = 5;
    s.nav_events = 6;
    s.recoveries = 7;
    assert(wpk_summary(&s, b));
    assert(b[0] == 0xc2 && b[1] == 22 && u32(b + 11) == 0x12345678u && u32(b + 15) == 0x23456789u &&
           u16(b + 23) == 7);
    d = (WifiDiagnosticsPacket){.seq = 0x5a,
                                .bank = 62,
                                .flags = 0x0f,
                                .uptime_ms = 0x78563412u,
                                .threshold_adc = 4095,
                                .decoded_total = 0x01020304u,
                                .turns = 0x11121314u,
                                .lefts = 0x21222324u,
                                .rights = 0x31323334u,
                                .uturns = 0x41424344u,
                                .bad_frames = 0x51525354u,
                                .unsupported = 0x61626364u,
                                .max_adc_gap_us = 0x71727374u,
                                .max_control_gap_us = 0x81828384u,
                                .tx_dropped = 0x91929394u,
                                .rx_dropped = 0xa1a2a3a4u,
                                .invalid_left = 0xb1b2b3b4u,
                                .invalid_right = 0xc1c2c3c4u};
    assert(wpk_diagnostics(&d, b));
    assert(b[0] == 0xe2 && b[1] == 60 && b[2] == 0x5a);
    assert(u32(b + 3) == 0x78563412u && u16(b + 7) == 4095 && b[9] == 62 && b[10] == 15);
    assert(u32(b + 11) == 0x01020304u && u32(b + 15) == 0x11121314u && u32(b + 19) == 0x21222324u);
    assert(u32(b + 23) == 0x31323334u && u32(b + 27) == 0x41424344u && u32(b + 31) == 0x51525354u);
    assert(u32(b + 35) == 0x61626364u && u32(b + 39) == 0x71727374u && u32(b + 43) == 0x81828384u);
    assert(u32(b + 47) == 0x91929394u && u32(b + 51) == 0xa1a2a3a4u && u32(b + 55) == 0xb1b2b3b4u &&
           u32(b + 59) == 0xc1c2c3c4u);
    assert(!wpk_diagnostics(NULL, b));
    d.threshold_adc = 4096;
    assert(!wpk_diagnostics(&d, b));
    d.threshold_adc = 4095;
    d.bank = 63;
    assert(!wpk_diagnostics(&d, b));
    d.bank = 62;
    d.flags = 0x10;
    assert(!wpk_diagnostics(&d, b));
    assert(wpk_log_chunk(1, 1, 9, 3, 4, 0, 2, 0, data, 49, b, &outlen));
    assert(outlen == 64 && b[0] == 0xd2 && b[1] == 61 && b[9] == 0 && b[10] == 2 && b[11] == 0 &&
           b[12] == 49 && b[13] == 0 && b[14] == 0);
    assert(wpk_log_chunk(2, 1, 9, 3, 4, 1, 2, 0, data, 9, b, &outlen) && outlen == 24);
    assert(!wpk_log_chunk(1, 1, 9, 3, 4, 2, 2, 0, data, 9, b, &outlen));
    assert(!wpk_log_chunk(1, 1, 9, 3, 4, 0, 2, 0, data, 50, b, &outlen));
    assert(!wpk_log_chunk(1, 4, 9, 3, 4, 0, 1, 0, data, 9, b, &outlen));
    assert(!wpk_log_chunk(1, 1, 9, 4096, 4097, 0, 1, 0, data, 9, b, &outlen));
    assert(wpk_log_chunk(1, 1, 9, 3, 0, 0, 1, WPK_LOG_STATUS_BUSY, NULL, 0, b, &outlen));
    (void)memset(b, 0, sizeof b);
    puts("wifi packet tests passed");
    return 0;
}
