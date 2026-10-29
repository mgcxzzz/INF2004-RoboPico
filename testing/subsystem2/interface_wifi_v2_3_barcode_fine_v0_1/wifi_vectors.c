#include "wifi_packets.h"
#include "wifi_protocol.h"
#include <stdio.h>

static void print_bytes(const uint8_t *p, unsigned n)
{
    unsigned i;
    putchar('[');
    for (i=0u;i<n;++i) printf("%s%u",i?",":"",(unsigned)p[i]);
    putchar(']');
}
static uint32_t map_word(void)
{
    return 18u | (0u<<5) | (12u<<10) | (16u<<15); /* S/A/M/Q */
}
int main(void)
{
    uint8_t out[WIFI_PACKET_MAX],ack[4],log_len;WifiCommand command={0};
    WifiHeartbeat hb={.seq=201,.net=2,.state=1,.last_cmd=11,.last_result=0,
        .uptime_s=16383,.published=1234,.lost=4,.accepted=8,.rejected=1,.connects=3,.fill_pct=75};
    WifiMotionPacket motion={.seq=9,.uptime_ms=0x12345678u,.elapsed_ms=70000u,.run_id=42,
        .phase=4,.profile=2,.flags=7,.recovery_state=1,.line_levels=3,
        .distance_mm=123456u,.heading_cdeg=-18000,.adc_left=101,.adc_right=202,.adc_barcode=303,
        .calibrated_level_left=400,.calibrated_level_right=800,.speed_left_0p1mm_s=-25,
        .speed_right_0p1mm_s=75,.target_left_mm_s=45,.target_right_mm_s=50,.pwm_left=111,.pwm_right=222};
    WifiLinePacket line={.seq=10,.event=WPK_EVENT_QUEUED,.uptime_ms=999,.last_decoded_letter='Z',
        .pending_letter='A',.pending_action='S',.pending_revision=4,.executing_action='R',.executing_revision=3,
        .adc_left=1001,.adc_right=1002,.adc_barcode=1003,.calibrated_level_left=450,.calibrated_level_right=550};
    WifiNavPacket nav={.seq=11,.event=WPK_EVENT_DISPATCH,.letter='A',.action='S',.map_revision=4,
        .owner=3,.mode=1,.command_state=1,.uptime_ms=1001,.command_id=0x12345678u,
        .generation=0x10203040u,.fault_bitmap=2,.detail=99};
    WifiSummaryPacket summary={.seq=12,.result=3,.flags=1,.uptime_ms=1200,.elapsed_ms=98765,
        .distance_mm=654321,.run_id=42,.line_events=7,.nav_events=8,.recoveries=2};
    WifiDiagnosticsPacket diagnostics={.seq=14,.uptime_ms=0x78563412u,.threshold_adc=2048,
        .bank=31,.flags=0x0b,.decoded_total=0x01020304u,.turns=0x11121314u,
        .lefts=0x21222324u,.rights=0x31323334u,.uturns=0x41424344u,
        .bad_frames=0x51525354u,.unsupported=0x61626364u,.max_adc_gap_us=0x71727374u,
        .max_control_gap_us=0x81828384u,.tx_dropped=0x91929394u,.rx_dropped=0xa1a2a3a4u,
        .invalid_left=0xb1b2b3b4u,.invalid_right=0xc1c2c3c4u};
    uint8_t log_data[3]={0x11,0x22,0x33};
    if(!wpk_heartbeat(&hb,out)) return 1;
    printf("{\"heartbeat\":");print_bytes(out,WPK_HEARTBEAT_BYTES);
    if(!wpk_motion(&motion,out)) return 2;
    printf(",\"motion\":");print_bytes(out,WPK_MOTION_BYTES);
    if(!wpk_line(&line,out)) return 3;
    printf(",\"line\":");print_bytes(out,WPK_LINE_BYTES);
    if(!wpk_nav(&nav,out)) return 4;
    printf(",\"nav\":");print_bytes(out,WPK_NAV_BYTES);
    if(!wpk_summary(&summary,out)) return 5;
    printf(",\"summary\":");print_bytes(out,WPK_SUMMARY_BYTES);
    if(!wpk_diagnostics(&diagnostics,out)) return 7;
    printf(",\"diagnostics\":");print_bytes(out,WPK_DIAGNOSTICS_BYTES);
    if(!wpk_log_chunk(13u,1u,0x4au,3u,4u,0u,2u,WPK_LOG_STATUS_OK,
                      log_data,3u,out,&log_len)) return 6;
    printf(",\"log\":");print_bytes(out,log_len);
    command.byte0=(uint8_t)((WP_CMD_GET_LOG<<4)|WP_VERSION);command.code=WP_CMD_GET_LOG;command.id=0x4au;
    wp_ack(&command,WP_ACK_ACCEPTED,3u,ack);
    printf(",\"ack\":");print_bytes(ack,sizeof(ack));
    wp_map_frame(wp_map_replace(0u,map_word()),0x4au,14u,out);
    printf(",\"map\":");print_bytes(out,WP_MAP_FRAME_BYTES);
    puts("}");
    return 0;
}
