/* Host integration coverage for the actual core0 Wi-Fi harness. Hardware and
 * transport are narrow fakes; coordinator, executor, codec and wire packers
 * are the production implementations. */
#define _CRT_SECURE_NO_WARNINGS
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_WIFI_LOGGING 1
#define CIRCUIT_SWEEP_SEARCH 1
#define LINE_CAL_MIN_SPAN 50
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "v12_2/baseline/testing/subsystem2/left_motor_bench/bench_logic.h"
#undef REST_US /* old bench header and Buddy 1 guard used the same macro */
#include "v12_2/baseline/testing/subsystem2/dual_wheel_bench/dual_logic.h"
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h"
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h"
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_raw.h"
#include "drive_diag.h"
#include "motion_api.h"
#include "motion_port.h"
#include "nav_api.h"
#include "nav_coordinator.h"
#include "nav_log_storage.h"
#include "wifi_protocol.h"
#include "wifi_packets.h"
#include "wifi_transport.h"

enum { HOLD=21, STOP=20, FLASH_SECTOR_SIZE=4096, DIAG_OFFSET=0x1ab000,
       PICO_FLASH_SIZE_BYTES=0x200000,
       INTERFACE_LOG_OFFSET=0x1f0000, PICO_ERROR_TIMEOUT=-1 };
#define XIP_BASE ((uintptr_t)0x1000u) /* Binary end is above this fake XIP window. */
extern char __flash_binary_end;
char __flash_binary_end;

static uint64_t fake_now;
static bool button_stop_released=true,button_hold_pressed=true,usb_connected;
static unsigned output_off_calls,flash_write_calls,transport_flash_begins;
static bool fake_configured=true,fake_ready=true,fake_connected=true,fake_subscribed=true;
static uint32_t fake_reconnects=1;
typedef struct { unsigned topic;size_t len;uint8_t bytes[64]; } Sent;
static Sent sent[256];static unsigned sent_count;
static uint8_t rx[16][8];static size_t rx_len[16];static unsigned rx_head,rx_count;

uint32_t save_and_disable_interrupts(void){return 0;}
void restore_interrupts(uint32_t state){(void)state;}
bool gpio_get(unsigned pin){return pin==STOP?button_stop_released:pin==HOLD?button_hold_pressed:true;}
bool stdio_usb_connected(void){return usb_connected;}
int64_t path_mm(const int64_t ticks[2]){(void)ticks;return 42;}
static double wheel_mm(unsigned wheel,int64_t ticks){return wheel?(double)ticks:-(double)ticks;}
void outputs_off(void){++output_off_calls;}
void flash_range_erase(uint32_t offset,size_t count){(void)offset;(void)count;++flash_write_calls;}
void flash_range_program(uint32_t offset,const uint8_t *p,size_t count){(void)offset;(void)p;(void)count;++flash_write_calls;}
void stdio_flush(void){}
void sleep_ms(unsigned ms){(void)ms;}
void watchdog_reboot(unsigned a,unsigned b,unsigned c){(void)a;(void)b;(void)c;}

/* Firmware context consumed by wifi_harness.h. These are real ABI types from
 * the Buddy 1 headers, not copies of the harness's behavior. */
static volatile DualGuard guard={.consumed=true};
static volatile BenchEncoder enc[2];
static volatile int64_t total_ticks[2],cut_ticks[2];
static volatile uint32_t initial_invalid[2],cut_invalid[2];
static volatile int32_t output[2];
static volatile float velocity[2];
static volatile uint16_t analog_raw[2];
static volatile uint16_t side_adc;
static volatile AnalogDemand demand;
static LineCalibration calibration;
static Circuit circuit;
static BarcodeNav nav;
static BarcodeAnalog side_analog;
static BarcodeTrace trace[B_ROWS];
static uint32_t trace_count,trace_head,runs;
static BarcodeRaw raw_capture;
static DriveDiag diag_capture;
static bool trace_settling;
static uint8_t test_mode=1,recorded_mode=1;
static char recorded_key;
static CircuitTracker fake_tracker;
static const CircuitTracker *current_tracker(void){return &fake_tracker;}
static uint32_t wifi_mapping;
static bool wifi_start_pending,wifi_estop,wifi_stopped;
static void kill(uint64_t now,uint8_t reason){guard_stop(&guard,now,reason);outputs_off();motion_session_end(reason);}
static unsigned start_invocations,start_block_reason;
static bool allow_start=true;
static uint64_t port_now(void *u){(void)u;return fake_now;}
static void port_off(void *u){(void)u;outputs_off();}
static uintptr_t port_lock(void *u){(void)u;return 0;}
static void port_unlock(void *u,uintptr_t s){(void)u;(void)s;}
static MotionPort port={port_lock,port_unlock,port_off,port_now,NULL};
static bool start_run(unsigned mode){
    (void)mode;++start_invocations;if(!allow_start)return false;
    guard.active=true;guard.start_us=fake_now;guard.consumed=true;++runs;
    motion_session_begin(fake_now);return true;
}
static unsigned start_block(unsigned mode){(void)mode;return start_block_reason;}

bool wifi_transport_configure(const WifiTransportConfig *c){(void)c;return true;}
void wifi_transport_init(void){}
bool wifi_transport_send(unsigned topic,const uint8_t *p,size_t n){
    if(sent_count>=sizeof(sent)/sizeof(sent[0]) || n>64)return false;
    sent[sent_count].topic=topic;sent[sent_count].len=n;memcpy(sent[sent_count].bytes,p,n);++sent_count;return true;
}
bool wifi_transport_receive(uint8_t p[8],size_t *n){
    if(!rx_count)return false;
    memcpy(p,rx[rx_head],8);*n=rx_len[rx_head];rx_head=(rx_head+1)%16;--rx_count;return true;
}
WifiTransportStatus wifi_transport_status(void){return (WifiTransportStatus){
    .configured=fake_configured,.ready=fake_ready,.connected=fake_connected,.subscribed=fake_subscribed,
    .published=sent_count,.reconnects=fake_reconnects};}
bool wifi_transport_flash_begin(void){++transport_flash_begins;return true;}
void wifi_transport_flash_end(void){}

#define PICO_FLASH_SIZE_BYTES 0x200000u
#include "wifi_harness.h"

static void queue_command(uint8_t code,uint8_t id,uint16_t arg,uint32_t mapping){
    uint8_t p[8]={0};p[0]=(uint8_t)((code<<4)|WP_VERSION);p[1]=id;
    if(code==WP_CMD_MAP){p[2]=(uint8_t)mapping;p[3]=(uint8_t)(mapping>>8);p[4]=(uint8_t)(mapping>>16);}
    else {p[2]=(uint8_t)arg;p[3]=(uint8_t)(arg>>8);}
    unsigned i=(rx_head+rx_count)%16;memcpy(rx[i],p,8);rx_len[i]=code==WP_CMD_MAP?WP_MAP_CMD_BYTES:WP_CMD_BYTES;++rx_count;
}
static const Sent *last_src(unsigned src){
    for(unsigned k=sent_count;k>0;--k) {
        if(sent[k-1].len>=3 && (sent[k-1].bytes[0]&15u)==WP_VERSION &&
           (sent[k-1].bytes[0]>>4)==src)return &sent[k-1];
    }
    return NULL;
}
static const Sent *last_map(void){
    for(unsigned k=sent_count;k>0;--k)
        if(sent[k-1].len==8 && (sent[k-1].bytes[0]>>4)==WPK_SRC_MAP)return &sent[k-1];
    return NULL;
}
static void reset_capture(void){sent_count=0;rx_head=rx_count=0;}
static uint16_t rd16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t map_for(char letter,char action){
    static const char slots[]="SULR";const char *p=strchr(slots,action);assert(p);
    unsigned target=(unsigned)(p-slots),i=(unsigned)(letter-'A');uint32_t word=0;
    for(unsigned s=0;s<4;++s)word|=((i+s+26-target)%26)<<(s*5);
    return word;
}

static void test_map_during_run(void){
    nav=(BarcodeNav){.pending='A',.pending_action='L',.pending_revision=7,.mapping=wifi_mapping};
    guard.active=true;wifi_mapping=wp_map_replace(0,map_for('A','L'))|(6u<<WP_MAP_REV_SHIFT);
    nav.mapping=wifi_mapping;uint8_t oldrev=nav.pending_revision;char oldaction=nav.pending_action;
    uint32_t next=map_for('A','R');
    reset_capture();queue_command(WP_CMD_MAP,41,0,next);wifi_harness_poll(1000);
    assert(nav.mapping==wifi_mapping && (wifi_mapping>>21)==8);
    assert(nav.pending=='A' && nav.pending_action==oldaction && nav.pending_revision==oldrev);
    const Sent *map=last_map();assert(map && map->bytes[7]==41);
}

static void test_start_countdown_cancellation(void){
    guard.active=false;wifi_estop=false;wifi_stopped=false;start_invocations=0;start_block_reason=0;
    fake_now=10000000;reset_capture();queue_command(WP_CMD_START,1,2,0);wifi_harness_poll(fake_now);
    assert(start_invocations==0 && wifi_start_pending);
    wifi_harness_poll(fake_now+1999999);assert(start_invocations==0 && wifi_start_pending);
    wifi_harness_poll(fake_now+2000000);assert(start_invocations==1 && !wifi_start_pending);
    guard.active=false;wifi_was_active=false;wifi_start_pending=false;wifi_estop=false;start_invocations=0;
    fake_now+=3000000;queue_command(WP_CMD_START,2,0,0);wifi_harness_poll(fake_now);assert(wifi_start_pending);
    button_hold_pressed=false;wifi_harness_poll(fake_now+1000);assert(!wifi_start_pending && start_invocations==0);
    button_hold_pressed=true;queue_command(WP_CMD_START,3,0,0);wifi_harness_poll(fake_now+2000);assert(wifi_start_pending);
    queue_command(WP_CMD_ESTOP,4,0,0);wifi_harness_poll(fake_now+3000);assert(!wifi_start_pending && wifi_estop);
}

static void test_pause_resume_coordinator(void){
    fake_now=20000000;guard.active=true;wifi_estop=false;wifi_lease=0;nav=(BarcodeNav){0};
    motion_session_begin(fake_now);uint8_t p[8]={0};reset_capture();queue_command(WP_CMD_PAUSE,9,0,0);wifi_harness_poll(fake_now);
    MotionStatus s=motion_get_status();assert(s.mode==MOTION_STOPPING && s.owner==MOTION_BUDDY1);
    MotionFeedback f={.now_us=fake_now,.line_level={500,100},.speed_mm_s={0,0}};
    MotionOutput out=motion_update(&f);assert(out.external && out.target_mm_s[0]==0 && out.target_mm_s[1]==0);
    for(unsigned k=1;k<=8;++k){fake_now+=20000;f.now_us=fake_now;f.line_level[0]=500;(void)motion_update(&f);}
    assert(motion_get_status().mode==MOTION_PAUSED);
    for(unsigned k=0;k<8;++k){fake_now+=20000;f.now_us=fake_now;f.line_level[0]=500;(void)motion_update(&f);}
    fake_now+=10000;f.now_us=fake_now;(void)motion_update(&f);
    reset_capture();queue_command(WP_CMD_RESUME,10,0,0);wifi_harness_poll(fake_now);
    assert(motion_get_status().mode==MOTION_PAUSED);
    fake_now+=10000;f.now_us=fake_now;out=motion_update(&f);assert(out.resume_navigation && !out.external);
    (void)p;
}

static void test_stop_estop_reset(void){
    guard.active=true;wifi_estop=false;wifi_stopped=false;motion_session_begin(fake_now);
    reset_capture();queue_command(WP_CMD_STOP,11,0,0);wifi_harness_poll(fake_now);
    assert(!guard.active && !wifi_estop && wifi_stopped);
    guard.active=true;motion_session_begin(fake_now+500);
    queue_command(WP_CMD_ESTOP,12,0,0);wifi_harness_poll(fake_now+500);
    assert(!guard.active && wifi_estop && wifi_stopped);
    guard.active=true;queue_command(WP_CMD_RESET,13,0,0);wifi_harness_poll(fake_now+1000);assert(wifi_estop);
    guard.active=false;button_stop_released=false;button_hold_pressed=true;
    queue_command(WP_CMD_RESET,14,0,0);wifi_harness_poll(fake_now+1500);assert(wifi_estop);
    button_stop_released=true;button_hold_pressed=true;trace_settling=false;
    queue_command(WP_CMD_RESET,15,0,0);wifi_harness_poll(fake_now+2000);assert(!wifi_estop && !wifi_stopped);
}

static void test_downloads_and_recheck(void){
    fake_now=30000000;guard.active=false;trace_settling=false;diag_capture=(DriveDiag){0};
    diag_capture.count=1;diag_capture.head=1;for(unsigned i=0;i<DIAG_ROW_BYTES;++i)diag_capture.rows[0].bytes[i]=(uint8_t)(i+3);
    reset_capture();queue_command(WP_CMD_GET_LOG,23,(uint16_t)((WP_LOG_DIAGNOSTIC<<12)|0),0);wifi_harness_poll(fake_now);
    const Sent *first=NULL;
    for(unsigned i=0;i<sent_count;++i)if((sent[i].bytes[0]>>4)==WPK_SRC_LOG_CHUNK){first=&sent[i];break;}
    assert(first && first->len==64);
    const Sent *second=NULL;for(unsigned i=0;i<sent_count;++i)
        if((sent[i].bytes[0]>>4)==WPK_SRC_LOG_CHUNK)second=&sent[i];
    assert(second && second->len==24);
    assert(first->bytes[3]==WP_LOG_DIAGNOSTIC && first->bytes[4]==23 && rd16(first->bytes+5)==0);
    assert(first->bytes[9]==0 && first->bytes[10]==2 && first->bytes[11]==0 && first->bytes[12]==49);
    assert(first->bytes[15]==3 && first->bytes[63]==51);
    assert(second->bytes[9]==1 && second->bytes[10]==2 && second->bytes[11]==0 && second->bytes[12]==9);
    assert(memcmp(second->bytes+15,diag_capture.rows[0].bytes+49,9)==0);

    trace_count=trace_head=1;trace[0]=(BarcodeTrace){0};trace[0].motion.active=1;trace[0].motion.level[0]=600;trace[0].motion.level[1]=200;
    reset_capture();queue_command(WP_CMD_GET_LOG,24,(uint16_t)((WP_LOG_MOTION<<12)|0),0);wifi_harness_poll(fake_now+50000);
    first=last_src(WPK_SRC_LOG_CHUNK);
    assert(first && first->len==63 && first->bytes[12]==48 && first->bytes[15]==0x92);
    guard.active=false;WifiCommand pending=(WifiCommand){.code=WP_CMD_GET_LOG,.id=25,.arg=(WP_LOG_DIAGNOSTIC<<12)};
    assert(wifi_dispatch(&pending,fake_now)==WP_ACK_ACCEPTED && wifi_download);
    guard.active=true;reset_capture();wifi_pump_download();first=last_src(WPK_SRC_LOG_CHUNK);
    assert(first && first->len==15 && first->bytes[11]==WPK_LOG_STATUS_BUSY && first->bytes[12]==0);
}

static void test_calibration_summary_and_provision(void){
    calibration=(LineCalibration){0};guard.active=false;wifi_estop=false;analog_raw[0]=123;analog_raw[1]=456;
    WifiMotionPacket m=wifi_snapshot(50000000);assert(m.calibrated_level_left==65535 && m.calibrated_level_right==65535);
    assert((m.line_levels&3)==0);side_analog.black=true; m=wifi_snapshot(50000000);assert((m.line_levels&4)!=0);
    reset_capture();wifi_line_status(50000000,WPK_EVENT_STATUS);
    const Sent *line=last_src(WPK_SRC_LINE);assert(line && rd16(line->bytes+14)==65535 && rd16(line->bytes+16)==65535);
    guard.duration_us=2000000;runs=22;guard.reason=WHY_SERIAL;fake_subscribed=true;fake_reconnects=4;wifi_summary_run=0;wifi_summary_connection=0;
    reset_capture();wifi_harness_poll(50000000);unsigned summaries=0;
    for(unsigned i=0;i<sent_count;++i)if((sent[i].bytes[0]>>4)==WPK_SRC_SUMMARY)++summaries;
    assert(summaries==1);wifi_harness_poll(50010000);unsigned again=0;
    for(unsigned i=0;i<sent_count;++i)if((sent[i].bytes[0]>>4)==WPK_SRC_SUMMARY)++again;assert(again==1);
    ++fake_reconnects;wifi_harness_poll(50020000);again=0;
    for(unsigned i=0;i<sent_count;++i)if((sent[i].bytes[0]>>4)==WPK_SRC_SUMMARY)++again;assert(again==2);

    guard.active=true;unsigned writes=flash_write_calls,begins=transport_flash_begins;
    const char *provision="!net|ssid|password|192.168.1.5\n";for(const char *q=provision;*q;++q)(void)wifi_console_byte(*q);
    assert(flash_write_calls==writes && transport_flash_begins==begins);
    assert(wifi_setup_last_code==WIFI_SETUP_ACTIVE);
    guard.active=false;trace_settling=true;
    for(const char *q=provision;*q;++q)(void)wifi_console_byte(*q);
    assert(wifi_setup_last_code==WIFI_SETUP_SETTLING);
    trace_settling=false;button_stop_released=false;
    for(const char *q=provision;*q;++q)(void)wifi_console_byte(*q);
    assert(wifi_setup_last_code==WIFI_SETUP_STOP_HELD);
    button_stop_released=true;button_hold_pressed=false;
    for(const char *q=provision;*q;++q)(void)wifi_console_byte(*q);
    assert(wifi_setup_last_code==WIFI_SETUP_HOLD_HELD);
    button_hold_pressed=true;
    for(const char *q=provision;*q;++q)(void)wifi_console_byte(*q);
    assert(wifi_setup_last_code==WIFI_SETUP_OVERLAP); /* host has no real flash window */
    assert(flash_write_calls==writes && transport_flash_begins==begins);
    guard.active=false;
}


/* Exercise the real periodic producer through the real packer, including the
 * appended bank. The reserved flags regression previously suppressed all diagnostics. */
static void test_fine_periodic_reporting(void){
    guard.active=false;wifi_forward=true;fake_configured=fake_ready=fake_connected=fake_subscribed=true;
    for(unsigned i=0;i<2;++i){
        side_analog.selected=i?63:11;wifi_last_hb=0;reset_capture();
        wifi_harness_poll(60000000u+i*2000000u);
        const Sent *d=last_src(WPK_SRC_DIAGNOSTICS),*n=last_src(WPK_SRC_NAV);
        assert(d && d->len==63 && d->bytes[9]==side_analog.selected && !(d->bytes[10]&0xf0));
        assert(rd16(d->bytes+7)==(i?204:212));
        assert(n && n->len==26 && n->bytes[7]==WPK_EVENT_STATUS && rd16(n->bytes+24)==0xbc02);
        printf("FINE_REPORT_VECTOR ");for(size_t k=0;k<d->len;++k)printf("%02x",d->bytes[k]);puts("");
        printf("FINE_REPORT_VECTOR ");for(size_t k=0;k<n->len;++k)printf("%02x",n->bytes[k]);puts("");
    }
}

int main(void){
    motion_init(&port);motion_session_end(0);guard.active=false;guard.duration_us=0;wifi_mapping=0;wifi_harness_init();
    MotionEvent e={.type=MOTION_EVENT_NAV_STATE,.now_us=1};wifi_nav_event(&e);wifi_barcode_events(1);
    test_map_during_run();test_start_countdown_cancellation();test_pause_resume_coordinator();
    test_stop_estop_reset();test_downloads_and_recheck();test_calibration_summary_and_provision();test_fine_periodic_reporting();
    puts("PASS actual Wi-Fi harness integration with coordinator, packets and fake transport");return 0;
}
