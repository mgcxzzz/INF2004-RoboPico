#define APP_RTOS_RELEASE 1
#define APP_RTOS_BARCODE 1
#define APP_RTOS_NETWORK 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_program/rtos_bringup/bringup_policy.h"
#include "app_program/line_navigation/line_model.h"
#include "app_program/line_navigation/release_export.h"
#include "app_program/barcode_navigation/barcode_model.h"
#include "src/network/wifi_packets.h"
typedef unsigned UINT;
typedef struct { unsigned flags; } BringupStatus;
static unsigned depth, pushes;
static bool pending, dumping, dump_requested;
static uint32_t now_us = 1000000;
static BringupStatus hardware;
static RlExportLease wireless_export;
static LineLogRow rows[RL_LOG_ROWS];
static unsigned row_count, row_head;
static RbDecoder barcode;
static LineCalibration run_calibration;
static unsigned run_cal_origin;
static struct { LineCalibration cal; unsigned cal_origin; } view;
static WifiMotionPacket telemetry;
static uint16_t run_id;
static float release_run_centre;
static uint8_t packets[4][64], lengths[4], seq[16];
static UINT enter(void) { assert(!depth); depth=1; return 0; }
static void leave(UINT k) { (void)k; assert(depth); depth=0; }
static uint64_t board_time_us(void) { return now_us; }
static uint32_t board_kernel_ms(void) { return now_us/1000; }
static void bringup_get_status(BringupStatus *s) { assert(!depth); *s=hardware; }
static bool rtos_line_pending(void) { return pending; }
static uint8_t rn_next_sequence(unsigned source) { assert(source<16); return seq[source]++; }
static bool rn_tx_push(uint8_t topic, const uint8_t *p, uint8_t n, uint32_t now) {
    (void)topic;(void)now;assert(!depth && n<=64 && pushes<4);
    memcpy(packets[pushes],p,n);lengths[pushes++]=n;return true;
}
enum { RN_TLM=0 };
#include "app_program/line_navigation/release_exports.inc"
static uint64_t clock_now(void *u) { (void)u; return now_us; }
static unsigned reply(unsigned dataset,unsigned row) {
    WifiCommand cmd={.arg=(uint16_t)(dataset<<12|row),.id=77}; pushes=0;
    return rtos_line_log_reply(&cmd);
}
static void save(const char *name,const uint8_t *bytes,unsigned n) {
    char path[180]; snprintf(path,sizeof(path),"testing/rtos_migration/build-host/%s",name);
    FILE *f=fopen(path,"wb");assert(f);assert(fwrite(bytes,1,n,f)==n);fclose(f);
}
int main(void) {
    MotionPort port={.now_us=clock_now};motion_init(&port);motion_session_begin(now_us);
    run_id=telemetry.run_id=7;row_count=row_head=2;
    run_calibration=(LineCalibration){.white={122,138},.black={246,267},.mask=7};
    run_cal_origin=2; view.cal=run_calibration;
    rows[0]=(LineLogRow){.ms=123,.sequence=42,.ticks={-100,100},.speed={-123,456},
        .target={45,44},.pwm={200,210},.dt_us=10000,.cost_us=6000,
        .optics={122U|(138U<<12)|(232U<<24),3U|(100U<<4)|(200U<<14)|(3U<<24)},
        .meta=24U|(2U<<6)|(1U<<12)|(1U<<16)|(BR_ACTIVE<<24),
        .aux={111U|(222U<<12)|(188U<<24),2U|(500U<<2)|(1U<<14)|(1U<<17)|(8190U<<18)},
        .optical_target={43,44}};
    rows[1]=rows[0];rows[1].ms=133;rows[1].sequence=43;
    uint8_t info[15],line[56];rtos_line_release_info(9,info);save("release_metadata.bin",info,15);
    assert(info[0]==0xf2 && info[1]==12 && info[3]==0xc2 && info[5]==0x73);
    assert(reply(1,0)==WP_ACK_ACCEPTED && pushes==2 && rl_export_busy(&wireless_export,now_us));
    assert(lengths[0]==64 && lengths[1]==22 && packets[0][10]==2);
    memcpy(line,packets[0]+15,49);memcpy(line+49,packets[1]+15,7);
    save("release_line.bin",line,56);assert(line[0]==123 && line[40]==42);
    assert(reply(1,1)==WP_ACK_ACCEPTED && !rl_export_busy(&wireless_export,now_us));
    assert(reply(1,1)==WP_ACK_BUSY);
    assert(reply(1,0)==WP_ACK_ACCEPTED);++run_id;
    assert(reply(1,1)==WP_ACK_BUSY);--run_id;
    now_us+=5000001;assert(reply(1,1)==WP_ACK_BUSY);
    assert(reply(0,0)==WP_ACK_ACCEPTED && pushes==1 && packets[0][15]==0x92);
    assert(packets[0][15+15]&0x80); /* run-relative stored uptime */
    barcode.raw_count=barcode.raw_head=1;
    rb_put32(barcode.raw[0].bytes,77);rb_put32(barcode.raw[0].bytes+4,12345);
    barcode.raw[0].bytes[8]=0xff;barcode.raw[0].bytes[9]=0x0f;
    assert(reply(2,0)==WP_ACK_ACCEPTED && pushes==1 && lengths[0]==25);
    assert(memcmp(packets[0]+15,barcode.raw[0].bytes,10)==0);
    motion_note_navigation(1,0);motion_note_navigation(2,0);
    assert(motion_log_count()>1);assert(reply(3,0)==WP_ACK_ACCEPTED);
    motion_note_navigation(3,0);assert(reply(3,1)==WP_ACK_BUSY);
    hardware.flags=BR_ACTIVE;assert(reply(1,0)==WP_ACK_BUSY);hardware.flags=0;
    pending=true;assert(reply(1,0)==WP_ACK_BUSY);pending=false;
    dumping=true;assert(reply(1,0)==WP_ACK_BUSY);dumping=false;
    assert(reply(4,0)==WP_ACK_FORMAT);
    row_count=0;assert(reply(0,0)==WP_ACK_ACCEPTED && packets[0][11]==WPK_LOG_STATUS_UNAVAILABLE);
    RlExportLease lease;
    rl_export_open(&lease,1,2,7,0,UINT32_MAX-4999999U);
    assert(lease.until_us==0 && rl_export_busy(&lease,UINT32_MAX-1U));
    assert(!rl_export_busy(&lease,1));
    puts("PASS release exports: stopped gate, frozen lease, row formats, chunk bounds, navigation generation, empty data and timer wrap.");
}
