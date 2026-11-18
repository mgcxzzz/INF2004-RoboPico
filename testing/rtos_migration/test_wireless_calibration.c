/* Exercise the real staged model/codec and control-owner adapter, no hardware. */
#define LINE_CAL_MIN_SPAN 50
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_program/rtos_bringup/bringup_policy.h"
#include "app_program/line_navigation/line_model.h"
#include "app_program/line_navigation/wireless_calibration.h"

typedef unsigned UINT;
typedef struct { unsigned flags; } bringup_status_t;
static unsigned depth, writes, buttons, g_bringup_irq_stop;
static uint32_t clock_ms=1000, g_cancel_serial;
static bool allowed=true, save_ok=true, g_adc_ok=true, g_filter_ready=true;
static uint16_t g_adc[3]={120,200,180};
static unsigned g_cal_origin=1;
static line_calibration_t g_calibration_value={.white={122,138},.black={246,267},.mask=7};
static rl_cal_session_t g_wireless_cal;
static bringup_status_t hardware;
static UINT enter(void) { assert(!depth);depth=1;return 0; }
static void leave(UINT key) { (void)key;assert(depth);depth=0; }
static void bringup_get_status(bringup_status_t *out) { assert(!depth);*out=hardware; }
static bool rtos_line_mode_change_allowed(void) { return allowed; }
static bool board_button_pressed(unsigned pin) { return (buttons&(1U<<(pin-20U)))!=0; }
static uint32_t board_kernel_ms(void) { return clock_ms; }
static bool board_line_save(const uint8_t page[256]) {
    assert(!depth && allowed && !g_bringup_irq_stop && !buttons);
    line_calibration_t recovered;
    assert(rl_cal_decode(page,&recovered));
    assert(recovered.white[0]==120 && recovered.white[1]==180);
    assert(recovered.black[0]==450 && recovered.black[1]==650);
    ++writes;return save_ok;
}
#include "app_program/line_navigation/wireless_calibration.inc"

static uint8_t command(unsigned op) {
    static uint8_t id=1;
    return rtos_line_cal_command(op,id++);
}
static void ticks(unsigned n,unsigned l,unsigned r) {
    g_adc[0]=(uint16_t)l;g_adc[2]=(uint16_t)r;
    for(unsigned i=0;i<n;i++) {clock_ms+=10;wireless_cal_tick();}
}
static void capture(unsigned op,unsigned l,unsigned r) {
    assert(command(op)==WP_ACK_ACCEPTED);ticks(31,l,r);
    assert(g_wireless_cal.capture && g_wireless_cal.count==31);
    ticks(1,l,r);assert(!g_wireless_cal.capture);
    assert(g_wireless_cal.result==RL_CAL_CAPTURED);
}
static void complete_candidate(void) {
    assert(command(RL_CAL_BEGIN)==WP_ACK_ACCEPTED);
    assert(command(RL_CAL_LEFT)==WP_ACK_FORMAT);
    capture(RL_CAL_WHITE,120,180);
    capture(RL_CAL_LEFT,450,180);
    capture(RL_CAL_RIGHT,120,650);
    line_calibration_t c=rl_cal_candidate(&g_wireless_cal);
    assert(calibration_valid(&c));
}
int main(void) {
    line_calibration_t edge={.white={100,100},.black={150,149},.mask=7};
    assert(!calibration_valid(&edge));edge.black[1]=150;assert(calibration_valid(&edge));
    wireless_cal_tick();
    allowed=false;assert(command(RL_CAL_BEGIN)==WP_ACK_BUSY);allowed=true;
    buttons=1;assert(command(RL_CAL_BEGIN)==WP_ACK_BUSY);buttons=0;
    g_bringup_irq_stop=1;assert(command(RL_CAL_BEGIN)==WP_ACK_BUSY);g_bringup_irq_stop=0;
    hardware.flags=BR_ESTOP;assert(command(RL_CAL_BEGIN)==WP_ACK_BUSY);hardware.flags=0;
    g_adc_ok=false;assert(command(RL_CAL_BEGIN)==WP_ACK_BUSY);g_adc_ok=true;
    line_calibration_t original=g_calibration_value;
    complete_candidate();
    assert(!memcmp(&original,&g_calibration_value,sizeof(original)) && writes==0);
    /* A failed recapture cannot apply the old sample as if it were new. */
    assert(command(RL_CAL_LEFT)==WP_ACK_ACCEPTED);ticks(16,450,180);ticks(16,800,180);
    assert(g_wireless_cal.result==RL_CAL_UNSTABLE && g_wireless_cal.captured==5);
    assert(command(RL_CAL_APPLY)==WP_ACK_FORMAT);
    capture(RL_CAL_LEFT,450,180);
    save_ok=false;
    assert(rtos_line_cal_command(RL_CAL_SAVE,250)==WP_ACK_BUSY && writes==1);
    assert(g_wireless_cal.result==RL_CAL_SAVE_FAILED);
    assert(!memcmp(&original,&g_calibration_value,sizeof(original)));
    assert(rtos_line_cal_command(RL_CAL_SAVE,250)==WP_ACK_BUSY && writes==1);
    save_ok=true;
    assert(rtos_line_cal_command(RL_CAL_SAVE,251)==WP_ACK_ACCEPTED && writes==2);
    assert(g_calibration_value.white[1]==180 && g_calibration_value.black[1]==650);
    assert(!g_wireless_cal.session && g_wireless_cal.saved && g_cal_origin==2 && !g_filter_ready);
    assert(rtos_line_cal_command(RL_CAL_SAVE,251)==WP_ACK_ACCEPTED && writes==2);
    uint8_t packet[RL_CAL_FRAME_BYTES];rtos_line_cal_info(42,packet);
    assert(packet[0]==0xf2 && packet[1]==37 && packet[2]==42);
    FILE *file=fopen("testing/rtos_migration/build-host/front_calibration.bin","wb");
    assert(file && fwrite(packet,1,sizeof(packet),file)==sizeof(packet));fclose(file);
    /* Report at send time, including a stop button or ageing ADC snapshot. */
    buttons=2;rtos_line_cal_info(43,packet);
    assert((packet[13] & 12U)==4U);
    buttons=0;clock_ms+=101;rtos_line_cal_info(44,packet);
    assert((packet[13] & 12U)==8U);
    ticks(1,450,180);
    /* RAM-only application does not write flash, and can be explicitly saved later. */
    complete_candidate();assert(command(RL_CAL_APPLY)==WP_ACK_ACCEPTED && writes==2);
    assert(g_cal_origin==3 && !g_wireless_cal.saved && g_wireless_cal.applied);
    assert(command(RL_CAL_SAVE)==WP_ACK_ACCEPTED && writes==3);
    assert(command(RL_CAL_BEGIN)==WP_ACK_ACCEPTED);
    assert(command(RL_CAL_WHITE)==WP_ACK_ACCEPTED);
    ticks(4,120,180);uint8_t count=g_wireless_cal.count;
    assert(rtos_line_cal_command(RL_CAL_WHITE,g_wireless_cal.request_id)==WP_ACK_ACCEPTED);
    assert(g_wireless_cal.count==count); /* Retransmission never restarts sampling. */
    ++g_cancel_serial;ticks(1,120,180);assert(!g_wireless_cal.session);
    assert(command(RL_CAL_BEGIN)==WP_ACK_ACCEPTED);
    assert(command(RL_CAL_WHITE)==WP_ACK_ACCEPTED);
    g_adc_ok=false;ticks(1,120,180);assert(g_wireless_cal.result==RL_CAL_ADC);
    assert(command(RL_CAL_CANCEL)==WP_ACK_ACCEPTED);g_adc_ok=true;
    assert(command(RL_CAL_BEGIN)==WP_ACK_ACCEPTED);allowed=false;ticks(1,120,180);
    assert(!g_wireless_cal.session);allowed=true;
    /* Deadline arithmetic remains correct when the 32-bit millisecond clock wraps. */
    clock_ms=UINT32_MAX-1000U;wireless_cal_tick();
    assert(command(RL_CAL_BEGIN)==WP_ACK_ACCEPTED);
    clock_ms+=RL_CAL_SESSION_MS;wireless_cal_tick();
    assert(!g_wireless_cal.session && g_wireless_cal.result==RL_CAL_TIMEOUT);
    assert(sizeof(g_wireless_cal)<=64);
    puts("PASS wireless calibration: staged capture, quality, safety, idempotence, verified commit and codec");
    return 0;
}
