/* Standalone analog line test, derived from Buddy 2 V7. Not the RTOS coordinator. */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "../dual_wheel_bench/log_store.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "../left_motor_bench/bench_logic.h"
#include "../dual_wheel_bench/dual_logic.h"
#include "../line_motion_integration/logic.h"
#include "hardware/adc.h"
#include "line_logic.h"

enum { HOLD=21, STOP=20, PWM_TOP=999, TRACE_ROWS=100 };
static const uint ma[2]={8,10}, mb[2]={9,11}, ea[2]={0,4}, eb[2]={1,5};
static volatile BenchEncoder enc[2];
static volatile DualGuard guard={.consumed=true};
static volatile unsigned run_mode=LINE_USB_HOLD;
static volatile int32_t origin[2], cut_ticks[2];
static volatile uint32_t initial_invalid[2], cut_invalid[2];
static LineCalibration calibration;
static AnalogTracker tracker;
static AnalogDemand demand;
static uint16_t analog_raw[2];
static float adc_filtered[2], path_mm;
static LineRearm rearm;
static unsigned last_start_block;
static uint32_t runs;
static uint8_t profile;
static LineBreakaway breakaway[2];
_Static_assert(AL_KICK_PWM<=PWM_LIMIT,"Breakaway must remain inside motor PWM cap");
static WheelPI pi[2];
static int32_t previous[2], output[2];
static float velocity[2];
static uint64_t bad_direction_us[2], no_motion_us[2];
static uint64_t last_control, stopped_us, last_trace;
static uint32_t trace_interval_us=50000;
static char recorded_key;
static bool recorded_offline;
typedef struct {
    uint16_t ms;
    int16_t velocity10[2], pwm[2];
    int32_t ticks[2];
    uint16_t adc[2],level[2];
    int16_t target[2];
    uint8_t active,decision;
} Trace;
static Trace trace[TRACE_ROWS];
static unsigned trace_count, trace_dropped;
static bool trace_settling;

/* Dedicated last 4 KiB of the Pico W's flash; firmware never writes while driving.
   Single core only. CRC rejects an interrupted erase/program on the next boot. */
#define LOG_OFFSET (PICO_FLASH_SIZE_BYTES-FLASH_SECTOR_SIZE)
#define LOG_MAGIC 0x31464c41u
enum { WHY_CROSSING=12, WHY_LOST=13 };
extern char __flash_binary_end;
typedef struct {
    uint32_t magic, version, crc, count, dropped;
    uint64_t duration;
    int32_t cutoff[2];
    uint32_t invalid[2];
    uint8_t reason, key, offline, reserved;
    LineCalibration calibration;
    uint8_t profile;
    Trace rows[TRACE_ROWS];
} SavedLog;
_Static_assert(sizeof(SavedLog)<=FLASH_SECTOR_SIZE,"Log exceeds reserved flash sector");
static union { SavedLog data; uint8_t sector[FLASH_SECTOR_SIZE]; } saved;
static bool flash_loaded, save_failed;
static void outputs_off(void);
static bool storage_fits(void) {
    return (uintptr_t)&__flash_binary_end <= XIP_BASE+LOG_OFFSET;
}
static bool stored_valid(const SavedLog *v) {
    return v->magic==LOG_MAGIC && v->version==1 && v->count<=TRACE_ROWS && v->profile<=1 && calibration_valid(&v->calibration)
        && v->crc==log_crc((const uint8_t *)v+offsetof(SavedLog,count),
                          sizeof(*v)-offsetof(SavedLog,count));
}
static void load_log(void) {
    if(!storage_fits()) return;
    const SavedLog *v=(const SavedLog *)(XIP_BASE+LOG_OFFSET);
    if(!stored_valid(v)) return;
    memcpy(trace,v->rows,sizeof(trace));
    trace_count=v->count; trace_dropped=v->dropped;
    recorded_key=v->key; recorded_offline=v->offline;
    guard.duration_us=v->duration; guard.reason=v->reason;
    for(unsigned w=0;w<2;++w) {
        origin[w]=0; cut_ticks[w]=v->cutoff[w]; cut_invalid[w]=v->invalid[w];
    }
    calibration=v->calibration;profile=v->profile;
    flash_loaded=true;
}
static void save_log(void) {
    if(guard.active || !calibration_valid(&calibration) || !storage_fits()) { save_failed=true; return; }
    outputs_off();
    memset(&saved,0,sizeof(saved));
    SavedLog *v=&saved.data;
    v->magic=LOG_MAGIC; v->version=1; v->count=trace_count; v->dropped=trace_dropped;
    v->duration=guard.duration_us; v->reason=guard.reason;
    v->key=recorded_key; v->offline=recorded_offline;
    for(unsigned w=0;w<2;++w) {
        v->cutoff[w]=(int32_t)((int64_t)cut_ticks[w]-origin[w]);
        v->invalid[w]=cut_invalid[w];
    }
    v->calibration=calibration;v->profile=profile;
    memcpy(v->rows,trace,sizeof(trace));
    v->crc=log_crc((const uint8_t *)v+offsetof(SavedLog,count),sizeof(*v)-offsetof(SavedLog,count));
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(LOG_OFFSET,FLASH_SECTOR_SIZE);
    flash_range_program(LOG_OFFSET,saved.sector,FLASH_SECTOR_SIZE);
    restore_interrupts(irq);
    const SavedLog *written=(const SavedLog *)(XIP_BASE+LOG_OFFSET);
    save_failed=!stored_valid(written) || memcmp(written,v,sizeof(*v))!=0;
    flash_loaded=false;
}

static uint8_t read_ab(unsigned w) {
    uint32_t p=gpio_get_all();
    return (uint8_t)((((p>>ea[w])&1u)<<1)|((p>>eb[w])&1u));
}
static void encoder_irq(uint pin, uint32_t events) {
    (void)events;
    unsigned w=(pin==ea[0] || pin==eb[0]) ? 0 : 1;
    bench_encoder_update(&enc[w],read_ab(w));
}
static void outputs_off(void) {
    pwm_set_both_levels(pwm_gpio_to_slice_num(ma[0]),0,0);
    pwm_set_both_levels(pwm_gpio_to_slice_num(ma[1]),0,0);
}
/* Caller either masks IRQs (main) or is the timer ISR. No floating point here. */
static void capture_cutoff(void) {
    for(unsigned w=0;w<2;++w) {
        cut_ticks[w]=enc[w].position;
        cut_invalid[w]=enc[w].invalid-initial_invalid[w];
    }
}
static void kill(uint64_t now,uint8_t reason) {
    bool was_active=guard.active;
    guard_stop(&guard,now,reason);
    outputs_off();
    if(was_active) capture_cutoff();
}
static bool safety_tick(struct repeating_timer *t) {
    (void)t;
    bool active=guard.active;
    uint64_t now=time_us_64();
    guard_tick(&guard,now,run_mode==LINE_USB_HOLD ? !gpio_get(HOLD) : true,!gpio_get(STOP));
    if(guard.active && run_mode!=LINE_USB_HOLD && !gpio_get(HOLD)) guard_stop(&guard,now,WHY_BUTTON);
    if(!guard.active) {
        outputs_off();
        if(active) capture_cutoff();
    }
    return true;
}
static const char *reason_name(uint8_t r) {
    static const char *const names[]={"none","GP20 pressed","GP21 released",
        "run time limit","serial s","USB connection changed","control loop late",
        "no encoder movement under drive","encoder direction mismatch",
        "invalid encoder transitions","distance test limit","GP21 stop pressed",
        "both sensors dark - ambiguous crossing","line not recovered"};
    return r<sizeof(names)/sizeof(names[0]) ? names[r] : "unknown";
}
static void help(void) {
    puts("\nBUDDY23 ANALOG LINE FOLLOW v3 | SDK bench, NOT RTOS NAV | DRIVE OFF on boot");
    puts("GP26/ADC0 LEFT A0; GP28/ADC2 RIGHT A0; sensor supply3.3V; UART OFF.");
    puts("Calibrate, stopped and GP21 released: w=BOTH WHITE; l=LEFT BLACK; r=RIGHT BLACK.");
    puts("Wait1s on each surface before key. w clears old black calibration; k=saves calibration/profile.");
    puts("Place LEFT spot at RIGHT-HAND edge of black (robot perspective), RIGHT spot on white.");
    puts("Start accepts left level150..1000, right<=450; aim left near500 for smoother starts.");
    puts("Hold GP21 then f=follow; release GP21, GP20 or s stops. No b/l/r motor commands.");
    puts("g=USB HANDS-FREE follow: both buttons released,2s countdown, no need to hold GP21.");
    puts("Either button / s / USB disconnect stops hands-free; either button cancels countdown.");
    puts("Profile1(default):150mm/3s. Profile2:600mm/12s. 1/2 select stopped; k saves for offline.");
    puts("BANK ONLY: hold BOTH3s; release BOTH; wait1s; GP21 tap/release;2s countdown.");
    puts("Repeat runs: release BOTH, wait for settling/save then1s estimated standstill; ready=1.");
    puts("Then request f/g again or repeat bank button sequence. No automatic restart.");
    puts("Speeds35..95mm/s, PWM cap25%; PI and breakaway assistance are provisional.");
    puts("Breakaway:25% for up to120ms/control tick; ends at20mm/s,100ms cooldown.");
    puts("Both-white recovery bounded650ms/40mm; both-dark120ms stops. No branch turns or barcode.");
    puts("p=saved run; h=help. Wait3s after stop before switching power sources, then p.");
    printf("CAL mask=%u white=%u,%u black=%u,%u valid=%u profile=%u restored=%u\n",
        calibration.mask,calibration.white[0],calibration.white[1],calibration.black[0],calibration.black[1],
        calibration_valid(&calibration),profile+1,flash_loaded);
}
static uint16_t read_adc(unsigned channel) {
    adc_select_input(channel);(void)adc_read();uint32_t sum=0;
    for(unsigned i=0;i<32;++i) { sum+=adc_read();sleep_us(20); }
    return (uint16_t)((sum+16)/32);
}
static void calibrate(int key) {
    if(guard.active || trace_settling || !gpio_get(HOLD) || !gpio_get(STOP)) {
        puts("CAL rejected: stop, release both buttons and wait1s.");return;
    }
    trace_count=trace_dropped=0;flash_loaded=false;
    if(key=='w') {
        calibration.white[0]=analog_raw[0];calibration.white[1]=analog_raw[1];calibration.mask=1;
    } else if(!(calibration.mask&1)) { puts("Capture BOTH WHITE with w first.");return; }
    else if(key=='l') { calibration.black[0]=analog_raw[0];calibration.mask|=2; }
    else if(key=='r') { calibration.black[1]=analog_raw[1];calibration.mask|=4; }
    help();
}
static void add_trace(uint64_t now) {
    if(!guard.generation) return;
    if(trace_count>=TRACE_ROWS) { ++trace_dropped; return; }
    Trace *r=&trace[trace_count++];
    uint32_t irq=save_and_disable_interrupts();
    r->ms=(uint16_t)((now-guard.start_us)/1000);
    r->active=guard.active;
    r->decision=demand.decision;
    for(unsigned w=0;w<2;++w) { r->adc[w]=analog_raw[w];r->level[w]=demand.level[w];r->target[w]=guard.active?demand.target[w]:0; }
    for(unsigned w=0;w<2;++w) {
        r->ticks[w]=(int32_t)((int64_t)enc[w].position-origin[w]);
        r->pwm[w]=guard.active ? (int16_t)output[w] : 0;
    }
    restore_interrupts(irq);
    for(unsigned w=0;w<2;++w) r->velocity10[w]=(int16_t)(velocity[w]*10.0f);
}
static void dump(void) {
    if(guard.active) { puts("Dump deferred: stop before requesting p."); return; }
    if(!trace_count) { puts("No recorded run yet."); return; }
    if(flash_loaded) puts("RESTORED: CRC-verified analog-line calibration/log.");
    if(save_failed) puts("WARNING: flash save failed; this RAM log is not persistent.");
    printf("RUN: key=%c mode=%s rows=%u dropped=%u settling=%u\n",recorded_key,
        recorded_offline ? "offline" : recorded_key=='g' ? "USB hands-free" : "USB hold",trace_count,trace_dropped,trace_settling);
    printf("CAL white=%u,%u black=%u,%u profile=%u\n",calibration.white[0],calibration.white[1],calibration.black[0],calibration.black[1],profile+1);
    puts("ms,active,L_mm,R_mm,progress_mm,heading_deg,L_mm_s,R_mm_s,PWM_L,PWM_R,ADC_L,ADC_R,levelL_1000,levelR_1000,targetL,targetR,decision");
    for(unsigned i=0;i<trace_count;++i) {
        Trace *r=&trace[i];
        float l=raw_to_mm(0,r->ticks[0]),rr=raw_to_mm(1,r->ticks[1]);
        printf("%u,%u,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%u,%u,%u,%u,%d,%d,%u\n",r->ms,r->active,
            (double)l,(double)rr,(double)((l+rr)*0.5f),(double)heading_deg(l,rr),
            (double)(r->velocity10[0]*0.1f),(double)(r->velocity10[1]*0.1f),
            r->pwm[0],r->pwm[1],r->adc[0],r->adc[1],r->level[0],r->level[1],r->target[0],r->target[1],r->decision);
    }
    uint32_t irq=save_and_disable_interrupts();
    uint8_t why=guard.reason;
    uint64_t duration=guard.duration_us;
    int64_t dl=(int64_t)cut_ticks[0]-origin[0],dr=(int64_t)cut_ticks[1]-origin[1];
    uint32_t il=cut_invalid[0],ir=cut_invalid[1];
    restore_interrupts(irq);
    float l=raw_to_mm(0,dl),r=raw_to_mm(1,dr);
    printf("STOP: %s | run_ms=%" PRIu64 " L_mm=%.2f R_mm=%.2f heading_deg=%.2f invalid_L=%" PRIu32
           " invalid_R=%" PRIu32 "\n",reason_name(why),duration/1000,(double)l,
           (double)r,(double)heading_deg(l,r),il,ir);
    puts("END RUN LOG. Distances/headings are encoder estimates, not external measurements.");
}
static bool start_run(unsigned mode) {
    uint16_t left=line_level(analog_raw[0],calibration.white[0],calibration.black[0]);
    uint16_t right=line_level(analog_raw[1],calibration.white[1],calibration.black[1]);
    last_start_block=line_start_block(guard.active,trace_settling,calibration_valid(&calibration),
        !gpio_get(STOP),rearm.ready,left,right,!gpio_get(HOLD),mode);
    if(last_start_block) return false;
    uint64_t now=time_us_64();
    uint32_t irq=save_and_disable_interrupts();
    bool ok=guard_start(&guard,now,mode!=LINE_USB_HOLD || !gpio_get(HOLD),!gpio_get(STOP));
    if(ok) {
        ++runs;rearm=(LineRearm){0};run_mode=mode;
        guard.deadline_us=now+(profile?12000000u:3000000u);
        trace_interval_us=profile?150000:50000;
        for(unsigned w=0;w<2;++w) {
            origin[w]=previous[w]=enc[w].position;initial_invalid[w]=enc[w].invalid;
            velocity[w]=0;output[w]=0;pi[w]=(WheelPI){0};
            breakaway[w]=(LineBreakaway){0};bad_direction_us[w]=no_motion_us[w]=0;
        }
        tracker=(AnalogTracker){0};path_mm=0;
        demand=analog_demand(&tracker,&calibration,analog_raw[0],analog_raw[1],now,0);
        outputs_off();trace_count=trace_dropped=0;
        recorded_key=mode==LINE_USB_FREE?'g':'f';recorded_offline=mode==LINE_BANK;trace_settling=false;
        flash_loaded=false;save_failed=false;last_control=last_trace=now;
    }
    restore_interrupts(irq);
    if(!ok) last_start_block=START_GUARD;
    if(ok) add_trace(now);
    return ok;
}
static void report_start_block(void) {
    static const char *const text[]={"none","run already active","wait for settling/log save",
        "capture valid w/l/r calibration","release GP20","release both buttons; wait for save and1s standstill",
        "left sensor sees white: move onto edge/black (level>=150)",
        "right sensor too dark: move it toward white (level<=450)","hold GP21 for f",
        "release GP21 for hands-free/offline start","release/repress GP21 and wait for cooldown"};
    printf("START rejected: %s | ready=%u levelL=%u levelR=%u\n",
        last_start_block<sizeof(text)/sizeof(text[0])?text[last_start_block]:"unknown",rearm.ready,
        line_level(analog_raw[0],calibration.white[0],calibration.black[0]),
        line_level(analog_raw[1],calibration.white[1],calibration.black[1]));
}
static void control_step(uint64_t now) {
    uint64_t elapsed=now-last_control;if(elapsed<CONTROL_PERIOD_US) return;
    last_control=now;
    uint32_t irq=save_and_disable_interrupts();
    bool active=guard.active;uint32_t generation=guard.generation;
    BenchEncoder e[2]={enc[0],enc[1]};restore_interrupts(irq);
    float dt=(float)elapsed/1000000.0f;int64_t delta[2];float step=0;
    for(unsigned w=0;w<2;++w) {
        delta[w]=(int64_t)e[w].position-previous[w];previous[w]=e[w].position;
        float moved=raw_to_mm(w,delta[w]);step+=fabsf(moved)*0.5f;
        velocity[w]+=0.25f*(moved/dt-velocity[w]);
    }
    if(!active) return;
    path_mm+=step;
    demand=analog_demand(&tracker,&calibration,(uint16_t)adc_filtered[0],(uint16_t)adc_filtered[1],now,(uint32_t)path_mm);
    uint8_t failure=WHY_NONE;int32_t next[2];
    if(demand.stop) failure=demand.decision==AL_BOTH_BLACK?WHY_CROSSING:WHY_LOST;
    if(path_mm>=(profile?600.0f:150.0f)) failure=WHY_TARGET;
    for(unsigned w=0;w<2;++w) {
        next[w]=pi_step(&pi[w],w,1,demand.target[w],velocity[w],dt);
        /* Stronger bounded floor kick; no-movement accounting below continues
           across pulses/cooldowns and all existing kill paths remain active. */
        next[w]=line_breakaway(&breakaway[w],now,demand.target[w],pi[w].ramp,velocity[w],next[w]);
        bad_direction_us[w]=velocity[w]<-20?bad_direction_us[w]+elapsed:0;
        if(bad_direction_us[w]>=50000) failure=WHY_DIRECTION;
        no_motion_us[w]=demand.target[w]>=20 && pi[w].ramp>=20 && llabs(delta[w])<2?no_motion_us[w]+elapsed:0;
        if(no_motion_us[w]>=500000) failure=WHY_FEEDBACK;
        if(e[w].invalid-initial_invalid[w]>=8) failure=WHY_ENCODER;
    }
    irq=save_and_disable_interrupts();
    if(guard.active && guard.generation==generation) {
        if(failure) kill(now,failure);
        else {
            for(unsigned w=0;w<2;++w) {
                output[w]=next[w];DualLevels v=dual_levels(w,next[w]);
                pwm_set_both_levels(pwm_gpio_to_slice_num(ma[w]),v.a,v.b);
            }
            guard.last_control_us=now;
        }
    }
    restore_interrupts(irq);
}
int main(void) {
    for(uint p=8;p<=11;++p) { gpio_init(p); gpio_put(p,0); gpio_set_dir(p,GPIO_OUT); }
    for(unsigned w=0;w<2;++w) {
        pwm_config cfg=pwm_get_default_config();
        pwm_config_set_wrap(&cfg,PWM_TOP);
        pwm_config_set_clkdiv(&cfg,(float)clock_get_hz(clk_sys)/(10000*(PWM_TOP+1)));
        unsigned slice=pwm_gpio_to_slice_num(ma[w]);
        pwm_init(slice,&cfg,false); pwm_set_both_levels(slice,0,0);
        gpio_set_function(ma[w],GPIO_FUNC_PWM); gpio_set_function(mb[w],GPIO_FUNC_PWM);
        pwm_set_enabled(slice,true);
        gpio_init(ea[w]); gpio_set_dir(ea[w],GPIO_IN); gpio_pull_up(ea[w]);
        gpio_init(eb[w]); gpio_set_dir(eb[w],GPIO_IN); gpio_pull_up(eb[w]);
        enc[w].ab=read_ab(w);
    }
    gpio_init(HOLD); gpio_set_dir(HOLD,GPIO_IN); gpio_pull_up(HOLD);
    gpio_init(STOP); gpio_set_dir(STOP,GPIO_IN); gpio_pull_up(STOP);
    gpio_set_irq_enabled_with_callback(ea[0],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true,encoder_irq);
    gpio_set_irq_enabled(eb[0],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    gpio_set_irq_enabled(ea[1],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    gpio_set_irq_enabled(eb[1],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    irq_set_priority(IO_IRQ_BANK0,0x40); /* Encoder capture may pre-empt safety timer. */
    struct repeating_timer timer;
    if(!add_repeating_timer_us(-1000,safety_tick,NULL,&timer)) while(true) tight_loop_contents();
    adc_init();adc_gpio_init(26);adc_gpio_init(28);
    analog_raw[0]=read_adc(0);analog_raw[1]=read_adc(2);
    adc_filtered[0]=analog_raw[0];adc_filtered[1]=analog_raw[1];
    stdio_init_all();load_log();last_control=time_us_64();
    bool old_connected=false,usb_pending=false;uint8_t raw_buttons=0,buttons=0;
    uint64_t changed=0,last_status=0,last_adc=0,usb_start_at=0;uint32_t seen_stops=0;
    BenchArm arm={0};
    while(true) {
        uint64_t now=time_us_64();bool connected=stdio_usb_connected(),host=tud_mounted();
        bool hold=!gpio_get(HOLD),stop=!gpio_get(STOP);
        uint8_t raw=(hold?1:0)|(stop?2:0);
        if(raw!=raw_buttons) { raw_buttons=raw;changed=now; }
        if(now-changed>=20000) buttons=raw_buttons;
        if(now-last_adc>=10000) {
            last_adc=now;analog_raw[0]=read_adc(0);analog_raw[1]=read_adc(2);
            for(unsigned w=0;w<2;++w) adc_filtered[w]+=0.35f*(analog_raw[w]-adc_filtered[w]);
        }
        if(connected!=old_connected) {
            usb_pending=false;
            if(guard.active) { uint32_t irq=save_and_disable_interrupts();kill(now,WHY_USB);restore_interrupts(irq); }
            if(connected) { arm.state=ARM_IDLE;help();if(trace_count) dump(); }
            old_connected=connected;
        }
        if(host && guard.active && run_mode==LINE_BANK) {
            uint32_t irq=save_and_disable_interrupts();kill(now,WHY_USB);restore_interrupts(irq);
        }
        line_rearm_step(&rearm,now,!hold && !stop,fabsf(velocity[0])<5 && fabsf(velocity[1])<5,
            guard.active || trace_settling || stop);
        bool launch=bench_arm_step(&arm,(uint32_t)now,host,usb_pending || trace_settling || guard.active,buttons&1,buttons&2);
        if(launch) (void)start_run(LINE_BANK);
        int ch=connected?getchar_timeout_us(0):PICO_ERROR_TIMEOUT;
        if(ch=='h') help();
        else if(ch=='p') dump();
        else if(ch=='s') {
            usb_pending=false;arm.state=ARM_IDLE;uint32_t irq=save_and_disable_interrupts();kill(now,WHY_SERIAL);restore_interrupts(irq);
        } else if(ch=='w' || ch=='l' || ch=='r') { usb_pending=false;calibrate(ch); }
        else if(ch=='k' || ch=='1' || ch=='2') {
            usb_pending=false;
            if(guard.active || trace_settling || hold || stop) puts("CONFIG rejected: stop/release both buttons.");
            else if(ch=='k') { save_log();puts(save_failed?"SAVE FAILED":"CALIBRATION/PROFILE SAVED; bank-only requires fresh button sequence."); }
            else { profile=ch=='2';printf("PROFILE %u selected; k saves for power-bank boot.\n",profile+1); }
        } else if(ch=='f') {
            usb_pending=false;bool ok=start_run(LINE_USB_HOLD);
            if(ok) puts("FOLLOW accepted: USB hold GP21.");else report_start_block();
        } else if(ch=='g') {
            if(usb_pending) puts("USB countdown already pending; s/either button cancels.");
            else {
                last_start_block=line_start_block(guard.active,trace_settling,calibration_valid(&calibration),stop,
                    rearm.ready,line_level(analog_raw[0],calibration.white[0],calibration.black[0]),
                    line_level(analog_raw[1],calibration.white[1],calibration.black[1]),hold,LINE_USB_FREE);
                if(last_start_block) report_start_block();
                else { usb_pending=true;usb_start_at=time_us_64()+2000000;puts("USB HANDS-FREE: starts in2s; release robot/buttons. s/either button cancels."); }
            }
        }
        if(usb_pending) {
            if(line_countdown_cancel(connected,hold,stop)) { usb_pending=false;puts("USB countdown cancelled."); }
            else if(time_us_64()>=usb_start_at) {
                usb_pending=false;
                if(start_run(LINE_USB_FREE)) puts("FOLLOW accepted: USB hands-free; either button/s stops.");
                else report_start_block();
            }
        }
        control_step(time_us_64());now=time_us_64();
        if(guard.stops!=seen_stops) {
            seen_stops=guard.stops;stopped_us=now;trace_settling=true;run_mode=LINE_USB_HOLD;
            add_trace(now);last_trace=now;
            if(connected) printf("STOP: %s; wait3s then p\n",reason_name(guard.reason));
        }
        if(guard.active || trace_settling) {
            if(now-last_trace>=trace_interval_us) { add_trace(now);last_trace=now; }
            if(trace_settling && now-stopped_us>=1000000) {
                trace_settling=false;save_log();
                if(connected) puts(save_failed?"FLASH SAVE FAILED":"RUN SAVED; p prints log.");
            }
        }
        if(connected && now-last_status>=200000) {
            uint16_t nl=line_level(analog_raw[0],calibration.white[0],calibration.black[0]);
            uint16_t nr=line_level(analog_raw[1],calibration.white[1],calibration.black[1]);
            printf("ANALOG: L=%u R=%u levelL=%u levelR=%u cal=%u active=%u targets=%d,%d PWM=%ld,%ld arm=%u ready=%u pending=%u runs=%" PRIu32 "\n",
                analog_raw[0],analog_raw[1],nl,nr,calibration_valid(&calibration),guard.active,
                guard.active?demand.target[0]:0,guard.active?demand.target[1]:0,
                guard.active?(long)output[0]:0,guard.active?(long)output[1]:0,arm.state,rearm.ready,usb_pending,runs);
            last_status=now;
        }
        sleep_ms(1);
    }
}
