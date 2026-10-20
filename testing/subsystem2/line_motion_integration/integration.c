/* Isolated S3 -> real Navigation Coordinator -> real S2 bench adapter.
   uT-Kernel, SINGLE CORE. No Pico SDK timer/GPIO handlers replace the kernel. */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <string.h>
#include <stdlib.h>
#include "../system/nav_iface.h"
#include "../system/sys_hal.h"
#include "../system/safety_iface.h"
#include "../system/sys_fault.h"
#include "../subsystem3/s3_iface.h"
#include "../subsystem3/s3_config.h"
#include "../subsystem5/b5_iface.h"
#include "integration.h"
#include "logic.h"
#include "bench_logic.h"
#include "dual_logic.h"
#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define PWM_CC(w) (0x40050000u+0x14u*(4u+(w))+0x0cu)
#define TIMER_BASE_I 0x40054000u
#define SIO_IN_I 0xd0000004u
#define HOLD 21u
#define STOP 20u
#define RUN_LIMIT_US 5000000u
#define TRAVEL_LIMIT_MM 150.0f
extern volatile UW tm_usb_link_state,tm_usb_dropped,tm_usb_irq_count;
extern UW knl_idle_stack_used(UINT core);
static volatile uint32_t motion_cycles,line_frames,cutoff_ticks;
static const unsigned a_pin[2]={0,4},b_pin[2]={1,5};
static volatile BenchEncoder encoder[2];
static volatile bool active,inhibited=true,offline;
static volatile uint32_t generation,started,last_command,last_control,ended,reason;
static volatile int32_t origin[2],cutoff[2];
static volatile uint32_t invalid_origin[2],invalid_cut[2],accepted;
static volatile uint32_t stop_reply_id,stop_requested_at;
static volatile int16_t requested[2],target[2];
static float velocity[2];
static int32_t last_ticks[2],output[2];
static WheelPI pi[2];
static uint32_t still_us[2],wrong_us[2],pulse_us[2],cooldown_us[2];
static BenchLog log_data;
static bool settling,stored,save_ok,used,seen_active;
static uint32_t trace_at;
static int16_t distance_mm;
static BenchArmEvent arm_events[32];
static uint32_t arm_count;
static uint8_t start_reject;
enum { R_NONE,R_GP20,R_GP21,R_DEADLINE,R_TARGET_STALE,R_LOOP_STALE,R_TRAVEL,
       R_INHIBIT,R_UNSUPPORTED,R_ENCODER,R_WRONG,R_NO_MOTION,R_SERIAL,R_NAV_STOP,R_USB,R_ARM_REPORT };
static const char * const why[]={"none","GP20 stop","GP21 stop/release","5 s limit",
 "line target older than 50 ms","motion loop late","150 mm test limit",
 "safety inhibit","bounded manoeuvre blocked in line-only test","encoder invalid",
 "encoder wrong direction","no encoder movement","serial stop","coordinator stop","USB connected/disconnected",
 "offline arming diagnostic - no accepted run"};
static uint32_t pins(void) { return REG(SIO_IN_I); }
static bool pressed(unsigned n) { return (pins()&(1u<<n))==0; }
static void drive_off(void) { REG(PWM_CC(0))=0; REG(PWM_CC(1))=0; }
static void stop_locked(uint32_t now,unsigned why_code) {
    drive_off();
    if(active) {
        reason=why_code; ended=now;
        for(unsigned w=0;w<2;++w) {
            cutoff[w]=encoder[w].position-origin[w];
            invalid_cut[w]=encoder[w].invalid-invalid_origin[w];
        }
    }
    active=false;
    target[0]=target[1]=0;
}
static void stop_now(unsigned why_code) {
    UINT imask; DI(imask); stop_locked(sys_time_us(),why_code); EI(imask);
}
static volatile uint8_t rx_head,rx_tail;
static uint8_t rx[32];
void bench_rx_char(int ch) {
    /* Called only by the existing USB service task. */
    if(ch=='s') stop_now(R_SERIAL);
    uint8_t next=(uint8_t)((rx_head+1)&31u);
    if(next!=rx_tail) { rx[rx_head]=(uint8_t)ch;__asm volatile("dmb" ::: "memory");rx_head=next; }
}
static int rx_poll(void) {
    UINT imask;DI(imask);
    int ch=-1;
    if(rx_tail!=rx_head) { ch=rx[rx_tail];rx_tail=(uint8_t)((rx_tail+1)&31u); }
    EI(imask);return ch;
}
static void encoder_edge(uint32_t pin,uint32_t edge,uint32_t time) {
    (void)edge;(void)time;
    unsigned w=(pin==a_pin[0] || pin==b_pin[0]) ? 0 : 1;
    uint32_t p=pins();
    bench_encoder_update(&encoder[w],(uint8_t)((((p>>a_pin[w])&1u)<<1)|((p>>b_pin[w])&1u)));
}
/* Dedicated hardware alarm, independent of task scheduling. No FP/printing. */
static void watchdog_irq(UINT intno) {
    (void)intno;
    ++cutoff_ticks;
    REG(TIMER_BASE_I+0x34)=2;
    uint32_t now=sys_time_us();
    if(active) {
        if(pressed(STOP)) stop_locked(now,R_GP20);
        else if((offline && pressed(HOLD)) || (!offline && !pressed(HOLD))) stop_locked(now,R_GP21);
        else if(now-started>=RUN_LIMIT_US) stop_locked(now,R_DEADLINE);
        else if(bench_stale(now,last_command)) stop_locked(now,R_TARGET_STALE);
        else if(now-last_control>=50000) stop_locked(now,R_LOOP_STALE);
        else if(offline ? tm_usb_link_state>=2 : tm_usb_link_state<2) stop_locked(now,R_USB);
    }
    REG(TIMER_BASE_I+0x14)=sys_time_us()+1000;
}
/* Coordinator/S3 task calls. Both target values and time are committed together. */
ER mc_set_wheel_speeds(int16_t l,int16_t r) {
    int16_t scaled[2];
    if(!bench_targets(l,r,scaled)) { stop_now(R_UNSUPPORTED); return E_PAR; }
    UINT imask; DI(imask);
    bool ok=active && !inhibited;
    if(ok) {
        requested[0]=l; requested[1]=r;
        target[0]=scaled[0]; target[1]=scaled[1];
        last_command=sys_time_us(); ++accepted;
    }
    EI(imask);
    return ok ? E_OK : E_OBJ;
}
ER mc_drive_distance(int32_t mm,int16_t speed,uint32_t id) {
    (void)mm;(void)speed;(void)id;stop_now(R_UNSUPPORTED);return E_NOSPT;
}
ER mc_turn(int32_t dd,int16_t speed,uint32_t id) {
    (void)dd;(void)speed;(void)id;stop_now(R_UNSUPPORTED);return E_NOSPT;
}
ER mc_stop(uint32_t id) {
    stop_now(R_NAV_STOP);
    UINT imask;DI(imask);
    uint32_t old=stop_reply_id;stop_reply_id=id;stop_requested_at=sys_time_us();
    EI(imask);
    if(old && old!=id) (void)nav_post_motion_result(old,NAV_MOTION_CANCELLED);
    return E_OK;
}
void mc_inhibit(bool on) {
    UINT imask; DI(imask);
    inhibited=on;
    if(on) stop_locked(sys_time_us(),R_INHIBIT);
    EI(imask);
    if(on) safety_ack_drive_off();
}
bool s3_motion_get_distance(int16_t *mm) { *mm=distance_mm; return true; }
bool b5_motion_get_estimate(b5_motion_est_t *est) {
    est->distance_mm=distance_mm;
    est->heading_dd=0; /* S5 is not started and must not consume this stub heading. */
    return false;
}
/* Buddy 5 sensor/servo module is absent; no fake obstacle events or motion. */
ER b5_post_event(UINT event) { (void)event;return E_OK; }
void nav_telemetry_publish(const uint8_t *f,uint32_t n) { (void)f;(void)n; }
void line_telemetry_publish(const uint8_t *f,uint32_t n) { (void)f;(void)n;++line_frames; }
void safety_telemetry_publish(const uint8_t *f,uint32_t n) { (void)f;(void)n; }
static void trace_row(uint32_t now) {
    if(log_data.count>=BENCH_ROWS) { ++log_data.dropped;return; }
    BenchRow *r=&log_data.row[log_data.count++];
    UINT imask; DI(imask);
    r->ms=(uint16_t)((now-started)/1000);
    r->active=active;
    for(unsigned w=0;w<2;++w) {
        r->ticks[w]=encoder[w].position-origin[w];
        r->requested[w]=requested[w];r->target[w]=target[w];
        r->pwm[w]=active ? (int16_t)output[w] : 0;
    }
    EI(imask);
    for(unsigned w=0;w<2;++w) r->speed10[w]=(int16_t)(velocity[w]*10);
    uint32_t p=pins();r->raw=(uint8_t)(((p>>6)&1u)|(((p>>7)&1u)<<1));
    r->sensors=(uint8_t)s3_ir_read(); r->nav=(uint8_t)nav_get_state();
    r->owner=(uint8_t)nav_get_owner();r->line=(uint8_t)s3_get_state();
}
static void dump(void) {
    if(active || settling) { tm_putstring((UB *)"Wait until stopped and log saved.\n");return; }
    if(!log_data.count) { tm_putstring((UB *)"No integration run saved yet.\n");return; }
    tm_printf((UB *)"B23 LINE v4: restored=%u save_ok=%u rows=%u dropped=%u accepted=%u\n",
        stored,save_ok,log_data.count,log_data.dropped,log_data.accepted);
    tm_putstring((UB *)"ARM: states 0=idle 1=holding-both 2=release-both 3=wait-tap 4=release-tap 5=countdown; buttons bit0=GP21 bit1=GP20\n");
    tm_putstring((UB *)"ARM: reject bits 1=used 2=settling 4=active 8=GP20 16=safety 32=nav-not-ready 64=s3-not-idle\n");
    for(unsigned i=0;i<log_data.arm_count && i<32;++i) {
        BenchArmEvent *a=&log_data.arm[i];
        if(i%4==0) (void)tk_dly_tsk(20);
        tm_printf((UB *)"ARM ms=%u state=%u buttons=%u usb=%u reject=%u\n",a->ms,a->state,a->buttons,a->usb,a->reject);
    }
    tm_putstring((UB *)"ms,active,rawL,rawR,blackL,blackR,nav,owner,s3,L_mm10,R_mm10,L_speed10,R_speed10,requestL,requestR,targetL,targetR,PWM_L,PWM_R\n");
    uint32_t tx_before=tm_usb_dropped;
    for(unsigned i=0;i<log_data.count;++i) {
        if(i%4==0) (void)tk_dly_tsk(20);
        BenchRow *r=&log_data.row[i];
        tm_printf((UB *)"%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
         r->ms,r->active,r->raw&1,(r->raw>>1)&1,r->sensors&1,(r->sensors>>1)&1,
         r->nav,r->owner,r->line,(int)(raw_to_mm(0,r->ticks[0])*10),(int)(raw_to_mm(1,r->ticks[1])*10),
         r->speed10[0],r->speed10[1],r->requested[0],r->requested[1],r->target[0],r->target[1],r->pwm[0],r->pwm[1]);
    }
    unsigned k=log_data.reason;
    tm_printf((UB *)"STOP: %s | run_ms=%u invalid_L=%u invalid_R=%u usb_tx_dropped=%u\nEND B23 LOG\n",
        (UB *)(k<sizeof(why)/sizeof(why[0])?why[k]:"unknown"),log_data.duration_us/1000,
        log_data.invalid[0],log_data.invalid[1],(uint32_t)tm_usb_dropped-tx_before);
}
static bool start_run(bool bank) {
    start_reject=(used?1:0)|(settling?2:0)|(active?4:0)|(pressed(STOP)?8:0)|
       (safety_drive_inhibited()?16:0)|(nav_get_state()!=NAV_ST_READY?32:0)|(s3_get_state()!=S3_ST_IDLE?64:0);
    if(start_reject) return false;
    memset(&log_data,0,sizeof(log_data));
    log_data.arm_count=arm_count;memcpy(log_data.arm,arm_events,sizeof(arm_events));
    UINT imask;DI(imask);
    drive_off();++generation;
    started=last_command=last_control=sys_time_us();
    reason=0;accepted=0;offline=bank;
    for(unsigned w=0;w<2;++w) {
        origin[w]=last_ticks[w]=encoder[w].position;invalid_origin[w]=encoder[w].invalid;
        target[w]=requested[w]=0;pi[w]=(WheelPI){0};velocity[w]=0;output[w]=0;
        still_us[w]=wrong_us[w]=pulse_us[w]=cooldown_us[w]=0;
    }
    inhibited=false;active=true;used=true;
    EI(imask);
    stored=false;save_ok=false;seen_active=true;trace_at=started;
    trace_row(started);
    if(nav_post_command(NAV_CMD_START)!=E_OK) { stop_now(R_NAV_STOP); return false; }
    return true;
}
static void control(uint32_t now,uint32_t elapsed) {
    BenchEncoder e[2];int16_t wanted[2];uint32_t gen;bool running;
    UINT imask;DI(imask);
    e[0]=encoder[0];e[1]=encoder[1];
    wanted[0]=target[0];wanted[1]=target[1];gen=generation;running=active;
    EI(imask);
    float dt=elapsed/1000000.0f;
    int32_t change[2],next[2];
    float pos[2];
    unsigned fail=0;
    for(unsigned w=0;w<2;++w) {
        change[w]=e[w].position-last_ticks[w];last_ticks[w]=e[w].position;
        float measured=raw_to_mm(w,change[w])/dt;
        velocity[w]+=0.25f*(measured-velocity[w]);
        pos[w]=raw_to_mm(w,e[w].position-origin[w]);
        next[w]=pi_step(&pi[w],w,1,(float)wanted[w],velocity[w],dt);
        /* Brief, bounded help for low-speed steering wheel breakaway. */
        cooldown_us[w]=subtract_us(cooldown_us[w],elapsed);
        if(pulse_us[w]) {
            pulse_us[w]=subtract_us(pulse_us[w],elapsed);
            if(velocity[w]>=20 || !pulse_us[w] || !wanted[w]) {
                pulse_us[w]=0;cooldown_us[w]=100000;
            }
        }
        if(running && wanted[w]>=20 && pi[w].ramp>=20 && velocity[w]>=-8 && velocity[w]<8 &&
           !pulse_us[w] && !cooldown_us[w]) pulse_us[w]=80000;
        if(pulse_us[w] && next[w]<180) next[w]=180;
        still_us[w]=running && wanted[w]>=20 && pi[w].ramp>=20 && abs_ticks(change[w])<2 ? still_us[w]+elapsed : 0;
        wrong_us[w]=running && velocity[w]<-20 ? wrong_us[w]+elapsed : 0;
        if(still_us[w]>=500000) fail=R_NO_MOTION;
        if(wrong_us[w]>=50000) fail=R_WRONG;
        if(e[w].invalid-invalid_origin[w]>=8) fail=R_ENCODER;
    }
    /* S3 receives a continuous wrapping odometer, not reset command progress. */
    distance_mm=(int16_t)(uint16_t)(int32_t)((raw_to_mm(0,e[0].position)+raw_to_mm(1,e[1].position))*0.5f);
    if((pos[0]+pos[1])*0.5f>=TRAVEL_LIMIT_MM) fail=R_TRAVEL;
    if(safety_drive_inhibited()) fail=R_INHIBIT;
    DI(imask);
    if(active && gen==generation) {
        if(fail) stop_locked(now,fail);
        else {
            for(unsigned w=0;w<2;++w) {
                output[w]=next[w];DualLevels v=dual_levels(w,next[w]);
                REG(PWM_CC(w))=(uint32_t)v.a|((uint32_t)v.b<<16);
            }
            last_control=now;
        }
    }
    EI(imask);
}
static void motion_task(INT stacd,void *exinf) {
    (void)stacd;(void)exinf;
    uint32_t previous=sys_time_us();
    for(;;) {
        uint32_t now=sys_time_us(),elapsed=now-previous;
        if(elapsed>=10000) {
            previous=now;++motion_cycles;control(now,elapsed);safety_kick(SAFETY_SRC_MOTION);
        }
        if(stop_reply_id && !active && now-stop_requested_at>=100000 &&
           velocity[0]>-5 && velocity[0]<5 && velocity[1]>-5 && velocity[1]<5) {
            uint32_t id=stop_reply_id;
            if(nav_post_motion_result(id,NAV_MOTION_DONE)==E_OK) {
                UINT mask;DI(mask);if(stop_reply_id==id) stop_reply_id=0;EI(mask);
            }
        }
        if(seen_active && !active && !settling) {
            settling=true;trace_row(now);trace_at=now;
            (void)nav_post_command(NAV_CMD_STOP);
            log_data.reason=reason;log_data.duration_us=ended-started;
            for(unsigned w=0;w<2;++w) { log_data.cutoff[w]=cutoff[w];log_data.invalid[w]=invalid_cut[w]; }
            log_data.accepted=accepted;
        }
        if((active || settling) && now-trace_at>=100000) { trace_row(now);trace_at=now; }
        if(settling && now-ended>=1000000 && nav_get_state()!=NAV_ST_RUNNING) {
            drive_off();save_ok=bench_flash_save(&log_data);settling=false;seen_active=false;
        }
        (void)tk_dly_tsk(10);
    }
}
static void info(void) {
    tm_putstring((UB *)"\nBUDDY 2 + NAV + BUDDY 3 LINE integration v4 | real RTOS, real sensors/encoders\n");
    tm_putstring((UB *)"DRIVE OFF on boot. First verify raw GP6 L / GP7 R on white and black. Black assumed HIGH.\n");
    tm_putstring((UB *)"IR diagnostics stream while idle. h=help p=saved log s=stop f=USB hold-GP21 test.\n");
    tm_putstring((UB *)"Bank: hold BOTH 3s, release BOTH, then tap/release GP21; 2s countdown. Either button stops.\n");
    tm_putstring((UB *)"One run per boot; fixed 50% S3 speeds, max100mm/s, 150mm / 5s hard stop.\n");
    tm_putstring((UB *)"No barcode turns/search/bypass: those commands stop this limited test.\n");
    tm_putstring((UB *)"After stop wait 3s, switch power sources, open USB serial, p retrieves flash log.\n");
}
static void console_task(INT stacd,void *exinf) {
    (void)stacd;(void)exinf;
    BenchArm arm={0};
    bool prior_usb=false,report_saved=false,ever_button=false;
    uint8_t raw=0,buttons=0,previous_buttons=255,previous_state=255,previous_reject=255;
    uint32_t changed=0,last_event=0,last_print=0,seq=0;
    for(;;) {
        uint32_t now=sys_time_us();bool usb=tm_usb_link_state>=2;
        bool hold=pressed(HOLD),stop=pressed(STOP);
        uint8_t input=(hold?1:0)|(stop?2:0);
        if(raw!=input) { raw=input;changed=now; }
        if(now-changed>=20000) buttons=raw;
        if(usb && !prior_usb) { arm.state=ARM_IDLE;info();if(log_data.count) dump(); }
        prior_usb=usb;
        bool launch=bench_arm_step(&arm,now,usb,used || active || settling,buttons&1,buttons&2);
        if(!usb && !used && (buttons!=previous_buttons || arm.state!=previous_state || start_reject!=previous_reject)) {
            last_event=now;ever_button|=buttons!=0;
            if(arm_count<32) arm_events[arm_count++]=(BenchArmEvent){now/1000,arm.state,buttons,(uint8_t)tm_usb_link_state,start_reject};
            previous_buttons=buttons;previous_state=arm.state;previous_reject=start_reject;
        }
        if(launch) (void)start_run(true);
        /* One diagnostic flash write per boot, only idle and fully released.
           A successful later run replaces it with the full motion log. */
        if(!usb && !used && !active && !settling && !buttons && !input && !report_saved &&
           arm.state!=ARM_COUNTDOWN && now-last_event>=5000000u && (ever_button || now>=30000000u)) {
            memset(&log_data,0,sizeof(log_data));log_data.reason=R_ARM_REPORT;
            log_data.arm_count=arm_count;memcpy(log_data.arm,arm_events,sizeof(arm_events));
            started=now;trace_row(now);drive_off();save_ok=bench_flash_save(&log_data);report_saved=true;
        }
        int ch=rx_poll();
        if(ch=='p') dump(); else if(ch=='h') info();
        else if(ch=='s') { arm.state=ARM_IDLE;stop_now(R_SERIAL); }
        else if(ch=='f' && usb && hold && !stop) {
            if(!start_run(false)) tm_putstring((UB *)"Start rejected: one run/boot; wait Ready/Idle; check safety.\n");
        }
        if(usb && !active && !settling && now-last_print>=250000) {
            uint32_t bits=pins();
            tm_printf((UB *)"IR: seq=%u ms=%u rawL=%u rawR=%u blackL=%u blackR=%u nav=%u s3=%u save=%u mot=%u line=%u usb_irq=%u cut_irq=%u idle_stack=%u/2048 drops=%u\n",
                seq++,now/1000,(bits>>6)&1,(bits>>7)&1,s3_ir_read()&1,(s3_ir_read()>>1)&1,
                (unsigned)nav_get_state(),(unsigned)s3_get_state(),save_ok,
                motion_cycles,line_frames,(uint32_t)tm_usb_irq_count,cutoff_ticks,
                (uint32_t)knl_idle_stack_used(0),(uint32_t)tm_usb_dropped);
            last_print=now;
        }
        (void)tk_dly_tsk(20);
    }
}
static ER task_start(FP fn,INT priority) {
    T_CTSK t={0};t.tskatr=TA_HLNG|TA_RNG3;t.task=fn;t.itskpri=priority;t.stksz=4096;
    ID id=tk_cre_tsk(&t);return id<0 ? id : tk_sta_tsk(id,0);
}
EXPORT INT usermain(void) {
    /* Single-core build only: all GPIO attach, control and RAM flash on core0. */
    (void)sys_hal_init();
    sys_unreset(1u<<14); /* PWM */
    for(unsigned w=0;w<2;++w) {
        REG(0x40050000u+0x14u*(4u+w))=0;
        REG(0x40050000u+0x14u*(4u+w)+4)=200; /* DIV=12.5; 125 MHz / 12.5 / 1000 =10kHz */
        REG(0x40050000u+0x14u*(4u+w)+0x10)=999;
        REG(PWM_CC(w))=0;
        clr_w(GPIO(8u+2u*w),1u<<7);clr_w(GPIO(9u+2u*w),1u<<7);
        REG(0x40014000u+8u*(8u+2u*w)+4)=4; /* PWM function */
        REG(0x40014000u+8u*(9u+2u*w)+4)=4;
        REG(0x40050000u+0x14u*(4u+w))=1;
        (void)gpio_set_pin(a_pin[w],GPIO_MODE_IN);(void)gpio_set_pin(b_pin[w],GPIO_MODE_IN);
        clr_w(GPIO(a_pin[w]),1u<<2);set_w(GPIO(a_pin[w]),1u<<3);
        clr_w(GPIO(b_pin[w]),1u<<2);set_w(GPIO(b_pin[w]),1u<<3);
        uint32_t bits=pins();encoder[w].ab=(uint8_t)((((bits>>a_pin[w])&1)<<1)|((bits>>b_pin[w])&1));
        if(sys_gpio_irq_attach(a_pin[w],12,encoder_edge,3)!=E_OK ||
           sys_gpio_irq_attach(b_pin[w],12,encoder_edge,3)!=E_OK) goto failed;
    }
    (void)gpio_set_pin(HOLD,GPIO_MODE_IN);(void)gpio_set_pin(STOP,GPIO_MODE_IN);
    for(unsigned pin=20;pin<=21;++pin) {
        clr_w(GPIO(pin),1u<<2);set_w(GPIO(pin),1u<<3);
    }
    if(sys_fault_init()!=E_OK || safety_init()!=E_OK || nav_init()!=E_OK || s3_init()!=E_OK) goto failed;
    stored=bench_flash_load(&log_data);save_ok=stored;
    /* Raw handler deliberately makes no kernel calls; it may pre-empt
       kernel-aware handlers without entering scheduler bookkeeping. */
    T_DINT d={0};d.intatr=TA_ASM;d.inthdr=watchdog_irq;
    if(tk_def_int(1,&d)!=E_OK) goto failed;
    REG(TIMER_BASE_I+0x38+0x2000)=2;REG(TIMER_BASE_I+0x34)=2;
    REG(TIMER_BASE_I+0x14)=sys_time_us()+1000;
    ClearInt(1);EnableInt(1,0);
    (void)safety_watch(SAFETY_SRC_MOTION,100);
    if(safety_start()!=E_OK || nav_start()!=E_OK || s3_start()!=E_OK ||
       task_start((FP)motion_task,12)!=E_OK || task_start((FP)console_task,18)!=E_OK) goto failed;
    /* Returning from usermain makes init_task_main call shutdown_system(),
       which disables interrupts and kills USB. Keep this initial task asleep. */
    for(;;) (void)tk_slp_tsk(TMO_FEVR);
failed:
    stop_now(R_INHIBIT);tm_putstring((UB *)"B23 INIT FAILED: drive disabled.\n");
    /* Preserve the USB service for diagnosis even if app init fails. */
    for(;;) (void)tk_slp_tsk(TMO_FEVR);
}
