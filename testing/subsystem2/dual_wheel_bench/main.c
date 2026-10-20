/* Standalone development bench, not the final micro T-Kernel motion task. */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "log_store.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "../left_motor_bench/bench_logic.h"
#include "dual_logic.h"

enum { HOLD=21, STOP=20, PWM_TOP=999, TRACE_ROWS=100 };
static const uint ma[2]={8,10}, mb[2]={9,11}, ea[2]={0,4}, eb[2]={1,5};
static volatile BenchEncoder enc[2];
static volatile DualGuard guard={.consumed=true};
static volatile bool offline_running;
static volatile int32_t origin[2], cut_ticks[2];
static volatile uint32_t initial_invalid[2], cut_invalid[2];
static DualCommand command;
static WheelPI pi[2];
static TurnAssist turn_assist[2];
static int32_t previous[2], output[2];
static float velocity[2];
static uint64_t bad_direction_us[2], no_motion_us[2];
static bool wheel_done[2];
static uint64_t last_control, stopped_us, last_trace;
static uint32_t trace_interval_us=50000;
static char recorded_key;
static bool recorded_offline;
typedef struct {
    uint16_t ms;
    int16_t velocity10[2], pwm[2];
    int32_t ticks[2];
    uint8_t active;
} Trace;
static Trace trace[TRACE_ROWS];
static unsigned trace_count, trace_dropped;
static bool trace_settling;

/* Dedicated last 4 KiB of the Pico W's flash; firmware never writes while driving.
   Single core only. CRC rejects an interrupted erase/program on the next boot. */
#define LOG_OFFSET (PICO_FLASH_SIZE_BYTES-FLASH_SECTOR_SIZE)
#define LOG_MAGIC 0x35474f4cu
extern char __flash_binary_end;
typedef struct {
    uint32_t magic, version, crc, count, dropped;
    uint64_t duration;
    int32_t cutoff[2];
    uint32_t invalid[2];
    uint8_t reason, key, offline, reserved;
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
    return v->magic==LOG_MAGIC && v->version==5 && v->count>0 && v->count<=TRACE_ROWS
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
    flash_loaded=true;
}
static void save_log(void) {
    if(guard.active || !trace_count || !storage_fits()) { save_failed=true; return; }
    outputs_off();
    memset(&saved,0,sizeof(saved));
    SavedLog *v=&saved.data;
    v->magic=LOG_MAGIC; v->version=5; v->count=trace_count; v->dropped=trace_dropped;
    v->duration=guard.duration_us; v->reason=guard.reason;
    v->key=recorded_key; v->offline=recorded_offline;
    for(unsigned w=0;w<2;++w) {
        v->cutoff[w]=(int32_t)((int64_t)cut_ticks[w]-origin[w]);
        v->invalid[w]=cut_invalid[w];
    }
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
    guard_tick(&guard,now,offline_running ? true : !gpio_get(HOLD),!gpio_get(STOP));
    if(guard.active && offline_running && !gpio_get(HOLD)) guard_stop(&guard,now,WHY_BUTTON);
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
        "invalid encoder transitions","encoder target reached (drive off)","GP21 stop pressed"};
    return r<sizeof(names)/sizeof(names[0]) ? names[r] : "unknown";
}
static void help(void) {
    puts("\nBuddy 2 DUAL FEEDBACK bench v7 | USB only | BOOTS WITH DRIVE OFF");
    puts("Calibration: L=2524.05 R=2528.30 counts/rev; circumference=200 mm; track=110 mm.");
    puts("Both wheels use positive=chassis-forward. PWM cap=25%; PI gains provisional.");
    puts("WIRED raised tests: hold GP21 then f=both forward, b=both backward;");
    puts("l/r=timed left/right pivot. Release GP21 / GP20 / s stops. Max 3 seconds.");
    puts("1/2/3 select timed speed 60/80/100 mm/s while stopped (default 80).");
    puts("OFFLINE selection: D=100 mm at 60 mm/s; G=300 mm at 100 mm/s; Q/E=45 deg left/right.");
    puts("a=arm selected offline test, then unplug USB. No movement on unplug.");
    puts("Tap/release GP21 to request start; 2-second countdown; one run only.");
    puts("During offline run GP20 OR another GP21 press stops. GP20 also cancels countdown.");
    puts("Reconnecting USB cancels pending starts/stops offline motion and dumps last run.");
    puts("POWER-BANK START: hold BOTH GP20+GP21 for 2 s, release BOTH, then tap/release GP21.");
    puts("Before the GP21 start tap, GP20 taps cycle: 0=100 mm, 1=300 mm, 2=left45, 3=right45.");
    puts("GP21 tap/release starts selected test after 2 s; selection alone never drives.");
    puts("Short/turn limit 3 s; G long straight limit 6 s. PWM always capped at 25%.");
    puts("Completed logs auto-save to flash after 1 s settling. Wait 3 s after stop before power-off.");
    puts("p=dump RAM/restored flash log; h=help. Latest saved run replaces previous run.");
    puts("Use ONE power source at a time. Flash save occurs only with both motor outputs off.");
    puts("Turns: at most 100 ms of 20% breakaway assist when slow; no constant final boost.");
    puts("Finish raised direction and stop checks BEFORE floor tests.");
}
static void add_trace(uint64_t now) {
    if(!guard.generation) return;
    if(trace_count>=TRACE_ROWS) { ++trace_dropped; return; }
    Trace *r=&trace[trace_count++];
    uint32_t irq=save_and_disable_interrupts();
    r->ms=(uint16_t)((now-guard.start_us)/1000);
    r->active=guard.active;
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
    if(flash_loaded) puts("RESTORED: CRC-verified log from previous power session.");
    if(save_failed) puts("WARNING: flash save failed; this RAM log is not persistent.");
    printf("RUN: key=%c mode=%s rows=%u dropped=%u settling=%u\n",recorded_key,
        recorded_offline ? "offline" : "USB hold",trace_count,trace_dropped,trace_settling);
    puts("ms,active,L_mm,R_mm,progress_mm,heading_deg,L_mm_s,R_mm_s,L_PWM_permille,R_PWM_permille");
    for(unsigned i=0;i<trace_count;++i) {
        Trace *r=&trace[i];
        float l=raw_to_mm(0,r->ticks[0]),rr=raw_to_mm(1,r->ticks[1]);
        printf("%u,%u,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d\n",r->ms,r->active,
            (double)l,(double)rr,(double)((l+rr)*0.5f),(double)heading_deg(l,rr),
            (double)(r->velocity10[0]*0.1f),(double)(r->velocity10[1]*0.1f),
            r->pwm[0],r->pwm[1]);
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
static bool start_run(char key,float speed,bool offline) {
    DualCommand next;
    if(!command_make(key,speed,&next)) return false;
    uint64_t now=time_us_64();
    uint32_t irq=save_and_disable_interrupts();
    bool accepted=guard_start(&guard,now,offline ? true : !gpio_get(HOLD),!gpio_get(STOP));
    if(accepted) {
        command=next;
        guard.deadline_us=now+command_limit_us(key);
        trace_interval_us=key=='g' ? 100000 : 50000;
        offline_running=offline;
        for(unsigned w=0;w<2;++w) {
            origin[w]=previous[w]=enc[w].position;
            initial_invalid[w]=enc[w].invalid;
            velocity[w]=0; output[w]=0;
            pi[w]=(WheelPI){0};
            turn_assist[w]=(TurnAssist){0};
            bad_direction_us[w]=no_motion_us[w]=0;
            wheel_done[w]=false;
        }
        outputs_off();
        trace_count=trace_dropped=0;
        recorded_key=key; recorded_offline=offline; trace_settling=false;
        flash_loaded=false; save_failed=false;
        last_control=last_trace=now;
    }
    restore_interrupts(irq);
    if(accepted) add_trace(now);
    return accepted;
}
static void control_step(uint64_t now) {
    uint64_t elapsed=now-last_control;
    if(elapsed<CONTROL_PERIOD_US) return;
    last_control=now;
    uint32_t irq=save_and_disable_interrupts();
    bool active=guard.active;
    uint32_t generation=guard.generation;
    BenchEncoder e[2]={enc[0],enc[1]};
    restore_interrupts(irq);
    float dt=(float)elapsed/1000000.0f;
    int64_t deltas[2];
    for(unsigned w=0;w<2;++w) {
        deltas[w]=(int64_t)e[w].position-previous[w];
        previous[w]=e[w].position;
        float measured=raw_to_mm(w,deltas[w])/dt;
        velocity[w]+=0.25f*(measured-velocity[w]);
    }
    if(!active) return;
    int32_t next_output[2];
    uint8_t failure=WHY_NONE;
    for(unsigned w=0;w<2;++w) {
        float pos=raw_to_mm(w,(int64_t)e[w].position-origin[w]);
        float wanted=position_demand(&command,w,pos);
        if(command.bounded && wanted==0) wheel_done[w]=true;
        if(wheel_done[w]) wanted=0;
        float along=velocity[w]*command.sign[w];
        next_output[w]=pi_step(&pi[w],w,command.sign[w],wanted,along,dt);
        if(!wheel_done[w]) next_output[w]=finish_output(&command,w,pos,next_output[w]);
        next_output[w]=turn_assist_step(&turn_assist[w],&command,w,pi[w].ramp,along,
                                       next_output[w],(uint32_t)elapsed,wheel_done[w]);
        bad_direction_us[w]=along < -20.0f ? bad_direction_us[w]+elapsed : 0;
        if(bad_direction_us[w]>=50000) failure=WHY_DIRECTION;
        /* Count a stopped turning wheel throughout pulse cooldowns as well.
           Otherwise pulsed drive could repeatedly reset the no-movement guard. */
        bool turning=command.sign[0]!=command.sign[1];
        bool driven=turning ? wanted>0 && pi[w].ramp>=20 : abs(next_output[w])>=150;
        no_motion_us[w]=driven && llabs(deltas[w])<2 ? no_motion_us[w]+elapsed : 0;
        if(no_motion_us[w]>=500000) failure=WHY_FEEDBACK;
        if(e[w].invalid-initial_invalid[w]>=8) failure=WHY_ENCODER;
    }
    irq=save_and_disable_interrupts();
    if(guard.active && guard.generation==generation) {
        if(failure) kill(now,failure);
        else if(command.bounded && wheel_done[0] && wheel_done[1]) kill(now,WHY_TARGET);
        else {
            for(unsigned w=0;w<2;++w) {
                output[w]=next_output[w];
                DualLevels v=dual_levels(w,output[w]);
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
    stdio_init_all();
    load_log();
    last_control=time_us_64();
    bool old_connected=false,raw_button=false,button=false;
    uint64_t button_changed=0,last_status=0;
    uint32_t seen_stops=0;
    float speed=80;
    char offline_key='d';
    OfflineArm arm={0};
    BootArm boot_arm={0};
    ProfileSelect selection={0};
    bool stop_raw=false,stop_button=false;
    uint64_t stop_changed=0;
    while(true) {
        uint64_t now=time_us_64();
        bool connected=stdio_usb_connected();
        bool hold=!gpio_get(HOLD),stop=!gpio_get(STOP);
        if(hold!=raw_button) { raw_button=hold; button_changed=now; }
        if(now-button_changed>=20000) button=raw_button;
        if(stop!=stop_raw) { stop_raw=stop; stop_changed=now; }
        if(now-stop_changed>=20000) stop_button=stop_raw;
        if(connected!=old_connected) {
            if(guard.active) { uint32_t irq=save_and_disable_interrupts(); kill(now,WHY_USB); restore_interrupts(irq); }
            if(connected) {
                arm.state=OFF_NONE;
                help();
                if(trace_count && !guard.active) dump();
            }
            old_connected=connected;
        }
        if(!connected && guard.active && !offline_running) {
            uint32_t irq=save_and_disable_interrupts(); kill(now,WHY_USB); restore_interrupts(irq);
        }
        /* One deliberate power-bank arm per boot. Holding both never drives.
           Require release of BOTH, then a separate GP21 tap to request motion. */
        if(boot_arm_step(&boot_arm,now,connected,guard.active || trace_settling,hold,stop)) {
            arm.state=OFF_NONE; offline_key='d';
            selection=(ProfileSelect){.selecting=true};
        }
        if(profile_step(&selection,connected,button,stop_button)) {
            offline_key=profile_key(selection.index);
            arm.state=OFF_RELEASE;
        }
        if(offline_step(&arm,now,connected,button,stop))
            (void)start_run(offline_key,speed,true);
        int ch=connected ? getchar_timeout_us(0) : PICO_ERROR_TIMEOUT;
        if(ch=='h') help();
        else if(ch=='p') dump();
        else if(ch=='s') {
            arm.state=OFF_NONE;
            uint32_t irq=save_and_disable_interrupts(); kill(now,WHY_SERIAL); restore_interrupts(irq);
        } else if(ch=='D' || ch=='G' || ch=='Q' || ch=='E' || ch=='a' || ch=='1' || ch=='2' || ch=='3') {
            if(guard.active || hold || trace_settling) puts("CONFIG rejected: stop, release GP21 and wait for settling.");
            else if(ch=='a') { arm.state=OFF_UNPLUG; puts("OFFLINE ARMED: unplug USB, release GP21, tap/release it; 2 s countdown. GP20 cancels."); }
            else {
                arm.state=OFF_NONE;
                if(ch=='1'||ch=='2'||ch=='3') speed=ch=='1'?60:ch=='2'?80:100;
                else offline_key=ch=='D'?'d':ch=='G'?'g':ch=='Q'?'q':'e';
                printf("CONFIG: timed_speed=%.0f mm/s offline_test=%c | drive=off\n",(double)speed,offline_key);
            }
        } else if(ch=='f'||ch=='b'||ch=='l'||ch=='r') {
            arm.state=OFF_NONE;
            bool ok=!trace_settling && start_run((char)ch,speed,false);
            puts(ok ? "RUN accepted: BOTH wheels, USB hold-to-run." : "RUN rejected: release/repress GP21, hold it, release GP20, wait for rest.");
        }
        control_step(time_us_64());
        now=time_us_64();
        if(guard.stops!=seen_stops) {
            seen_stops=guard.stops; stopped_us=now; trace_settling=true;
            offline_running=false;
            add_trace(now); last_trace=now;
            if(connected) printf("STOP: %s | run_ms=%" PRIu64 " | drive=off | press p after 1 s for full log\n",
                reason_name(guard.reason),guard.duration_us/1000);
        }
        if(guard.active || trace_settling) {
            if(now-last_trace>=trace_interval_us) { add_trace(now); last_trace=now; }
            if(trace_settling && now-stopped_us>=1000000) {
                trace_settling=false;
                save_log();
                if(connected) puts(save_failed ? "FLASH SAVE FAILED: retrieve RAM log with p before power-off."
                                               : "LOG SAVED TO FLASH: press p to retrieve complete run.");
            }
        }
        if(connected && guard.active && now-last_status>=200000) {
            printf("LIVE: target_L=%.0f target_R=%.0f L_mm_s=%.1f R_mm_s=%.1f PWM=%ld,%ld\n",
                (double)(pi[0].ramp*command.sign[0]),(double)(pi[1].ramp*command.sign[1]),
                (double)velocity[0],(double)velocity[1],(long)output[0],(long)output[1]);
            last_status=now;
        }
        sleep_ms(1);
    }
}
