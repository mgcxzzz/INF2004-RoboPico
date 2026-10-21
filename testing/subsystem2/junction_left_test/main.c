/* Isolated GP19 cross detection, axle centring and optional left turn. No barcode. */
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
#include "../analog_line_follow/line_logic.h"
#include "signed_wheel.h"
#include "junction_logic.h"

enum { HOLD=21, STOP=20, PWM_TOP=999, TRACE_ROWS=100 };
_Static_assert(J_RUN_US/J_TRACE_US+1000000/J_TRACE_US+20<=TRACE_ROWS,
               "Symmetric run plus settling must fit the saved trace");
static const uint ma[2]={8,10}, mb[2]={9,11}, ea[2]={0,4}, eb[2]={1,5};
static volatile BenchEncoder enc[2];
static volatile DualGuard guard={.consumed=true};
static volatile unsigned run_mode=LINE_USB_HOLD;
static volatile int32_t origin[2], cut_ticks[2];
static volatile uint32_t initial_invalid[2], cut_invalid[2];
static LineCalibration calibration;
static Junction junction;
static SideCalibration side_cal;
static uint8_t test_mode=1,recorded_mode=1;
static uint8_t recorded_version=6;
static bool imported_front;
static AnalogDemand demand;
static uint16_t analog_raw[2];
static float adc_filtered[2], path_mm;
static LineRearm rearm;
static unsigned last_start_block;
static uint32_t runs;
static SignedWheel controller[2];
_Static_assert(AL_KICK_PWM<=PWM_LIMIT,"Breakaway must remain inside motor PWM cap");
static int32_t previous[2];
static volatile int32_t output[2];
static volatile uint64_t peak_until[2];
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
} LegacyTrace;
typedef struct {
    uint32_t magic, version, crc, count, dropped;
    uint64_t duration;
    int32_t cutoff[2];
    uint32_t invalid[2];
    uint8_t reason, key, offline, reserved;
    LineCalibration calibration;
    uint8_t profile;
    LegacyTrace rows[TRACE_ROWS];
} LegacyLog;

typedef struct {
    uint32_t ms;
    int32_t ticks[2];
    int16_t velocity10[2],pwm[2],target[2];
    uint16_t adc[2],level[2];
    uint8_t active,phase,side,decision;
} Trace;
static Trace trace[TRACE_ROWS];
static unsigned trace_count,trace_dropped;
static bool trace_settling;
#define LOG_OFFSET (PICO_FLASH_SIZE_BYTES-FLASH_SECTOR_SIZE)
#define LOG_MAGIC 0x314e434au
extern char __flash_binary_end;
typedef struct {
    uint32_t magic,version,crc,count,dropped;
    uint64_t duration;
    int32_t cutoff[2];uint32_t invalid[2];
    uint8_t reason,key,offline,mode;
    LineCalibration calibration;
    SideCalibration side;
    uint8_t selected_mode;
    uint8_t firmware; /* Uses V1's zero-filled padding: existing record layout unchanged. */
    Junction junction;
    Trace rows[TRACE_ROWS];
} SavedLog;
_Static_assert(sizeof(SavedLog)<=FLASH_SECTOR_SIZE,"Junction log exceeds sector");
_Static_assert(sizeof(Trace)==36,"Unexpected trace ABI");
_Static_assert(offsetof(SavedLog,firmware)==66 && offsetof(SavedLog,junction)==72,
               "V2 revision byte must fit V1 padding without moving records");
_Static_assert(offsetof(SavedLog,rows)==192 && sizeof(SavedLog)==3792,"Keep V1..V6 record ABI");
static union { SavedLog data;uint8_t sector[FLASH_SECTOR_SIZE]; } saved;
static bool flash_loaded,save_failed;
static void outputs_off(void);
static bool storage_fits(void) { return (uintptr_t)&__flash_binary_end<=XIP_BASE+LOG_OFFSET; }
static bool stored_valid(const SavedLog *v) {
    return v->magic==LOG_MAGIC && v->version==1 && v->count<=TRACE_ROWS &&
        v->selected_mode>=1 && v->selected_mode<=3 && v->mode>=1 && v->mode<=3 && v->firmware<=6 &&
        calibration_valid(&v->calibration) && side_valid(&v->side) &&
        v->crc==log_crc((const uint8_t *)v+offsetof(SavedLog,count),sizeof(*v)-offsetof(SavedLog,count));
}
static void load_log(void) {
    if(!storage_fits()) return;
    const SavedLog *v=(const SavedLog *)(XIP_BASE+LOG_OFFSET);
    if(stored_valid(v)) {
        memcpy(trace,v->rows,sizeof(trace));trace_count=v->count;trace_dropped=v->dropped;
        recorded_key=v->key;recorded_offline=v->offline;recorded_mode=v->mode;test_mode=v->selected_mode;
        recorded_version=v->firmware?v->firmware:1;
        guard.duration_us=v->duration;guard.reason=v->reason;
        for(unsigned w=0;w<2;++w) { origin[w]=0;cut_ticks[w]=v->cutoff[w];cut_invalid[w]=v->invalid[w]; }
        calibration=v->calibration;side_cal=v->side;memcpy(&junction,&v->junction,sizeof(junction));
        if(recorded_version<5) junction.rolling=false; /* Old byte was padding. */
        flash_loaded=true;
        return;
    }
    /* Read-only import of V11 front calibration, NOT its trace or run state.
       This experiment has a distinct record magic. First save replaces old log. */
    const LegacyLog *old=(const LegacyLog *)(XIP_BASE+LOG_OFFSET);
    if(old->magic==0x31464c41u && old->version==1 && old->count<=100 && old->profile<=5 && old->reserved<=6 &&
       calibration_valid(&old->calibration) &&
       old->crc==log_crc((const uint8_t *)old+offsetof(LegacyLog,count),sizeof(*old)-offsetof(LegacyLog,count))) {
        calibration=old->calibration;imported_front=true;
    }
}
static void save_log(void) {
    if(guard.active || !calibration_valid(&calibration) || !side_valid(&side_cal) || !storage_fits()) {
        save_failed=true;return;
    }
    outputs_off();memset(&saved,0,sizeof(saved));SavedLog *v=&saved.data;
    v->magic=LOG_MAGIC;v->version=1;v->count=trace_count;v->dropped=trace_dropped;
    v->duration=guard.duration_us;v->reason=guard.reason;v->key=recorded_key;
    v->offline=recorded_offline;v->mode=recorded_mode;v->selected_mode=test_mode;
    v->firmware=trace_count?recorded_version:6;
    v->calibration=calibration;v->side=side_cal;v->junction=junction;
    for(unsigned w=0;w<2;++w) { v->cutoff[w]=cut_ticks[w]-origin[w];v->invalid[w]=cut_invalid[w]; }
    memcpy(v->rows,trace,sizeof(trace));
    v->crc=log_crc((const uint8_t *)v+offsetof(SavedLog,count),sizeof(*v)-offsetof(SavedLog,count));
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(LOG_OFFSET,FLASH_SECTOR_SIZE);flash_range_program(LOG_OFFSET,saved.sector,FLASH_SECTOR_SIZE);
    restore_interrupts(irq);
    const SavedLog *written=(const SavedLog *)(XIP_BASE+LOG_OFFSET);
    save_failed=!stored_valid(written) || memcmp(written,v,sizeof(*v))!=0;flash_loaded=false;
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
    } else {
        /* Bound the stronger start pulse even if the main control is delayed.
           Only integer arithmetic in this ISR; global late-loop cutoff retained. */
        for(unsigned w=0;w<2;++w) {
            int32_t capped=junction_peak_cap(output[w],peak_until[w],now);
            if(capped!=output[w]) {
                output[w]=capped;DualLevels v=junction_levels(w,capped,true);
                pwm_set_both_levels(pwm_gpio_to_slice_num(ma[w]),v.a,v.b);
            }
        }
    }
    return true;
}

static const char *reason_name(uint8_t r) {
    static const char *const names[]={"none","GP20 pressed","GP21 released","25s run limit","serial s",
        "USB connection changed","control loop late","no encoder movement under drive","encoder direction mismatch",
        "invalid encoder transitions","distance test limit","GP21 stop pressed"};
    if(r<sizeof(names)/sizeof(names[0])) return names[r];
    switch(r) {
    case J_COMPLETE:return "test complete";case J_NO_CROSS:return "cross not found / side not armed";
    case J_BAD_STRIPE:return "cross-arm width or centring timeout";case J_CENTRE_DRIFT:return "centring drift / turn arcs unequal";
    case J_SETTLE_TIMEOUT:return "wheels did not settle";case J_TURN_TIMEOUT:return "turn timeout";
    case J_BRANCH_MISSING:return "left branch not confirmed";case J_PATH_LIMIT:return "900mm path limit";
    case J_LINE_LOST:return "line not recovered";default:return "unknown";
    }
}
static const char *phase_name(unsigned phase) {
    const char *const names[]={"idle","approach","centre","settle","left turn","turn settle","exit20mm","done","fault","acquire line"};
    return phase<sizeof(names)/sizeof(names[0])?names[phase]:"unknown";
}
static void help(void) {
    puts("\nBUDDY23 JUNCTION LEFT v6 | isolated SDK test, NO BARCODE | DRIVE OFF on boot");
    puts("Front analog: GP26 LEFT A0 / GP28 RIGHT A0; offset D0 GP19; 3.3V/common GND.");
    puts("Front calibration: w=BOTH WHITE, l=LEFT BLACK, r=RIGHT BLACK; wait1s before capture.");
    puts("Offset polarity: o=capture GP19 over WHITE, b=capture GP19 over BLACK. Motors remain OFF.");
    puts("Both captures must differ; k saves calibration/mode. First save replaces previous firmware log.");
    puts("1=centre/STOP; 2=old in-place LEFT; 3=NEW rolling LEFT starting55mm before cross centre. k saves.");
    puts("Mode3: LEFT OFF/RIGHT forward, then acquire line with BOTH forward, follow20mm/STOP. Use3,k.");
    puts("Start front spots on black, offset on white, straight approach, NO BARCODE/other side marks.");
    puts("f=hold GP21 USB; g=hands-free USB2s countdown. s/either stop button; f also stops on GP21 release.");
    puts("BANK: hold BOTH3s, release BOTH, wait1s, tap/release GP21,2s countdown. Either button stops.");
    puts("Repeat after settling/save, both released and stationary1s. No automatic restart.");
    puts("Approx geometry: side92mm ahead of rear-wheel axle; measured stripe midpoint+92mm centres axle.");
    puts("Mode2 reverses LEFT wheel, RIGHT forward. Mode3 leaves LEFT unpowered and needs longer outgoing line.");
    puts("Mode3: leave old line; either>=400 for30ms at75..110deg starts phase9 slow alignment.");
    puts("Forward PWM cap25%, turn sustained35%, one start pulse40% up to150ms (1ms timer clamp).");
    puts("V6: phase9 forward20..35mm/s,25% cap; both>=400/difference<=350 for80ms starts exit20mm.");
    puts("Approach<=450mm/12s; whole run<=25s/900mm. No voltage or motor speed target increase.");
    puts("Mode3 outgoing black needs>=220mm; prefer300mm for alignment room. No barcode/other offset marks.");
    puts("p=dump saved log, h=help. Wait3s after stop before switching power; never use two sources together.");
    printf("CAL white=%u,%u black=%u,%u valid=%u side_white=%u side_black=%u side_mask=%u side_valid=%u mode=%u restored=%u imported_v11=%u\n",
        calibration.white[0],calibration.white[1],calibration.black[0],calibration.black[1],calibration_valid(&calibration),
        side_cal.white,side_cal.black,side_cal.mask,side_valid(&side_cal),test_mode,flash_loaded,imported_front);
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
    r->ms=(uint32_t)((now-guard.start_us)/1000);
    r->phase=junction.phase;r->side=(uint8_t)gpio_get(19);
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
    if(guard.active) { puts("Dump deferred: stop before p.");return; }
    if(!trace_count) { puts("No junction run saved yet.");return; }
    if(flash_loaded) puts("RESTORED: CRC-verified junction test log.");
    if(save_failed) puts("WARNING: flash save failed; RAM log only.");
    printf("JUNCTION LEFT v%u: key=%c mode=%s test=%u rows=%u dropped=%u settling=%u\n",recorded_version,recorded_key,
        recorded_offline?"offline":recorded_key=='g'?"USB hands-free":"USB hold",recorded_mode,trace_count,trace_dropped,trace_settling);
    printf("CAL white=%u,%u black=%u,%u SIDE white=%u black=%u | side_ahead_mm=92\n",
        calibration.white[0],calibration.white[1],calibration.black[0],calibration.black[1],side_cal.white,side_cal.black);
    puts("ms,active,phase,side_raw,L_mm,R_mm,progress_mm,heading_deg,L_mm_s,R_mm_s,PWM_L,PWM_R,ADC_L,ADC_R,levelL,levelR,targetL,targetR,decision");
    for(unsigned i=0;i<trace_count;++i) {
        Trace *r=&trace[i];float l=raw_to_mm(0,r->ticks[0]),rr=raw_to_mm(1,r->ticks[1]);
        printf("%lu,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%u,%u,%u,%u,%d,%d,%u\n",
            (unsigned long)r->ms,r->active,r->phase,r->side,(double)l,(double)rr,(double)((l+rr)*0.5f),
            (double)heading_deg(l,rr),(double)(r->velocity10[0]*0.1f),(double)(r->velocity10[1]*0.1f),
            r->pwm[0],r->pwm[1],r->adc[0],r->adc[1],r->level[0],r->level[1],r->target[0],r->target[1],r->decision);
    }
    printf("JUNCTION: phase=%u(%s) leading_mm=%.2f trailing_mm=%.2f stripe_mm=%.2f axle_target_mm=%.2f left_rotation_deg=%.2f\n",
        junction.phase,phase_name(junction.phase),(double)junction.leading_mm,(double)junction.trailing_mm,
        (double)(junction.trailing_mm-junction.leading_mm),(double)junction.centre_mm,(double)junction.turn_degrees);
    printf("TURN: style=%s start_before_cross_mm=%.1f (axle_target is stop/turn-start goal)\n",
        junction.rolling?"rolling left":"centre/in-place",(double)j_turn_start_offset(&junction));
    float l=raw_to_mm(0,(int64_t)cut_ticks[0]-origin[0]),r=raw_to_mm(1,(int64_t)cut_ticks[1]-origin[1]);
    printf("STOP: %s | run_ms=%" PRIu64 " L_mm=%.2f R_mm=%.2f heading_deg=%.2f invalid_L=%lu invalid_R=%lu\n",
        reason_name(guard.reason),guard.duration_us/1000,(double)l,(double)r,(double)heading_deg(l,r),
        (unsigned long)cut_invalid[0],(unsigned long)cut_invalid[1]);
    puts("END JUNCTION LOG. Heading/distance are encoder estimates, not external measurements.");
}
static unsigned start_block(unsigned mode) {
    unsigned why=profile_start_block(5,guard.active,trace_settling,calibration_valid(&calibration),!gpio_get(STOP),
        rearm.ready,line_level(analog_raw[0],calibration.white[0],calibration.black[0]),
        line_level(analog_raw[1],calibration.white[1],calibration.black[1]),!gpio_get(HOLD),mode);
    if(why) return why;
    if(!side_valid(&side_cal)) return 20;
    if(gpio_get(19)!=side_cal.white) return 21;
    return 0;
}

static bool start_run(unsigned mode) {
    last_start_block=start_block(mode);if(last_start_block) return false;
    uint64_t now=time_us_64();uint32_t irq=save_and_disable_interrupts();
    bool ok=guard_start(&guard,now,mode!=LINE_USB_HOLD || !gpio_get(HOLD),!gpio_get(STOP));
    if(ok) {
        ++runs;rearm=(LineRearm){0};run_mode=mode;recorded_mode=test_mode;recorded_version=6;
        guard.deadline_us=now+J_RUN_US;trace_interval_us=J_TRACE_US;
        for(unsigned w=0;w<2;++w) {
            origin[w]=previous[w]=enc[w].position;initial_invalid[w]=enc[w].invalid;
            velocity[w]=0;output[w]=0;peak_until[w]=0;controller[w]=(SignedWheel){0};
            bad_direction_us[w]=no_motion_us[w]=0;
        }
        path_mm=0;
        JunctionInput input={.now=now,.adc={analog_raw[0],analog_raw[1]},.side_black=gpio_get(19)==side_cal.black};
        j_begin_mode(&junction,test_mode,&input);demand=junction_step(&junction,&calibration,&input);
        outputs_off();trace_count=trace_dropped=0;
        recorded_key=mode==LINE_USB_FREE?'g':'f';recorded_offline=mode==LINE_BANK;trace_settling=false;
        flash_loaded=false;save_failed=false;last_control=last_trace=now;
    }
    restore_interrupts(irq);if(!ok) last_start_block=START_GUARD;
    if(ok) add_trace(now);
    return ok;
}

static void report_start_block(void) {
    const char *text="check calibration/buttons/readiness/front sensors";
    if(last_start_block==20) text="capture offset WHITE with o, BLACK with b; readings must differ";
    else if(last_start_block==21) text="offset sensor must start on WHITE, before the cross";
    else if(last_start_block==START_REARM) text="release both; wait for save and1s standstill";
    else if(last_start_block==START_LINE_ABSENT) text="front sensors must see black (either level>=650)";
    else if(last_start_block==START_HOLD_GP21) text="hold GP21 for USB f";
    else if(last_start_block==START_RELEASE_GP21) text="release GP21 for hands-free";
    printf("START rejected: %s | reason=%u ready=%u side_raw=%u side_valid=%u\n",text,last_start_block,rearm.ready,gpio_get(19),side_valid(&side_cal));
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

    JunctionInput input={.now=now,.speed={velocity[0],velocity[1]},.path=path_mm,
        .adc={(uint16_t)adc_filtered[0],(uint16_t)adc_filtered[1]},.side_black=gpio_get(19)==side_cal.black};
    for(unsigned w=0;w<2;++w) input.wheel[w]=raw_to_mm(w,(int64_t)e[w].position-origin[w]);
    demand=junction_step(&junction,&calibration,&input);
    uint8_t failure=demand.stop?junction.reason:WHY_NONE;int32_t next[2];
    bool turning=junction.phase==J_TURN && demand.target[1]>0 &&
                 (junction.rolling?demand.target[0]==0:demand.target[0]<0);
    for(unsigned w=0;w<2;++w) {
        int sign=demand.target[w]>0?1:demand.target[w]<0?-1:0;
        next[w]=junction_wheel_step(&controller[w],w,demand.target[w],velocity[w],dt,now,turning);
        float along=velocity[w]*sign;
        bad_direction_us[w]=sign && along<-20?bad_direction_us[w]+elapsed:0;
        if(bad_direction_us[w]>=50000) failure=WHY_DIRECTION;
        no_motion_us[w]=abs(demand.target[w])>=20 && controller[w].pi.ramp>=20 && llabs(delta[w])<2?no_motion_us[w]+elapsed:0;
        if(no_motion_us[w]>=500000) failure=WHY_FEEDBACK;
        if(e[w].invalid-initial_invalid[w]>=8) failure=WHY_ENCODER;
    }
    irq=save_and_disable_interrupts();
    if(guard.active && guard.generation==generation) {
        if(failure) kill(now,failure);
        else {
            for(unsigned w=0;w<2;++w) {
                peak_until[w]=turning?controller[w].peak_end_us:0;
                output[w]=junction_peak_cap(next[w],peak_until[w],time_us_64());
                DualLevels v=junction_levels(w,output[w],turning);
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
    gpio_init(19);gpio_set_dir(19,GPIO_IN);gpio_pull_up(19);
    adc_init();adc_gpio_init(26);adc_gpio_init(28);
    analog_raw[0]=read_adc(0);analog_raw[1]=read_adc(2);
    adc_filtered[0]=analog_raw[0];adc_filtered[1]=analog_raw[1];
    stdio_init_all();load_log();last_control=time_us_64();
    bool old_connected=false,usb_pending=false;uint8_t raw_buttons=0,buttons=0,last_peak_mask=0;
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
        else if(ch=='o' || ch=='b') {
            usb_pending=false;
            if(guard.active || trace_settling || hold || stop) puts("SIDE CAL rejected: stop/release/wait for save.");
            else {
                trace_count=trace_dropped=0;flash_loaded=false;
                if(ch=='o') { side_cal.white=(uint8_t)gpio_get(19);side_cal.mask=1; }
                else if(!(side_cal.mask&1)) puts("Capture side WHITE with o first.");
                else { side_cal.black=(uint8_t)gpio_get(19);side_cal.mask|=2; }
                help();
            }
        } else if(ch=='k' || ch=='1' || ch=='2' || ch=='3') {
            usb_pending=false;
            if(guard.active || trace_settling || hold || stop) puts("CONFIG rejected: stop/release both buttons.");
            else if(ch=='k') { save_log();puts(save_failed?"SAVE FAILED: need valid front + differing side captures":"CALIBRATION/MODE SAVED."); }
            else { test_mode=(uint8_t)(ch-'0');printf("TEST %u selected; k saves mode.\n",test_mode); }
        } else if(ch=='f') {
            usb_pending=false;bool ok=start_run(LINE_USB_HOLD);
            if(ok) puts("FOLLOW accepted: USB hold GP21.");else report_start_block();
        } else if(ch=='g') {
            if(usb_pending) puts("USB countdown already pending; s/either button cancels.");
            else {
                last_start_block=start_block(LINE_USB_FREE);
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
        uint8_t before_phase=junction.phase;
        control_step(time_us_64());now=time_us_64();
        uint8_t peak_mask=(output[0]>J_TURN_PWM_LIMIT || output[0]<-J_TURN_PWM_LIMIT?1:0)
                        |(output[1]>J_TURN_PWM_LIMIT || output[1]<-J_TURN_PWM_LIMIT?2:0);
        if(guard.active && (junction.phase!=before_phase || peak_mask!=last_peak_mask)) {
            add_trace(now);last_trace=now;
            if(connected && junction.phase!=before_phase) printf("PHASE %u: %s\n",junction.phase,phase_name(junction.phase));
        }
        last_peak_mask=peak_mask;
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

            printf("JUNCTION: ADC=%u,%u levels=%u,%u side_raw=%u side_black=%u side_valid=%u mode=%u phase=%u(%s) active=%u targets=%d,%d PWM=%ld,%ld arm=%u ready=%u pending=%u runs=%lu\n",
                analog_raw[0],analog_raw[1],nl,nr,gpio_get(19),side_valid(&side_cal) && gpio_get(19)==side_cal.black,
                side_valid(&side_cal),test_mode,junction.phase,phase_name(junction.phase),guard.active,
                guard.active?demand.target[0]:0,guard.active?demand.target[1]:0,
                guard.active?(long)output[0]:0,guard.active?(long)output[1]:0,arm.state,rearm.ready,usb_pending,(unsigned long)runs);
            last_status=now;
        }
        sleep_ms(1);
    }
}
