/* Continuous bare-line circuit: follow bends, take every qualified cross LEFT. */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "../left_motor_bench/bench_logic.h"
#include "../line_motion_integration/logic.h"
#include "circuit_wheel.h"
#include "circuit_record.h"
#include "circuit_capture.h"
#include "circuit_feedback.h"

enum { HOLD=21,STOP=20,PWM_TOP=999 };
static const uint ma[2]={8,10},mb[2]={9,11},ea[2]={0,4},eb[2]={1,5};
static volatile BenchEncoder enc[2];
static volatile DualGuard guard={.consumed=true};
static volatile unsigned run_mode=LINE_USB_HOLD;
static volatile int64_t total_ticks[2],cut_ticks[2];
static volatile uint32_t initial_invalid[2],cut_invalid[2];
static volatile int32_t output[2];
static volatile uint64_t peak_until[2];
static LineCalibration calibration;
static SideCalibration side_cal;
static Circuit circuit;
static uint8_t test_mode=1,recorded_mode=1;
static uint8_t recorded_version=3;
static bool last_fast;
static CircuitCapture capture;
static CircuitFeedback feedback[2];
static int capture_outer=-1;
static AnalogDemand demand;
static SignedWheel controller[2];
static LineRearm rearm;
static uint16_t analog_raw[2];
static float adc_filtered[2],velocity[2];
static uint64_t bad_direction_us[2],last_control,last_trace,stopped_us;
static unsigned last_start_block;
static uint32_t runs;
static char recorded_key='-';
static bool recorded_offline,imported_front,flash_loaded,save_failed,trace_settling;
static CircuitTrace trace[C_ROWS];
static CircuitSummary summaries[C_SUMMARIES];
static uint32_t trace_count,trace_head,trace_dropped,summary_count,summary_head;
#define LOG_OFFSET (PICO_FLASH_SIZE_BYTES-2*C_STORE_BYTES)
extern char __flash_binary_end;
static union { CircuitSaved data;uint8_t bytes[C_STORE_BYTES]; } saved;

static double wheel_mm(unsigned w,int64_t ticks) {
    return (double)ticks*(double)mm_per_tick(w)*(w==0?-1.0:1.0);
}
static double path_mm(const int64_t ticks[2]) {
    return (fabs(wheel_mm(0,ticks[0]))+fabs(wheel_mm(1,ticks[1])))*0.5;
}
static int32_t trace_mm10(double mm) {
    return mm>214748364.0?INT32_MAX:mm < -214748364.0?INT32_MIN:(int32_t)(mm*10.0);
}
static uint32_t trace_path10(double mm) { return mm>429496729.0?UINT32_MAX:(uint32_t)(mm*10.0); }
static void outputs_off(void) {
    for(unsigned w=0;w<2;++w) pwm_set_both_levels(pwm_gpio_to_slice_num(ma[w]),0,0);
}
/* IRQs masked, or called by the safety ISR. Raw position is drained each tick
   into64-bit counters so long circuits cannot saturate the shared encoder. */
static void capture_cutoff(void) {
    for(unsigned w=0;w<2;++w) {
        cut_ticks[w]=total_ticks[w]+enc[w].position;
        cut_invalid[w]=enc[w].invalid-initial_invalid[w];
    }
}
static void kill(uint64_t now,uint8_t reason) {
    bool active=guard.active;guard_stop(&guard,now,reason);outputs_off();
    if(active) capture_cutoff();
}
static uint8_t read_ab(unsigned w) {
    uint32_t p=gpio_get_all();return (uint8_t)((((p>>ea[w])&1u)<<1)|((p>>eb[w])&1u));
}
static void encoder_irq(uint pin,uint32_t events) {
    (void)events;unsigned w=(pin==ea[0] || pin==eb[0])?0:1;
    bench_encoder_update(&enc[w],read_ab(w));
}
static bool safety_tick(struct repeating_timer *t) {
    (void)t;bool active=guard.active;uint64_t now=time_us_64();
    guard_tick(&guard,now,run_mode==LINE_USB_HOLD?!gpio_get(HOLD):true,!gpio_get(STOP));
    if(guard.active && run_mode!=LINE_USB_HOLD && !gpio_get(HOLD)) guard_stop(&guard,now,WHY_BUTTON);
    if(!guard.active) { outputs_off();if(active) capture_cutoff(); }
    else for(unsigned w=0;w<2;++w) {
        int32_t capped=boosted_junction_peak_cap(output[w],peak_until[w],now);
        if(capped!=output[w]) {
            output[w]=capped;DualLevels v=circuit_levels(w,capped,true,recorded_mode);
            pwm_set_both_levels(pwm_gpio_to_slice_num(ma[w]),v.a,v.b);
        }
    }
    return true;
}
static const char *reason_name(unsigned r) {
    switch(r) {
    case WHY_NONE:return "none";case WHY_GP20:return "GP20 pressed";
    case WHY_RELEASE:return "GP21 released";case WHY_SERIAL:return "serial s";
    case WHY_USB:return "USB connection changed";case WHY_LATE:return "control loop late";
    case WHY_FEEDBACK:return recorded_version>=3?"no forward encoder progress for750ms under drive":"no encoder movement under drive";
    case WHY_DIRECTION:return "encoder direction mismatch";case WHY_ENCODER:return "invalid encoder transitions";
    case WHY_BUTTON:return "GP21 stop pressed";case J_BAD_STRIPE:return "cross-arm width or centring timeout";
    case J_CENTRE_DRIFT:return "centring drift / rolling inner-wheel movement";
    case J_TURN_TIMEOUT:return "turn timeout";case J_BRANCH_MISSING:return "left branch not confirmed";
    case J_PATH_LIMIT:return "junction manoeuvre path limit";case J_LINE_LOST:return "line not recovered";
    case J_ALIGN_TIMEOUT:return "outgoing-line alignment timeout";
    case C_APPROACH_FAST:return "junction approach too fast; use a slower profile";
    case C_BAD_PROFILE:return "invalid profile";
    case C_LOSS_NO_DIRECTION:return "line not recovered: no recent direction";
    case C_LOSS_TIME:return "line not recovered:2s recovery expired";
    case C_LOSS_DISTANCE:return "line not recovered:50mm recovery limit";
    default:return "other guard";
    }
}
static const char *phase_name(unsigned p) {
    return p==C_FOLLOW?"circuit follow":p==J_CENTRE?"cross approach":p==J_TURN?"rolling left":
           p==J_ACQUIRE?"align outgoing line":p==J_FAULT?"fault":"idle";
}
static bool storage_fits(void) { return (uintptr_t)&__flash_binary_end<=XIP_BASE+LOG_OFFSET; }
static void load_log(void) {
    if(!storage_fits()) return;
    const CircuitSaved *v=(const CircuitSaved *)(XIP_BASE+LOG_OFFSET);
    if(circuit_record_valid(v)) {
        memcpy(trace,v->rows,sizeof(trace));memcpy(summaries,v->summaries,sizeof(summaries));
        trace_count=v->count;trace_head=trace_count%C_ROWS;trace_dropped=v->overwritten;
        summary_count=v->summary_count;summary_head=summary_count%C_SUMMARIES;
        recorded_key=v->key;recorded_offline=v->offline;test_mode=v->profile;recorded_mode=v->recorded_profile;
        recorded_version=(uint8_t)v->version;
        guard.duration_us=v->duration_us;guard.reason=v->reason;
        for(unsigned w=0;w<2;++w) { cut_ticks[w]=v->ticks[w];cut_invalid[w]=v->invalid[w]; }
        calibration=v->calibration;side_cal=v->side;imported_front=v->imported;
        circuit.turns=v->turns;circuit.ignored=v->ignored;flash_loaded=true;
    } else imported_front=circuit_import((const CircuitLegacy *)(XIP_BASE+PICO_FLASH_SIZE_BYTES-FLASH_SECTOR_SIZE),
                                         &calibration,&side_cal);
}
static void save_log(void) {
    if(guard.active || !calibration_valid(&calibration) || !side_valid(&side_cal) || !storage_fits()) {
        save_failed=true;return;
    }
    outputs_off();memset(&saved,0,sizeof(saved));CircuitSaved *v=&saved.data;
    v->magic=C_MAGIC;v->version=trace_count?recorded_version:3;v->count=trace_count;v->summary_count=summary_count;v->overwritten=trace_dropped;
    v->duration_us=guard.duration_us;v->reason=guard.reason;v->key=recorded_key;v->offline=recorded_offline;
    v->profile=test_mode;v->recorded_profile=recorded_mode;v->imported=imported_front;
    v->calibration=calibration;v->side=side_cal;v->turns=circuit.turns;v->ignored=circuit.ignored;
    int64_t ticks[2];for(unsigned w=0;w<2;++w) { ticks[w]=v->ticks[w]=cut_ticks[w];v->invalid[w]=cut_invalid[w]; }
    v->path10=(uint64_t)(path_mm(ticks)*10.0);
    for(unsigned n=0;n<trace_count;++n) v->rows[n]=trace[(trace_head+C_ROWS-trace_count+n)%C_ROWS];
    for(unsigned n=0;n<summary_count;++n) v->summaries[n]=summaries[(summary_head+C_SUMMARIES-summary_count+n)%C_SUMMARIES];
    v->crc=log_crc((const uint8_t *)v+offsetof(CircuitSaved,count),sizeof(*v)-offsetof(CircuitSaved,count));
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(LOG_OFFSET,C_STORE_BYTES);flash_range_program(LOG_OFFSET,saved.bytes,C_STORE_BYTES);
    restore_interrupts(irq);
    const CircuitSaved *written=(const CircuitSaved *)(XIP_BASE+LOG_OFFSET);
    save_failed=!circuit_record_valid(written) || memcmp(written,v,sizeof(*v))!=0;flash_loaded=false;
}
static void reset_log(void) {
    trace_count=trace_head=trace_dropped=summary_count=summary_head=0;
    guard.duration_us=0;guard.reason=0;circuit=(Circuit){0};
    for(unsigned w=0;w<2;++w) { cut_ticks[w]=0;cut_invalid[w]=0; }
    flash_loaded=false;recorded_key='-';recorded_version=3;
}
static void help(void) {
    puts("\nBUDDY23 CIRCUIT LEFT v3 | NO BARCODE/IMU/HUMP | motors OFF on boot");
    puts("Every qualified straight cross -> rolling LEFT -> align -> follow indefinitely.");
    puts("Profiles1/2/3 cruise45/90/135mm/s only when both>=400/diff<=80 for300ms.");
    puts("Correction/recovery targets and30% forward cap match profile1. Fast cruise caps30/36/42%.");
    puts("Turn45/55/65 then30/35/40;42% sustained,48% start pulse<=150ms. Test profile1 FIRST.");
    puts("Branch handover: normal curve following immediately; either>=400 for150ms confirms completion.");
    puts("V3: both-white recovery<=2s/50mm; direction memory1.5s; weak>=200/diff>=150 consistent30ms refreshes it.");
    puts("Post-turn visible near-pivot: OUTER wheel turn effort<=42%, no fresh peak, at most3s/45mm.");
    puts("Feedback:4 accumulated forward ticks (~0.32mm) renew progress;750ms without progress stops.");
    puts("w=both front WHITE, l=left BLACK, r=right BLACK; o=side WHITE,b=side BLACK;k=save.");
    puts("1/2/3 selects profile; k saves. Front A0 GP26/GP28; side D0 GP19;3.3V/common GND.");
    puts("Start facing along line, front sees black, side on white, straight gap before cross.");
    puts("f=USB hold GP21; g=USB hands-free2s countdown; s/either button stops (f releases GP21).");
    puts("BANK: wait5s; BOTH3s; release BOTH; wait1s; tap/release GP21;2s countdown.");
    puts("No time/lap stop; line-loss/stall/turn/control guards remain. No barcode/other side marks.");
    puts("Side edge gate: white100ms,straight300ms; after turn clear150mm before rearming.");
    puts("p=log+whole-run summary. Latest5min detail/latest64 turns retained; totals cover whole run.");
    puts("Offline: STOP with either button, WAIT5s before removing power. Then USB,p.");
    printf("CAL white=%u,%u black=%u,%u valid=%u side_white=%u side_black=%u side_valid=%u profile=%u restored=%u imported_junction=%u\n",
        calibration.white[0],calibration.white[1],calibration.black[0],calibration.black[1],calibration_valid(&calibration),
        side_cal.white,side_cal.black,side_valid(&side_cal),test_mode,flash_loaded,imported_front);
}
static uint16_t read_adc(unsigned channel) {
    adc_select_input(channel);(void)adc_read();uint32_t sum=0;
    for(unsigned n=0;n<32;++n) { sum+=adc_read();sleep_us(20); }
    return (uint16_t)((sum+16)/32);
}
static void calibrate(int key) {
    if(guard.active || trace_settling || !gpio_get(HOLD) || !gpio_get(STOP)) {
        puts("CAL rejected: stop/release/wait for save.");return;
    }
    if(key!='w' && !(calibration.mask&1)) { puts("Capture BOTH WHITE with w first.");return; }
    reset_log();
    if(key=='w') { calibration.white[0]=analog_raw[0];calibration.white[1]=analog_raw[1];calibration.mask=1; }
    else if(key=='l') { calibration.black[0]=analog_raw[0];calibration.mask|=2; }
    else { calibration.black[1]=analog_raw[1];calibration.mask|=4; }
    help();
}
static void add_trace(uint64_t now) {
    CircuitTrace *r=&trace[trace_head];memset(r,0,sizeof(*r));
    uint32_t irq=save_and_disable_interrupts();bool active=guard.active;
    int64_t ticks[2];for(unsigned w=0;w<2;++w) ticks[w]=active?total_ticks[w]+enc[w].position:cut_ticks[w];
    r->ms=(now-guard.start_us)/1000;r->active=active;
    for(unsigned w=0;w<2;++w) r->pwm[w]=active?(int16_t)output[w]:0;
    restore_interrupts(irq);
    r->path10=trace_path10(path_mm(ticks));r->turns=circuit.turns;r->phase=circuit.phase;
    r->side=(uint8_t)gpio_get(19);
    r->decision=(uint8_t)(demand.decision | (active && capture_outer>=0?(capture_outer+1)<<5:0));
    for(unsigned w=0;w<2;++w) {
        r->wheel10[w]=trace_mm10(wheel_mm(w,ticks[w]));r->speed10[w]=(int16_t)(velocity[w]*10);
        r->adc[w]=analog_raw[w];r->level[w]=demand.level[w];r->target[w]=active?demand.target[w]:0;
    }
    trace_head=(trace_head+1)%C_ROWS;
    if(trace_count<C_ROWS) ++trace_count;else if(trace_dropped<UINT32_MAX) ++trace_dropped;
}
static void add_summary(uint64_t now,double path) {
    summaries[summary_head]=(CircuitSummary){.ms=(now-guard.start_us)/1000,.path10=trace_path10(path),
        .number=circuit.turns,.follow_ms=circuit.follow_ms,.centre_ms=circuit.centre_ms,
        .turn_ms=circuit.turn_ms,.align_ms=circuit.align_ms,.angle10=(int16_t)(circuit.turn.turn_degrees*10)};
    summary_head=(summary_head+1)%C_SUMMARIES;if(summary_count<C_SUMMARIES) ++summary_count;
}
static void print_summary(void) {
    int64_t ticks[2]={cut_ticks[0],cut_ticks[1]};double distance=path_mm(ticks),seconds=(double)guard.duration_us/1e6;
    printf("SUMMARY: total_time_s=%.3f total_distance_mm=%.1f average_speed_mm_s=%.2f completed_lefts=%lu four_turn_cycles=%lu ignored_side_edges=%lu invalid_L=%lu invalid_R=%lu\n",
        seconds,distance,seconds>0?distance/seconds:0,(unsigned long)circuit.turns,(unsigned long)(circuit.turns/4),
        (unsigned long)circuit.ignored,(unsigned long)cut_invalid[0],(unsigned long)cut_invalid[1]);
}
static void dump(void) {
    if(guard.active || trace_settling) { puts("Stop and wait for RUN SAVED before p.");return; }
    if(!trace_count) { puts("No circuit run saved yet.");return; }
    if(flash_loaded) puts("RESTORED: CRC-verified circuit calibration/log.");
    if(save_failed) puts("FLASH SAVE FAILED: RAM log only.");
    printf("CIRCUIT LEFT v%u: key=%c mode=%s profile=%u rows=%lu overwritten=%lu turn_summaries=%lu\n",
        recorded_version,recorded_key,recorded_offline?"offline":recorded_key=='g'?"USB hands-free":"USB hold",recorded_mode,
        (unsigned long)trace_count,(unsigned long)trace_dropped,(unsigned long)summary_count);
    puts("ms,active,phase,side_raw,distance_mm,L_mm,R_mm,L_mm_s,R_mm_s,PWM_L,PWM_R,ADC_L,ADC_R,levelL,levelR,targetL,targetR,decision,lefts,assist");
    for(unsigned n=0;n<trace_count;++n) {
        CircuitTrace *r=&trace[(trace_head+C_ROWS-trace_count+n)%C_ROWS];
        printf("%" PRIu64 ",%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%u,%u,%u,%u,%d,%d,%u,%lu,%u\n",
            r->ms,r->active,r->phase,r->side,(double)r->path10/10,(double)r->wheel10[0]/10,(double)r->wheel10[1]/10,
            (double)r->speed10[0]/10,(double)r->speed10[1]/10,r->pwm[0],r->pwm[1],r->adc[0],r->adc[1],
            r->level[0],r->level[1],r->target[0],r->target[1],r->decision&31,(unsigned long)r->turns,r->decision>>5);
    }
    puts("TURN_SUMMARIES: number,elapsed_ms,distance_mm,follow_ms,centre_ms,turn_ms,alignment_ms,rotation_deg");
    for(unsigned n=0;n<summary_count;++n) {
        CircuitSummary *s=&summaries[(summary_head+C_SUMMARIES-summary_count+n)%C_SUMMARIES];
        printf("%lu,%" PRIu64 ",%.1f,%lu,%u,%u,%u,%.1f\n",(unsigned long)s->number,s->ms,(double)s->path10/10,
            (unsigned long)s->follow_ms,s->centre_ms,s->turn_ms,s->align_ms,(double)s->angle10/10);
    }
    printf("STOP: %s | run_ms=%" PRIu64 "\n",reason_name(guard.reason),guard.duration_us/1000);print_summary();
    puts("END CIRCUIT LOG. Distance/speed are encoder estimates; four turns is a route cycle, not a start marker.");
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
        ++runs;rearm=(LineRearm){0};run_mode=mode;recorded_mode=test_mode;recorded_version=3;last_fast=false;
        capture=(CircuitCapture){0};capture_outer=-1;
        guard.deadline_us=UINT64_MAX;guard.duration_us=0;
        for(unsigned w=0;w<2;++w) {
            enc[w].position=0;total_ticks[w]=cut_ticks[w]=0;initial_invalid[w]=enc[w].invalid;cut_invalid[w]=0;
            velocity[w]=0;output[w]=0;peak_until[w]=0;controller[w]=(SignedWheel){0};
            bad_direction_us[w]=0;feedback[w]=(CircuitFeedback){0};
        }
        CircuitInput input={.now=now,.adc={analog_raw[0],analog_raw[1]},.side_black=false};
        circuit_begin(&circuit,&input);demand=circuit_step(&circuit,&calibration,&input,recorded_mode);
        outputs_off();trace_count=trace_head=trace_dropped=summary_count=summary_head=0;
        recorded_key=mode==LINE_USB_FREE?'g':'f';recorded_offline=mode==LINE_BANK;
        trace_settling=false;flash_loaded=save_failed=false;last_control=last_trace=now;
    }
    restore_interrupts(irq);if(!ok) last_start_block=START_GUARD;
    if(ok) add_trace(now);
    return ok;
}
static void report_start_block(void) {
    printf("START rejected: reason=%u ready=%u side_valid=%u side_raw=%u. Need valid front/side calibration, front black>=650, side WHITE, buttons released (hold GP21 for f).\n",
        last_start_block,rearm.ready,side_valid(&side_cal),gpio_get(19));
}
static void control_step(uint64_t now) {
    uint64_t elapsed=now-last_control;if(elapsed<CONTROL_PERIOD_US) return;last_control=now;
    uint32_t irq=save_and_disable_interrupts();bool active=guard.active;uint32_t generation=guard.generation;
    BenchEncoder e[2]={enc[0],enc[1]};int64_t ticks[2];
    for(unsigned w=0;w<2;++w) {
        enc[w].position=0;if(active) total_ticks[w]+=e[w].position;ticks[w]=total_ticks[w];
    }
    restore_interrupts(irq);float dt=(float)elapsed/1e6f;
    for(unsigned w=0;w<2;++w) velocity[w]+=0.25f*(raw_to_mm(w,e[w].position)/dt-velocity[w]);
    if(!active) return;
    CircuitInput input={.now=now,.wheel={wheel_mm(0,ticks[0]),wheel_mm(1,ticks[1])},.path=path_mm(ticks),
        .speed={velocity[0],velocity[1]},.adc={(uint16_t)adc_filtered[0],(uint16_t)adc_filtered[1]},
        .side_black=gpio_get(19)==side_cal.black};
    unsigned previous_phase=circuit.phase;
    demand=circuit_step(&circuit,&calibration,&input,recorded_mode);
    if(previous_phase==J_TURN && circuit.phase==J_ACQUIRE) circuit_capture_begin(&capture,now,input.path);
    if(circuit.completed) add_summary(now,input.path);
    uint8_t failure=demand.stop?circuit.reason:WHY_NONE;int32_t next[2];
    bool turning=circuit.phase==J_TURN && demand.target[0]==0 && demand.target[1]>0;
    unsigned wheel_profile=circuit.fast?recorded_mode:1;
    int previous_outer=capture_outer;
    capture_outer=circuit_capture_outer(&capture,&input,circuit.phase,&demand);
    bool loaded[2]={turning || capture_outer==0,turning || capture_outer==1};
    if(capture_outer>=0 && capture_outer!=previous_outer) circuit_capture_prepare(&controller[capture_outer]);
    if(last_fast && !circuit.fast) for(unsigned w=0;w<2;++w) circuit_leave_fast(&controller[w]);
    last_fast=circuit.fast;
    for(unsigned w=0;w<2;++w) {
        next[w]=circuit_wheel_step(&controller[w],w,demand.target[w],velocity[w],dt,now,loaded[w],wheel_profile);
        bad_direction_us[w]=demand.target[w]>0 && velocity[w]<-20?bad_direction_us[w]+elapsed:0;
        if(bad_direction_us[w]>=50000) failure=WHY_DIRECTION;
        if(circuit_stalled(&feedback[w],now,w==0?-ticks[w]:ticks[w],demand.target[w],controller[w].pi.ramp)) failure=WHY_FEEDBACK;
        if(e[w].invalid-initial_invalid[w]>=8) failure=WHY_ENCODER;
    }
    irq=save_and_disable_interrupts();
    if(guard.active && guard.generation==generation) {
        if(failure) kill(now,failure);
        else {
            for(unsigned w=0;w<2;++w) {
                peak_until[w]=turning?controller[w].peak_end_us:0;
                output[w]=boosted_junction_peak_cap(next[w],peak_until[w],time_us_64());
                DualLevels v=circuit_levels(w,output[w],loaded[w],wheel_profile);
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
        else if(ch=='o' || ch=='b') {
            usb_pending=false;
            if(guard.active || trace_settling || hold || stop) puts("SIDE CAL rejected: stop/release/wait for save.");
            else {
                reset_log();
                if(ch=='o') { side_cal.white=(uint8_t)gpio_get(19);side_cal.mask=1; }
                else if(!(side_cal.mask&1)) puts("Capture side WHITE with o first.");
                else { side_cal.black=(uint8_t)gpio_get(19);side_cal.mask|=2; }
                help();
            }
        } else if(ch=='k' || ch=='1' || ch=='2' || ch=='3') {
            usb_pending=false;
            if(guard.active || trace_settling || hold || stop) puts("CONFIG rejected: stop/release both buttons.");
            else if(ch=='k') { save_log();puts(save_failed?"SAVE FAILED: need valid front + differing side captures":"CALIBRATION/PROFILE SAVED."); }
            else { test_mode=(uint8_t)(ch-'0');printf("PROFILE %u selected; k saves profile.\n",test_mode); }
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
        uint8_t before_phase=circuit.phase;
        control_step(time_us_64());now=time_us_64();
        if(connected && guard.active && circuit.phase!=before_phase)
            printf("PHASE %u: %s\n",circuit.phase,phase_name(circuit.phase));
        if(guard.stops!=seen_stops) {
            seen_stops=guard.stops;stopped_us=now;trace_settling=true;run_mode=LINE_USB_HOLD;
            add_trace(now);last_trace=now;
            if(connected) printf("STOP: %s; wait5s then p\n",reason_name(guard.reason));
        }
        if(guard.active || trace_settling) {
            if(now-last_trace>=C_TRACE_US) { add_trace(now);last_trace=now; }
            if(trace_settling && now-stopped_us>=1000000) {
                trace_settling=false;save_log();
                if(connected) { puts(save_failed?"FLASH SAVE FAILED":"RUN SAVED; p prints log.");print_summary(); }
            }
        }
        if(connected && now-last_status>=200000) {
            int64_t status_ticks[2];
            uint32_t irq=save_and_disable_interrupts();
            for(unsigned w=0;w<2;++w) status_ticks[w]=guard.active?total_ticks[w]+enc[w].position:cut_ticks[w];
            uint64_t duration=guard.active?now-guard.start_us:guard.duration_us;
            restore_interrupts(irq);
            double distance=path_mm(status_ticks);
            printf("CIRCUIT: ms=%" PRIu64 " profile=%u phase=%u(%s) active=%u distance_mm=%.1f speed_mm_s=%.1f avg_mm_s=%.1f wheel_mm_s=%.1f,%.1f targets=%d,%d PWM=%ld,%ld ADC=%u,%u levels=%u,%u side=%u turns=%lu assist=%d arm=%u ready=%u pending=%u runs=%lu\n",
                duration/1000,test_mode,circuit.phase,phase_name(circuit.phase),guard.active,distance,
                (double)(velocity[0]+velocity[1])*0.5,duration?distance/((double)duration/1e6):0,
                (double)velocity[0],(double)velocity[1],guard.active?demand.target[0]:0,guard.active?demand.target[1]:0,
                guard.active?(long)output[0]:0,guard.active?(long)output[1]:0,analog_raw[0],analog_raw[1],
                line_level(analog_raw[0],calibration.white[0],calibration.black[0]),
                line_level(analog_raw[1],calibration.white[1],calibration.black[1]),gpio_get(19),
                (unsigned long)circuit.turns,guard.active?capture_outer+1:0,arm.state,rearm.ready,usb_pending,(unsigned long)runs);
            last_status=now;
        }
        sleep_ms(1);
    }
}
