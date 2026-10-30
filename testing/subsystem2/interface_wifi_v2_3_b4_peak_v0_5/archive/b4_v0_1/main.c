/* Circuit + automatic A0/GP27 Code39 threshold search; runtime-mapped A-Z junction actions. */
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
#include "../circuit_left_test/circuit_wheel.h"
#include "barcode_record.h"
#include "barcode_analog.h"
#include "barcode_start.h"
#include "barcode_raw.h"
#include "smooth_follow.h"
#include "../../../../scan_smooth.h"
#include "../../../../../nav_api.h"
#include "../../../../../motion_port.h"
#include "../../../../../buddy_hooks.h"
#include "maneuver_power.h"
#include "../circuit_left_test/circuit_capture.h"
#include "../circuit_left_test/circuit_feedback.h"
#include "../circuit_left_test/circuit_load.h"
#include "../../../../../continuous_drive.h"
#include "../../../../../continuous_follow.h"
#include "../../../../../wifi_transport.h"
#include "../../../../../wifi_packets.h"
#include "../../../../../wifi_protocol.h"
#include "hardware/watchdog.h"
#include "../../../../../successful_calibration.h"

enum { HOLD=21,STOP=20,PWM_TOP=999 };
static const uint ma[2]={8,10},mb[2]={9,11},ea[2]={0,4},eb[2]={1,5};
static volatile BenchEncoder enc[2];
static spin_lock_t *interface_lock;
static void interface_save_nav_log(void);
static void interface_dump_nav_log(void);
static void load_drive_diag(void);
static void save_drive_diag(void);
static void dump_drive_diag(void);
static volatile DualGuard guard={.consumed=true};
static volatile unsigned run_mode=LINE_USB_HOLD;
static volatile int64_t total_ticks[2],cut_ticks[2];
static volatile uint32_t initial_invalid[2],cut_invalid[2];
static volatile int32_t output[2];
static volatile uint64_t peak_until[2];
static volatile bool ramp_output[2],signed_output[2],maneuver_output[2],continuous_output[2];
static ManeuverPower maneuver_power[2];
static volatile uint64_t travelled_ticks[2];
static double restored_distance;
static RampWheel ramp_wheel[2];
static LineCalibration calibration;
static SideCalibration side_cal;
static Circuit circuit;
static BarcodeNav nav;
static BarcodeAnalog side_analog;
static SmoothFollow steering;
static ScanSmooth gentle;
static uint16_t side_adc;static uint64_t last_side_sample;static bool status_enabled;
static uint32_t last_nav_event;
static uint8_t test_mode=1,recorded_mode=1;
static uint8_t recorded_version=28;
static bool last_fast;
static CircuitCapture capture;
static CircuitLoad load;
static unsigned assist_code,feedback_mask;
static uint16_t last_trace_state;
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
static BarcodeTrace trace[B_ROWS];
static BarcodeSummary summaries[C_SUMMARIES];
static uint32_t trace_count,trace_head,trace_dropped,summary_count,summary_head;
#define LOG_OFFSET (PICO_FLASH_SIZE_BYTES-3*C_STORE_BYTES)
#define RAW_OFFSET (PICO_FLASH_SIZE_BYTES-5*C_STORE_BYTES)
_Static_assert(RAW_OFFSET+RAW_BANK_BYTES<=PICO_FLASH_SIZE_BYTES-4*C_STORE_BYTES,"Raw/probe banks overlap");
extern char __flash_binary_end;
static union { BarcodeSaved data;uint8_t bytes[32768]; } saved;
static BarcodeRaw raw_capture;
static bool raw_new,raw_loaded,raw_persisted,raw_save_failed;
static uint32_t wifi_mapping;
static bool wifi_estop,wifi_stopped,wifi_start_pending;
static void wifi_harness_init(void);
static void wifi_harness_poll(uint64_t now);
static bool wifi_console_byte(int ch);
static void wifi_nav_event(const MotionEvent *e);
static void wifi_barcode_events(uint64_t now);


static double wheel_mm(unsigned w,int64_t ticks) {
    return (double)ticks*(double)mm_per_tick(w)*(w==0?-1.0:1.0);
}
static double path_mm(const int64_t ticks[2]) {
    if(recorded_version>=11) {
        uint32_t irq=save_and_disable_interrupts();uint64_t l=travelled_ticks[0],r=travelled_ticks[1];restore_interrupts(irq);
        return restored_distance+(fabs(wheel_mm(0,(int64_t)l))+fabs(wheel_mm(1,(int64_t)r)))*.5;
    }
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
static uintptr_t interface_enter(void *u) {(void)u;return spin_lock_blocking(interface_lock);}
static void interface_leave(void *u,uintptr_t state) {(void)u;spin_unlock(interface_lock,(uint32_t)state);}
static uint64_t interface_now(void *u) {(void)u;return time_us_64();}
static void interface_off(void *u) {
    (void)u;outputs_off();
    for(unsigned w=0;w<2;++w) {output[w]=0;peak_until[w]=0;
        ramp_output[w]=signed_output[w]=maneuver_output[w]=continuous_output[w]=false;}
}
static void capture_cutoff(void) {
    for(unsigned w=0;w<2;++w) {
        cut_ticks[w]=total_ticks[w]+enc[w].position;
        travelled_ticks[w]+=barcode_travel_ticks(enc[w].position);
        cut_invalid[w]=enc[w].invalid-initial_invalid[w];
    }
}
static void kill(uint64_t now,uint8_t reason) {
    wifi_start_pending=false;bool active=guard.active;guard_stop(&guard,now,reason);outputs_off();
    if(active) {capture_cutoff();motion_session_end(reason);}
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
    if(!guard.active) { outputs_off();if(active) {capture_cutoff();motion_session_end(guard.reason);} }
    else for(unsigned w=0;w<2;++w) {
        int32_t capped=continuous_output[w]?continuous_output_cap(output[w]):maneuver_output[w]?maneuver_output_cap(output[w],peak_until[w],now):
            barcode_output_cap(output[w],ramp_output[w],peak_until[w],now);
        if(capped!=output[w]) {
            output[w]=capped;DualLevels v=continuous_output[w]?continuous_output_levels(w,capped):maneuver_output[w]?maneuver_levels(w,capped,signed_output[w]):
                signed_output[w]?boosted_junction_levels(w,capped,true):
                barcode_output_levels(w,capped,ramp_output[w],true,recorded_mode);
            pwm_set_both_levels(pwm_gpio_to_slice_num(ma[w]),v.a,v.b);
        }
    }
    return true;
}
static const char *reason_name(unsigned r) {
    switch(r) {
    case WP_STOP_REASON:return "MQTT stop requested";case WP_ESTOP_REASON:return "MQTT ESTOP latched";
    case WHY_NONE:return "none";case WHY_GP20:return "GP20 pressed";
    case WHY_RELEASE:return "GP21 released";case WHY_SERIAL:return "serial s";
    case WHY_USB:return "USB connection changed";case WHY_LATE:return "control loop late";
    case WHY_FEEDBACK:return recorded_version>=3?"no forward encoder progress for750ms under drive":"no encoder movement under drive";
    case WHY_DIRECTION:return "encoder direction mismatch";case WHY_ENCODER:return "invalid encoder transitions";
    case WHY_BUTTON:return "GP21 stop pressed";case J_BAD_STRIPE:return "cross-arm width or centring timeout";
    case J_CENTRE_DRIFT:return "centring drift / rolling inner-wheel movement";
    case J_TURN_TIMEOUT:return "turn timeout";case J_BRANCH_MISSING:return "requested branch not confirmed";
    case J_PATH_LIMIT:return "junction manoeuvre path limit";case J_LINE_LOST:return "line not recovered";
    case J_ALIGN_TIMEOUT:return "outgoing-line alignment timeout";
    case C_APPROACH_FAST:return "junction approach too fast; use a slower profile";
    case C_BAD_PROFILE:return "invalid profile";
    case C_LOSS_NO_DIRECTION:return "line not recovered: no recent direction";
    case C_LOSS_TIME:return "line not recovered:2s recovery expired";
    case C_LOSS_DISTANCE:return "line not recovered:50mm recovery limit";
    case LS_TIMEOUT:return "line not recovered: five rolling sweeps exhausted";
    case LS_DISTANCE:return "line not recovered:150mm search travel limit";
    case B_EDGE_FAILURE:return "barcode edge capture overflow (legacy digital run)";
    case C_LOSS_STRAIGHT:return "straight gap not confirmed within500ms/15mm";
    default:return "other guard";
    }
}
static const char *phase_name(unsigned p) {
    return p==C_FOLLOW?"circuit follow":p==J_CENTRE?"cross approach":p==J_TURN?(barcode_nav_action(&nav,nav.action)=='U'?"U-turn":barcode_nav_action(&nav,nav.action)=='R'?"rolling right":"rolling left"):
           p==J_TURN_SETTLE?"U-turn settle":p==J_ACQUIRE?"align outgoing line":p==J_FAULT?"fault":"idle";
}
static bool storage_fits(void) { return (uintptr_t)&__flash_binary_end<=XIP_BASE+RAW_OFFSET; }
static void load_raw(void) {
    if(!storage_fits()) return;
    const BarcodeRaw *r=(const BarcodeRaw *)(XIP_BASE+RAW_OFFSET);
    if(barcode_raw_valid(r)) { memcpy(&raw_capture,r,sizeof(raw_capture));raw_loaded=raw_persisted=true; }
}
static void save_raw(void) {
    if(!raw_new || guard.active || !storage_fits()) return;
    int64_t ticks[2]={cut_ticks[0],cut_ticks[1]};
    barcode_raw_finish(&raw_capture,guard.duration_us,ticks,guard.reason);
    if(!wifi_transport_flash_begin()){raw_save_failed=true;return;}
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(RAW_OFFSET,RAW_BANK_BYTES);
    flash_range_program(RAW_OFFSET,(const uint8_t *)&raw_capture,sizeof(raw_capture));
    restore_interrupts(irq);wifi_transport_flash_end();
    const BarcodeRaw *written=(const BarcodeRaw *)(XIP_BASE+RAW_OFFSET);
    raw_save_failed=!barcode_raw_valid(written) || memcmp(written,&raw_capture,sizeof(raw_capture))!=0;
    raw_persisted=!raw_save_failed;raw_new=false;
}
static void dump_raw(void) {
    if(guard.active || trace_settling) { puts("RAW rejected: stop and wait5s before a.");return; }
    if(!barcode_raw_valid(&raw_capture)) { puts("RAW unavailable: make a new WiFi V1 driving run first.");return; }
    const BarcodeRawMeta *m=&raw_capture.header.meta;
    bool matches=recorded_version>=8 && m->duration_us==guard.duration_us &&
        m->ticks[0]==cut_ticks[0] && m->ticks[1]==cut_ticks[1] && m->profile==recorded_mode && m->reason==guard.reason;
    printf("RAW_ADC_BEGIN: rows=%lu capacity=%u stride=12 pin=27 channel=1 data_crc=%08lx duration_us=%" PRIu64 " max_gap_us=%lu full=%u stored=%u restored=%u matches_motion=%u\n",
        (unsigned long)m->count,RAW_CAPACITY,(unsigned long)m->data_crc,m->duration_us,
        (unsigned long)m->max_gap_us,m->full,raw_persisted,raw_loaded,matches);
    puts("sample,us,path_mm,adc");
    for(unsigned n=0;n<m->count;++n) {
        if(!stdio_usb_connected()) return;
        const BarcodeRawRow *r=&raw_capture.rows[n];
        printf("%u,%lu,%.2f,%u\n",n,(unsigned long)r->us,(double)r->path100/100,r->adc);
        stdio_flush();sleep_ms(5);
    }
    printf("RAW_ADC_END: rows=%lu data_crc=%08lx\n",(unsigned long)m->count,(unsigned long)m->data_crc);
}
static void load_log(void) {
    if(!storage_fits()) return;
    const BarcodeSaved *v=(const BarcodeSaved *)(XIP_BASE+LOG_OFFSET);
    uint8_t imported_profile=1;
    if(barcode_import_v25((const uint8_t *)v,&calibration,&imported_profile)) {
        test_mode=recorded_mode=imported_profile;imported_front=true;
        side_cal=(SideCalibration){0,1,3};return;
    }
    if(barcode_record_valid(v)) {
        memcpy(trace,v->rows,sizeof(trace));memcpy(summaries,v->summaries,sizeof(summaries));
        trace_count=v->count;trace_head=trace_count%B_ROWS;trace_dropped=v->overwritten;
        summary_count=v->summary_count;summary_head=summary_count%C_SUMMARIES;
        recorded_key=v->key;recorded_offline=v->offline;test_mode=v->profile;recorded_mode=v->recorded_profile;
        recorded_version=(uint8_t)v->version;guard.duration_us=v->duration_us;guard.reason=v->reason;
        restored_distance=recorded_version>=11?(double)v->path10/10:0;
        for(unsigned w=0;w<2;++w) { cut_ticks[w]=v->ticks[w];cut_invalid[w]=v->invalid[w]; }
        calibration=v->calibration;side_cal=v->side;imported_front=v->imported;
        circuit.turns=v->turns;circuit.ignored=v->ignored;
        memcpy(nav.events,v->events,sizeof(nav.events));nav.event_count=v->event_count;
        nav.event_head=nav.event_count%B_EVENTS;nav.event_overwritten=v->event_overwritten;
        nav.decoded_a=v->decoded_a;nav.decoded_f=v->decoded_f;nav.unsupported=v->unsupported;
        nav.rejected=v->rejected;nav.lefts=v->lefts;nav.rights=v->rights;nav.straight_crosses=v->straight_crosses;
        nav.last_code=(char)v->last_code;nav.decoded_z=recorded_version>=11?v->decoded_z:0;
        nav.uturns=recorded_version>=11?v->uturns:0;nav.decoded_total=v->decoded_total;flash_loaded=true;
        side_cal=(SideCalibration){0,1,3}; /* Legacy record field; no operator side calibration in V4. */
    } else {
        const CircuitSaved *old=(const CircuitSaved *)(XIP_BASE+PICO_FLASH_SIZE_BYTES-2*C_STORE_BYTES);
        if(circuit_record_valid(old)) { calibration=old->calibration;side_cal=old->side;imported_front=true; }
        else imported_front=circuit_import((const CircuitLegacy *)(XIP_BASE+PICO_FLASH_SIZE_BYTES-FLASH_SECTOR_SIZE),
                                           &calibration,&side_cal);
    }
}
static void save_log(void) {
    if(guard.active || !calibration_valid(&calibration) || !side_valid(&side_cal) || !storage_fits()) {
        save_failed=true;return;
    }
    outputs_off();memset(&saved,0,sizeof(saved));BarcodeSaved *v=&saved.data;
    v->magic=B_MAGIC;v->version=recorded_version;v->count=trace_count;v->summary_count=summary_count;v->overwritten=trace_dropped;
    v->duration_us=guard.duration_us;v->reason=guard.reason;v->key=recorded_key;v->offline=recorded_offline;
    v->profile=test_mode;v->recorded_profile=recorded_mode;v->imported=imported_front;
    v->calibration=calibration;v->side=side_cal;v->turns=circuit.turns;v->ignored=circuit.ignored;
    int64_t ticks[2];for(unsigned w=0;w<2;++w) { ticks[w]=v->ticks[w]=cut_ticks[w];v->invalid[w]=cut_invalid[w]; }
    v->path10=(uint64_t)(path_mm(ticks)*10.0);
    for(unsigned n=0;n<trace_count;++n) v->rows[n]=trace[(trace_head+B_ROWS-trace_count+n)%B_ROWS];
    for(unsigned n=0;n<summary_count;++n) v->summaries[n]=summaries[(summary_head+C_SUMMARIES-summary_count+n)%C_SUMMARIES];
    v->event_count=nav.event_count;v->event_overwritten=nav.event_overwritten;
    v->decoded_a=nav.decoded_a;v->decoded_f=nav.decoded_f;v->unsupported=nav.unsupported;v->rejected=nav.rejected;
    v->lefts=nav.lefts;v->rights=nav.rights;v->straight_crosses=nav.straight_crosses;v->last_code=(uint8_t)nav.last_code;
    v->decoded_z=nav.decoded_z;v->uturns=nav.uturns;v->decoded_total=nav.decoded_total;
    for(unsigned n=0;n<nav.event_count;++n) v->events[n]=nav.events[(nav.event_head+B_EVENTS-nav.event_count+n)%B_EVENTS];
    v->crc=log_crc((const uint8_t *)v+offsetof(BarcodeSaved,count),barcode_record_crc_size(v->version));
    if(!wifi_transport_flash_begin()){save_failed=true;return;}
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(LOG_OFFSET,sizeof(saved.bytes));flash_range_program(LOG_OFFSET,saved.bytes,sizeof(saved.bytes));
    restore_interrupts(irq);wifi_transport_flash_end();
    const BarcodeSaved *written=(const BarcodeSaved *)(XIP_BASE+LOG_OFFSET);
    save_failed=!barcode_record_valid(written) || memcmp(written,v,sizeof(*v))!=0;flash_loaded=false;
    save_raw();interface_save_nav_log();save_drive_diag();
}
static void reset_log(void) {
    trace_count=trace_head=trace_dropped=summary_count=summary_head=0;
    guard.duration_us=0;guard.reason=0;circuit=(Circuit){0};nav=(BarcodeNav){0};gentle=(ScanSmooth){0};
    for(unsigned w=0;w<2;++w) {cut_ticks[w]=0;cut_invalid[w]=0;travelled_ticks[w]=0;}
    restored_distance=0;
    flash_loaded=false;recorded_key='-';recorded_version=28;
}
static void help(void) {
    puts("\nBUDDY123 INTERFACE WIFI v2.3 | AUTO ANALOG GP27 | A-Z MQTT MAPPING | no boot assignments | motors OFF on boot");
    puts("Ordinary profiles1/3: continuous steering + continuous per-wheel PI, physical420 cap. No repeated running kick or load-driver switching.");
    puts("Running FF provisional: L150/R180 at45mm/s; encoder PI adjusts. Wheel reference +150/-250mm/s^2, physical PWM slew1200/s; explicit STOP immediate.");
    puts("One bounded150ms starting aid per positive segment, max300, ends early once rolling. Actual output tracked for smooth aid release.");
    puts("Original optical/barcode/junction/search decisions retained. Profile2 remains legacy ramp experiment; use PROFILE1 first.");
    puts("V2 steering: brief gain70%, progressive full same-side authority by350ms; bounded encoder turn damping while returning. Motor controller/caps unchanged.");
    puts("Profile1 normal45mm/s; profile3 up to135; scan45. Pre-search gap/probe uses same motor controller; original zero/coast/search/STOP commands remain.");
    puts("Shared motion APIs retained; A-Z routing enabled: all Buddy3 targets, Buddy4 telemetry and Buddy5 avoidance route through nav coordinator to the single motor writer.");
    puts("Barcode A0 GP27/ADC1; front A0 GP26/GP28. Automatic complete Code39 frame consensus; Mapped A-Z pending until next separate cross; no boot mapping.");
    puts("Cross width11..30mm,4mm WHITE confirmation; rolling left/right and U-turn manoeuvres remain existing settings. Side ahead92mm; turn start55mm before centre.");
    puts("FRONT ONLY:w=both WHITE,l=left BLACK,r=right BLACK;k=save;c=restore successful V2 calibration.1/2/3=profile. Manual calibration remains available.");
    puts("g=USB hands-free2s;f=hold GP21;s/either button stops. BANK:wait5s,BOTH3s,release,wait1s,tap GP21,2s.");
    puts("h=help;n=network status;p=motion/events;d=saved drive diagnostics;j=barcode;a=raw ADC;z=nav coordinator;v=live(defaultOFF). Stop,WAIT5s before changing power.");
    puts("Record28: V2 calibration preset unless own saved calibration exists; diagnostics format4. DRIVER3=continuous; ordinary startup flags256/512. Copied binary telemetry20Hz; no flash writes while moving.");
    printf("MAP: configured=%u revision=%u; set Straight/U-turn/Left/Right on dashboard. Motion history384 rows; raw first2048 samples.\n",(wifi_mapping&WP_MAP_CONFIGURED)!=0,(unsigned)(wifi_mapping>>21));
    printf("CAL FRONT white=%u,%u black=%u,%u valid=%u min_span=%u span=%d,%d barcode=AUTO_A0_GP27 profile=%u restored=%u imported=%u\n",
        calibration.white[0],calibration.white[1],calibration.black[0],calibration.black[1],calibration_valid(&calibration),
        (unsigned)LINE_CAL_MIN_SPAN,(int)calibration.black[0]-(int)calibration.white[0],(int)calibration.black[1]-(int)calibration.white[1],
        test_mode,flash_loaded,imported_front);
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
static const CircuitTracker *current_tracker(void) {
    static CircuitTracker real;
    if(circuit.phase!=J_ACQUIRE) return &circuit.follower;
    if(barcode_nav_action(&nav,nav.action)!='R') return &circuit.joining;
    real=circuit.joining;barcode_mirror_tracker(&real);return &real;
}
static uint16_t trace_state(uint64_t now) {
    unsigned probe=demand.decision==C_PROBE_LEFT?1:demand.decision==C_PROBE_RIGHT?2:0;
    return (uint16_t)(circuit_trace_phase(circuit.phase,current_tracker(),now) |
                      (assist_code<<8) | (feedback_mask<<11) | (probe<<13));
}
static void add_trace(uint64_t now) {
    BarcodeTrace *entry=&trace[trace_head];memset(entry,0,sizeof(*entry));CircuitTrace *r=&entry->motion;
    entry->side_adc=side_adc;entry->ao_black=side_analog.black;entry->ao_bank=side_analog.selected;last_nav_event=nav.event_sequence;
    uint32_t irq=save_and_disable_interrupts();bool active=guard.active;
    int64_t ticks[2];for(unsigned w=0;w<2;++w) ticks[w]=active?total_ticks[w]+enc[w].position:cut_ticks[w];
    r->ms=(now-guard.start_us)/1000;r->active=active;
    for(unsigned w=0;w<2;++w) r->pwm[w]=active?(int16_t)output[w]:0;
    restore_interrupts(irq);
    r->path10=trace_path10(path_mm(ticks));r->turns=circuit.turns;
    r->phase=circuit_trace_phase(circuit.phase,current_tracker(),now);
    r->side=circuit_trace_side(side_analog.black,feedback_mask);
    last_trace_state=trace_state(now);
    r->decision=(uint8_t)(demand.decision | (active?assist_code<<5:0));
    for(unsigned w=0;w<2;++w) {
        r->wheel10[w]=trace_mm10(wheel_mm(w,ticks[w]));r->speed10[w]=(int16_t)(velocity[w]*10);
        r->adc[w]=analog_raw[w];r->level[w]=demand.level[w];r->target[w]=active?demand.target[w]:0;
    }
    trace_head=(trace_head+1)%B_ROWS;
    if(trace_count<B_ROWS) ++trace_count;else if(trace_dropped<UINT32_MAX) ++trace_dropped;
}
static void add_summary(uint64_t now,double path) {
    BarcodeSummary *entry=&summaries[summary_head];memset(entry,0,sizeof(*entry));
    entry->motion=(CircuitSummary){.ms=(now-guard.start_us)/1000,.path10=trace_path10(path),
        .number=circuit.turns,.follow_ms=circuit.follow_ms,.centre_ms=circuit.centre_ms,
        .turn_ms=circuit.turn_ms,.align_ms=circuit.align_ms,
        .angle10=(int16_t)(circuit.turn.turn_degrees*(barcode_nav_action(&nav,nav.action)=='R'?-10:10))};
    entry->code=(uint8_t)nav.action;entry->action=(uint8_t)barcode_nav_action(&nav,nav.action);
    summary_head=(summary_head+1)%C_SUMMARIES;if(summary_count<C_SUMMARIES) ++summary_count;
}
static void print_summary(void) {
    int64_t ticks[2]={cut_ticks[0],cut_ticks[1]};double distance=path_mm(ticks),seconds=(double)guard.duration_us/1e6;
    printf("SUMMARY: total_time_s=%.3f total_distance_mm=%.1f average_speed_mm_s=%.2f completed_turns=%lu lefts=%lu rights=%lu decoded_A=%lu decoded_F=%lu decoded_Z=%lu uturns=%lu unsupported=%lu bad_frames=%lu straight_crosses=%lu ignored_crosses=%lu invalid_L=%lu invalid_R=%lu decoded_total=%lu\n",
        seconds,distance,seconds>0?distance/seconds:0,(unsigned long)circuit.turns,
        (unsigned long)nav.lefts,(unsigned long)nav.rights,(unsigned long)nav.decoded_a,(unsigned long)nav.decoded_f,
        (unsigned long)nav.decoded_z,(unsigned long)nav.uturns,
        (unsigned long)nav.unsupported,(unsigned long)nav.rejected,(unsigned long)nav.straight_crosses,
        (unsigned long)circuit.ignored,(unsigned long)cut_invalid[0],(unsigned long)cut_invalid[1],(unsigned long)nav.decoded_total);
    if(recorded_version>=10 && recorded_mode==2) {
        int peak[2]={0,0};
        for(unsigned n=0;n<trace_count;++n) for(unsigned w=0;w<2;++w)
            if(trace[n].motion.pwm[w]>peak[w]) peak[w]=trace[n].motion.pwm[w];
        printf("RAMP2: straight_target_mm_s=180 sustained_pwm=500 boost_pwm=650 boost_ms=1500 recorded_peak_pwm=%d,%d assist7=ramp_controller (peaks from retained motion rows)\n",peak[0],peak[1]);
    }
}
static const char *event_name(unsigned type) {
    switch(type) {
    case B_RUN:return "RUN";case B_FRAME_START:return "START";case B_DECODE:return "DECODE";
    case B_BAD_FRAME:return "BAD_FRAME";case B_CROSS:return "CROSS";case B_NO_COMMAND:return "NO_COMMAND_STRAIGHT";
    case B_DISPATCH:return "DISPATCH";case B_COMPLETE:return "COMPLETE";case B_GATE_REJECT:return "CROSS_GATE_REJECT";
    case B_QUEUE_OVERFLOW:return "EDGE_OVERFLOW";case B_CROSS_CANDIDATE:return "CROSS_CANDIDATE";
    case B_CROSS_CANCEL:return "CROSS_CANCEL";
    case B_ANALOG_CANDIDATE:return "ANALOG_CANDIDATE";case B_ANALOG_SELECT:return "ANALOG_SELECT";
    case B_ANALOG_CONFLICT:return "ANALOG_CONFLICT";case B_ANALOG_GAP:return "ANALOG_SAMPLE_GAP";
    case B_UTURN_STAGE:return "UTURN_STAGE";
    case B_RECOVERY_WAIT:return "RECOVERY_WAIT";case B_RECOVERY_SWEEP:return "RECOVERY_SWEEP";
    case B_RECOVERY_FOUND:return "RECOVERY_FOUND";case B_RECOVERY_FAIL:return "RECOVERY_FAILED";
    case B_HOLD_BEGIN:return "HOLD_BEGIN";case B_HOLD_PATTERN:return "HOLD_PATTERN";
    case B_HOLD_END:return "HOLD_END";case B_HOLD_CANCEL:return "HOLD_CANCEL";
    case B_SOFT_BEGIN:return "SCAN_SOFT_BEGIN";case B_SOFT_END:return "SCAN_SOFT_END";
    default:return "OTHER";
    }
}
static void dump_events(void) {
    if(guard.active || trace_settling) { puts("Stop and wait for RUN SAVED before j/p.");return; }
    printf("BARCODE_EVENTS: rows=%lu overwritten=%lu (RUN width_mm; COMPLETE width field=rotation_deg)\n",
        (unsigned long)nav.event_count,(unsigned long)nav.event_overwritten);
    if(recorded_version>=3)
        puts("BAD_FRAME flags:1=width,2=class separation,4=wide count,8=character gap,16=pattern,32=timeout,64=bad run; values combine. Complete frames show all9-bit patterns.");
    if(recorded_version>=8) puts("ANALOG_CANDIDATE/SELECT width column=ADC threshold,flags=bank1..63; SAMPLE_GAP width=ms. DECODE flags bit7=analogue,bit0=reverse,bits1..6=bank0..62.");
    else if(recorded_version>=4) puts("ANALOG_CANDIDATE/SELECT width column=ADC threshold,flags=bank1..31; SAMPLE_GAP width=ms. DECODE flags bit7=analogue,bit0=reverse,bits1..5=bank0..30.");
    if(recorded_version>=11) puts("UTURN_STAGE width=rotation_deg;flags1=first90,2=reverse quarter,3=line near180,4=align. Event distance is signed mean-wheel position;SUMMARY distance is accumulated travel.");
    if(recorded_version>=12) puts("RECOVERY_WAIT width=ms;SWEEP width=mm/s,flags=sweep1..5;FOUND flags1=after sweeps;FAILED width=reason. V12 U/search power60%/75% start.");
    puts("number,ms,distance_mm,event,width_mm,code,action,flags,pattern_start,pattern_data,pattern_stop,map_revision");
    for(unsigned k=0;k<nav.event_count;++k) {
        BarcodeEvent *e=&nav.events[(nav.event_head+B_EVENTS-nav.event_count+k)%B_EVENTS];
        printf("%lu,%" PRIu64 ",%.1f,%s,%.2f,%c,%c,%u,%03x,%03x,%03x,%u\n",
            (unsigned long)e->number,e->ms,(double)e->path10/10,event_name(e->type),(double)e->width100/100,
            e->code?e->code:'-',e->action?e->action:'-',e->flags,e->pattern[0],e->pattern[1],e->pattern[2],e->map_revision);
        stdio_flush();sleep_ms(5);
    }
    printf("STOP: %s | run_ms=%" PRIu64 "\n",reason_name(guard.reason),guard.duration_us/1000);print_summary();
}
static void dump(void) {
    if(guard.active || trace_settling) { puts("Stop and wait for RUN SAVED before p.");return; }
    if(!trace_count) { puts("No barcode circuit run saved yet.");return; }
    if(flash_loaded) puts("RESTORED: CRC-verified barcode circuit calibration/log.");
    if(save_failed) puts("FLASH SAVE FAILED: RAM log only.");
    char version_name[12];snprintf(version_name,sizeof(version_name),"%u",recorded_version);
    printf("CIRCUIT WITH BARCODE v%s: key=%c mode=%s profile=%u rows=%lu overwritten=%lu turn_summaries=%lu\n",
        recorded_version==26?"interface-wifi1":recorded_version==25?"interface-continuous2":recorded_version==24?"interface-continuous1":recorded_version==23?"interface1.1handover1":recorded_version==22?"interface1.1diag1":recorded_version==21?"interface1.3":recorded_version==20?"interface1.2":recorded_version==19?"interface1":recorded_version==18?"12.2":recorded_version==17?"12.1":version_name,recorded_key,recorded_key=='m'?"MQTT hands-free":recorded_offline?"offline":recorded_key=='g'?"USB hands-free":"USB hold",recorded_mode,
        (unsigned long)trace_count,(unsigned long)trace_dropped,(unsigned long)summary_count);
    if(recorded_version>=4) puts("ms,active,phase,side_black,distance_mm,L_mm,R_mm,L_mm_s,R_mm_s,PWM_L,PWM_R,ADC_L,ADC_R,levelL,levelR,targetL,targetR,decision,turns,assist,memory,recovery,reacquire,feedback,side_ADC,threshold_ADC,analog_black,bank");
    else puts("ms,active,phase,side_raw,distance_mm,L_mm,R_mm,L_mm_s,R_mm_s,PWM_L,PWM_R,ADC_L,ADC_R,levelL,levelR,targetL,targetR,decision,turns,assist,memory,recovery,reacquire,feedback,pending,last_decoded,action,scan");
    for(unsigned n=0;n<trace_count;++n) {
        if(!stdio_usb_connected()) return;
        BarcodeTrace *entry=&trace[(trace_head+B_ROWS-trace_count+n)%B_ROWS];CircuitTrace *r=&entry->motion;
        printf("%" PRIu64 ",%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%u,%u,%u,%u,%d,%d,%u,%lu,%u,%u,%u,%u,%u,",
            r->ms,r->active,r->phase&15,r->side&1,(double)r->path10/10,(double)r->wheel10[0]/10,(double)r->wheel10[1]/10,
            (double)r->speed10[0]/10,(double)r->speed10[1]/10,r->pwm[0],r->pwm[1],r->adc[0],r->adc[1],
            r->level[0],r->level[1],r->target[0],r->target[1],r->decision&31,(unsigned long)r->turns,r->decision>>5,
            (r->phase>>4)&3,(r->phase>>6)&1,(r->phase>>7)&1,(r->side>>1)&3);
        if(recorded_version>=4) printf("%u,%u,%u,%u\n",entry->side_adc,barcode_analog_log_threshold(recorded_version,entry->ao_bank),entry->ao_black,entry->ao_bank);
        else printf("%c,%c,%c,%u\n",entry->pending?entry->pending:'-',entry->decoded?entry->decoded:'-',entry->action?entry->action:'-',entry->scan);
        stdio_flush();sleep_ms(5);
    }
    printf("MOTION_DUMP_END: rows=%lu\n",(unsigned long)trace_count);
    puts("TURN_SUMMARIES: number,elapsed_ms,distance_mm,follow_ms,centre_ms,turn_ms,alignment_ms,left_positive_rotation_deg,code,action");
    for(unsigned n=0;n<summary_count;++n) {
        BarcodeSummary *entry=&summaries[(summary_head+C_SUMMARIES-summary_count+n)%C_SUMMARIES];CircuitSummary *v=&entry->motion;
        printf("%lu,%" PRIu64 ",%.1f,%lu,%u,%u,%u,%.1f,%c,%c\n",(unsigned long)v->number,v->ms,(double)v->path10/10,
            (unsigned long)v->follow_ms,v->centre_ms,v->turn_ms,v->align_ms,(double)v->angle10/10,entry->code,entry->action);
    }
    dump_events();puts("END BARCODE CIRCUIT LOG. Distances/headings are encoder estimates, not external measurements.");
}
#include "../../../../../harness_adapter.h"
#include "../../../../../drive_diag_pico.h"
#include "../../../../../nav_log_harness.h"
#include "../../../../../wifi_harness.h"
#include "../../../../../buddy4_harness.h" /* Buddy4 add-on (observer only) */

static unsigned start_block(unsigned mode) {
    if(wifi_estop)return START_GP20;
    return barcode_start_block(guard.active,trace_settling,calibration_valid(&calibration),!gpio_get(STOP),
        rearm.ready,!gpio_get(HOLD),mode);
}
static bool start_run(unsigned mode) {
    last_start_block=start_block(mode);if(last_start_block) return false;
    uint64_t now=time_us_64();uint32_t irq=save_and_disable_interrupts();
    bool ok=guard_start(&guard,now,mode!=LINE_USB_HOLD || !gpio_get(HOLD),!gpio_get(STOP));
    if(ok) {
        ++runs;rearm=(LineRearm){0};run_mode=mode;recorded_mode=test_mode;recorded_version=28;last_fast=false;
        continuous_active=false;continuous_steering_reset(&continuous_steering);
        memset(continuous_wheel,0,sizeof(continuous_wheel));
        diag_begin(&diag_capture,guard.start_us,recorded_mode,calibration.white,calibration.black);
        diag_new=true;diag_saved=diag_restored=diag_failed=false;
        barcode_raw_begin(&raw_capture,recorded_mode);raw_new=true;raw_loaded=raw_persisted=raw_save_failed=false;
        capture=(CircuitCapture){0};load=(CircuitLoad){0};steering=(SmoothFollow){0};gentle=(ScanSmooth){0};capture_outer=-1;assist_code=feedback_mask=0;
        guard.deadline_us=UINT64_MAX;guard.duration_us=0;restored_distance=0;
        for(unsigned w=0;w<2;++w) {
            enc[w].position=0;total_ticks[w]=cut_ticks[w]=0;travelled_ticks[w]=0;initial_invalid[w]=enc[w].invalid;cut_invalid[w]=0;
            velocity[w]=0;output[w]=0;peak_until[w]=0;ramp_output[w]=signed_output[w]=maneuver_output[w]=continuous_output[w]=false;maneuver_power[w]=(ManeuverPower){0};
            ramp_wheel[w]=(RampWheel){0};controller[w]=(SignedWheel){0};
            bad_direction_us[w]=0;feedback[w]=(CircuitFeedback){0};
        }
        CircuitInput input={.now=now,.adc={analog_raw[0],analog_raw[1]},.side_black=false};
        barcode_nav_begin(&nav,&input);nav.mapping=wifi_mapping;double initial_wheel[2]={0,0};
        barcode_analog_begin(&side_analog,&nav,side_adc,now,initial_wheel);last_side_sample=now;last_nav_event=0;
        circuit_begin(&circuit,&input);motion_session_begin(now);demand=interface_navigation_step(&input,0);
        barcode_recovery_events(&nav,now,0,false,false,false,0,current_tracker(),&demand,circuit.reason);
        outputs_off();trace_count=trace_head=trace_dropped=summary_count=summary_head=0;
        recorded_key=mode==LINE_USB_FREE?'g':'f';recorded_offline=mode==LINE_BANK;
        trace_settling=false;flash_loaded=save_failed=false;last_control=last_trace=now;
    }
    restore_interrupts(irq);if(!ok) last_start_block=START_GUARD;
    if(ok) add_trace(now);
    return ok;
}
static void report_start_block(void) {
    printf("START rejected: reason=%u ready=%u front_valid=%u levels=%u,%u optical_start_gate=OFF. Need valid FRONT calibration, inactive/settled robot, ready=1, GP20 released; GP21 held for f or released for g/bank. Barcode A0 requires no manual calibration.\n",
        last_start_block,rearm.ready,calibration_valid(&calibration),
        line_level(analog_raw[0],calibration.white[0],calibration.black[0]),
        line_level(analog_raw[1],calibration.white[1],calibration.black[1]));
}
static void control_step(uint64_t now) {
    uint64_t elapsed=now-last_control;if(elapsed<CONTROL_PERIOD_US) return;last_control=now;const uint64_t diag_began=time_us_64();
    uint32_t irq=save_and_disable_interrupts();bool active=guard.active;uint32_t generation=guard.generation;
    BenchEncoder e[2]={enc[0],enc[1]};int64_t ticks[2];
    for(unsigned w=0;w<2;++w) {
        enc[w].position=0;if(active) {total_ticks[w]+=e[w].position;travelled_ticks[w]+=barcode_travel_ticks(e[w].position);}ticks[w]=total_ticks[w];
    }
    restore_interrupts(irq);float dt=(float)elapsed/1e6f;
    for(unsigned w=0;w<2;++w) velocity[w]+=0.25f*(raw_to_mm(w,e[w].position)/dt-velocity[w]);
    buddy4_odometry_note(e,now,elapsed); /* Buddy4 add-on: observe drained encoders */
    if(!active) return;
    diag_observing=diag_capture.begun && !diag_capture.frozen;
    if(diag_observing) {
        memset(&diag_tick,0,sizeof(diag_tick));
        for(unsigned w=0;w<2;++w) {
            diag_tick.v[D_RAW_L+w]=analog_raw[w];diag_tick.v[D_ADC_L+w]=(uint16_t)adc_filtered[w];
            diag_tick.v[D_REF_BEFORE_L+w]=diag_half(controller[w].pi.ramp);
            diag_tick.v[D_I_BEFORE_L+w]=diag_half(controller[w].pi.integral);
        }
        diag_tick.v[D_MEAN_BEFORE]=diag_half(gentle.mean);diag_tick.v[D_HALF_BEFORE]=diag_half(gentle.half);
        if(gentle.active)diag_tick.v[D_FLAGS]|=DF_ACTIVE_BEFORE;
    }
    CircuitInput input={.now=now,.wheel={wheel_mm(0,ticks[0]),wheel_mm(1,ticks[1])},.path=path_mm(ticks),
        .speed={velocity[0],velocity[1]},.adc={(uint16_t)adc_filtered[0],(uint16_t)adc_filtered[1]},
        .side_black=side_analog.black};
    unsigned previous_phase=circuit.phase,previous_decision=demand.decision;
    const CircuitTracker *before_tracker=current_tracker();
    bool was_white=before_tracker->core.white,was_waiting=before_tracker->search.waiting,was_search=before_tracker->search.active;
    demand=interface_navigation_step(&input,dt);
    barcode_recovery_events(&nav,now,(input.wheel[0]+input.wheel[1])*.5,was_white,was_waiting,was_search,previous_decision,current_tracker(),&demand,circuit.reason);
    if((previous_phase==J_TURN || (barcode_nav_action(&nav,nav.action)=='U' && previous_phase==J_TURN_SETTLE)) && circuit.phase==J_ACQUIRE) circuit_capture_begin(&capture,now,input.path);
    if(circuit.completed) add_summary(now,input.path);
    bool external=interface_motion.external;
    uint8_t failure=demand.stop?circuit.reason:WHY_NONE;int32_t next[2];
    bool turning=circuit.phase==J_TURN && ((demand.target[0]==0 && demand.target[1]>0) ||
                                                         (demand.target[1]==0 && demand.target[0]!=0));
    if(external)turning=interface_motion.turning;
    bool ramp_drive=!external && ramp_profile_active(recorded_mode,&circuit,&demand);
    unsigned wheel_profile=!external && circuit.fast?recorded_mode:1;
    unsigned previous_assist=assist_code;
    capture_outer=circuit_capture_outer(&capture,&input,circuit.phase,&demand);
    bool recovering=current_tracker()->core.white;
    int search_outer=line_search_outer(current_tracker()->search.active && recovering && !demand.stop,
                                       demand.target[0],demand.target[1]);
    if(external) {capture_outer=-1;recovering=false;search_outer=-1;}
    bool maneuver_drive=external?turning:(turning && barcode_nav_action(&nav,nav.action)=='U') || search_outer>=0;
    if(recovering) capture_outer=-1;
    int load_outer=circuit_load_outer(&load,&input,circuit.phase,&demand,capture_outer>=0 || recovering);
    continuous_active=continuous_drive_selected(recorded_mode,circuit.phase,circuit.busy,external,ramp_drive,maneuver_drive,recovering,&demand);
    if(external || continuous_active)load_outer=-1;
    if(continuous_active) {capture_outer=-1;load=(CircuitLoad){0};}
    assist_code=search_outer>=0?(unsigned)search_outer+5:capture_outer>=0?(unsigned)capture_outer+1:load_outer>=0?(unsigned)load_outer+3:0;
    int effort_outer=search_outer>=0?search_outer:capture_outer>=0?capture_outer:load_outer;
    bool loaded[2]={turning || effort_outer==0,turning || effort_outer==1};
    for(unsigned w=0;w<2;++w) {
        if(!continuous_active)continuous_leave(&continuous_wheel[w],&controller[w],w,demand.target[w],velocity[w],output[w]);
        if(!ramp_drive)ramp_wheel_leave(&ramp_wheel[w],&controller[w],demand.target[w]);
        if(!maneuver_drive)maneuver_leave(&maneuver_power[w],&controller[w],demand.target[w]);
    }
    if(diag_observing && !ramp_drive && !maneuver_drive && effort_outer>=0 && assist_code!=previous_assist)diag_tick.v[D_FLAGS]|=DF_PREPARE;
    if(!ramp_drive && !maneuver_drive && effort_outer>=0 && assist_code!=previous_assist) circuit_capture_prepare(&controller[effort_outer]);
    bool leave_fast=last_fast && !circuit.fast;
    bool preserve_cruise_I=leave_fast && !continuous_active && cruise_handover_eligible(recorded_mode,circuit.phase,demand.decision,
        demand.target,velocity,controller,external || ramp_drive || maneuver_drive || circuit.busy ||
        recovering || effort_outer>=0,demand.stop);
    if(diag_observing && leave_fast)diag_tick.v[D_FLAGS]|=DF_LEAVE_FAST;
    if(diag_observing && preserve_cruise_I)diag_tick.v[D_FLAGS]|=DF_HANDOVER_KEEP;
    if(leave_fast && !continuous_active) for(unsigned w=0;w<2;++w) cruise_handover_apply(&controller[w],preserve_cruise_I);
    last_fast=circuit.fast;
    feedback_mask=0;
    for(unsigned w=0;w<2;++w) {
        if(continuous_active)next[w]=continuous_drive_step(&continuous_wheel[w],&controller[w],w,demand.target[w],velocity[w],dt,now,output[w]);
        else if(maneuver_drive)next[w]=maneuver_wheel_step(&maneuver_power[w],&controller[w],w,demand.target[w],velocity[w],dt,now);
        else if(ramp_drive) next[w]=ramp_wheel_step(&ramp_wheel[w],&controller[w],w,demand.target[w],velocity[w],dt,now);
        else {
            ramp_wheel_leave(&ramp_wheel[w],&controller[w],demand.target[w]);
            next[w]=circuit_wheel_step(&controller[w],w,demand.target[w],velocity[w],dt,now,loaded[w],wheel_profile);
        }
        bad_direction_us[w]=demand.target[w]!=0 && velocity[w]*(demand.target[w]>0?1:-1)<-20?bad_direction_us[w]+elapsed:0;
        if(bad_direction_us[w]>=50000) failure=WHY_DIRECTION;
        /* User-requested V5 behavior: retain diagnostic evidence but do not
           stop driving solely because the encoder-progress window expires. */
        if(circuit_stalled(&feedback[w],now,w==0?-ticks[w]:ticks[w],demand.target[w],controller[w].pi.ramp))
            feedback_mask|=1u<<w;
        if(e[w].invalid-initial_invalid[w]>=8) failure=WHY_ENCODER;
    }
    if(ramp_drive) assist_code=7; /* V10 ramp per-wheel speed controller */
    irq=save_and_disable_interrupts();
    if(guard.active && guard.generation==generation) {
        if(failure) kill(now,failure);
        else {
            InterfaceCommit commit={.next={next[0],next[1]},.ramp=ramp_drive,.maneuver=maneuver_drive,
                .continuous=continuous_active,.turning=turning,.external=external,.loaded={loaded[0],loaded[1]},.profile=wheel_profile,.now=now};
            bool committed=motion_commit(interface_motion.generation,interface_commit_outputs,&commit);
            if(!committed)outputs_off();
            if(diag_observing && committed)diag_tick.v[D_FLAGS]|=DF_COMMIT;
            guard.last_control_us=now;
        }
    }
    restore_interrupts(irq);
    observe_drive_tick(&input,e,next,wheel_profile,ramp_drive,maneuver_drive,external,loaded,diag_began,elapsed);
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
    interface_lock=spin_lock_instance((uint)spin_lock_claim_unused(true));
    MotionPort port={interface_enter,interface_leave,interface_off,interface_now,NULL};motion_init(&port);
    struct repeating_timer timer;
    if(!add_repeating_timer_us(-1000,safety_tick,NULL,&timer)) while(true) tight_loop_contents();
    /* GP19 deliberately unused: offset signal is analogue GP27. */
    adc_init();adc_gpio_init(26);adc_gpio_init(28);adc_gpio_init(27);
    analog_raw[0]=read_adc(0);analog_raw[1]=read_adc(2);
    adc_filtered[0]=analog_raw[0];adc_filtered[1]=analog_raw[1];
    side_adc=read_adc(1);side_analog.selected=AO_DEFAULT_BANK;stdio_init_all();load_log();
    const BarcodeSaved *startup_record=(const BarcodeSaved *)(XIP_BASE+LOG_OFFSET);
    if(successful_calibration_boot(&calibration,startup_record->version,barcode_record_valid(startup_record))) {
        test_mode=1;imported_front=true;
        puts("CAL PRESET: successful Continuous V2 run; white=163,160 black=3457,3700. w/l/r/k remain available; c restores preset.");
    } else puts("CAL SAVED: restoring this V2.3 firmware's saved calibration/profile; c restores preset.");
    recorded_version=28;
    load_raw();load_drive_diag();buddy_extension_init();side_cal=(SideCalibration){0,1,3};last_control=time_us_64();wifi_harness_init();
    buddy4_init(); /* Buddy4 add-on */
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
            if(connected) { arm.state=ARM_IDLE;help();if(trace_count) puts("Saved run available: j=barcode events, p=motion, d=drive diagnostics."); }
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
        if(wifi_console_byte(ch))ch=PICO_ERROR_TIMEOUT;
        if(ch=='h') help();
        else if(ch=='p') dump();
        else if(ch=='j') dump_events();
        else if(ch=='d') { usb_pending=false;arm.state=ARM_IDLE;dump_drive_diag(); }
        else if(ch=='z') { usb_pending=false;arm.state=ARM_IDLE;interface_dump_nav_log(); }
        else if(ch=='a') { usb_pending=false;arm.state=ARM_IDLE;dump_raw(); }
        else if(ch=='s') {
            usb_pending=false;arm.state=ARM_IDLE;uint32_t irq=save_and_disable_interrupts();kill(now,WHY_SERIAL);restore_interrupts(irq);

        } else if(ch=='c') {
            usb_pending=false;wifi_start_pending=false;arm.state=ARM_IDLE;
            if(guard.active || trace_settling || hold || stop)
                puts("CAL RESTORE rejected: stop, wait5s, release both buttons.");
            else {
                reset_log();calibration=successful_calibration();imported_front=true;
                puts("CAL RESTORED: white=163,160 black=3457,3700; k saves. Previous run rows cleared; profile unchanged.");
            }
        } else if(ch=='w' || ch=='l' || ch=='r') { usb_pending=false;calibrate(ch); }
        else if(ch=='o' || ch=='b') puts("No side calibration needed: automatic analogue A0 on GP27. w/l/r calibrate FRONT sensors only.");
        else if(ch=='v') { status_enabled=!status_enabled;printf("LIVE enabled=%u\n",status_enabled); }
        else if(ch=='k' || ch=='1' || ch=='2' || ch=='3') {
            usb_pending=false;
            if(guard.active || trace_settling || hold || stop) puts("CONFIG rejected: stop/release both buttons.");
            else if(ch=='k') { save_log();puts(save_failed?"SAVE FAILED: need valid FRONT calibration":"CALIBRATION/PROFILE SAVED."); }
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
        if(time_us_64()-last_side_sample>=2000) {
            last_side_sample=time_us_64();adc_select_input(1);(void)adc_read();unsigned sum=0;
            for(unsigned k=0;k<4;++k) sum+=adc_read();
            side_adc=(uint16_t)((sum+2)/4);
            if(guard.active && nav_get_status().mode==MOTION_AUTO) {
                int64_t ticks[2];uint32_t irq=save_and_disable_interrupts();
                for(unsigned w=0;w<2;++w) ticks[w]=total_ticks[w]+enc[w].position;
                restore_interrupts(irq);double wheel[2]={wheel_mm(0,ticks[0]),wheel_mm(1,ticks[1])};
                barcode_analog_sample(&side_analog,&nav,side_adc,last_side_sample,wheel);
                double path100=(wheel[0]+wheel[1])*50;
                int32_t point=path100>INT32_MAX?INT32_MAX:path100<INT32_MIN?INT32_MIN:(int32_t)path100;
                barcode_raw_sample(&raw_capture,last_side_sample-guard.start_us,point,side_adc);
            } else { side_analog.adc=side_adc;side_analog.black=side_adc>=barcode_analog_threshold(side_analog.selected); }
        }
        uint8_t before_phase=circuit.phase;
        control_step(time_us_64());now=time_us_64();
        buddy_extension_poll(now);wifi_barcode_events(now);wifi_harness_poll(now);
        buddy4_poll(time_us_64()); /* Buddy4 add-on */
        if(connected && guard.active && circuit.phase!=before_phase)
            printf("PHASE %u: %s\n",circuit.phase,phase_name(circuit.phase));
        if(guard.stops!=seen_stops) {
            seen_stops=guard.stops;stopped_us=now;trace_settling=true;run_mode=LINE_USB_HOLD;
            add_trace(now);last_trace=now;
            if(connected) printf("STOP: %s; wait5s then p\n",reason_name(guard.reason));
        }
        if(guard.active || trace_settling) {
            if(now-last_trace>=20000 || (guard.active && (trace_state(now)!=last_trace_state || nav.event_sequence!=last_nav_event))) {
                add_trace(now);last_trace=now;
            }
            if(trace_settling && now-stopped_us>=1000000) {
                trace_settling=false;save_log();
                if(connected) {
                    puts(save_failed?"FLASH SAVE FAILED":"RUN SAVED; p=motion/events; d=drive diagnostics; j=barcode; a=raw; z=coordinator.");
                    if(raw_save_failed) puts("RAW FLASH SAVE FAILED; a can still dump the RAM trace.");
                    if(interface_log_failed) puts("NAV FLASH SAVE FAILED; z can still dump the RAM audit.");
                    if(diag_failed)puts("DIAG FLASH SAVE FAILED; d can still dump RAM capture.");
                    print_summary();
                }
            }
        }
        if(connected && status_enabled && now-last_status>=500000) {
            int64_t status_ticks[2];
            uint32_t irq=save_and_disable_interrupts();
            for(unsigned w=0;w<2;++w) status_ticks[w]=guard.active?total_ticks[w]+enc[w].position:cut_ticks[w];
            uint64_t duration=guard.active?now-guard.start_us:guard.duration_us;
            restore_interrupts(irq);
            double distance=path_mm(status_ticks);
            printf("BARCODE_CIRCUIT: ms=%" PRIu64 " profile=%u phase=%u(%s) active=%u distance_mm=%.1f speed_mm_s=%.1f avg_mm_s=%.1f wheel_mm_s=%.1f,%.1f targets=%d,%d PWM=%ld,%ld ADC=%u,%u levels=%u,%u side=%u turns=%lu assist=%d memory=%u recovery=%u reacquire=%u feedback=%u arm=%u ready=%u pending=%u runs=%lu barcode=%c queued=%c action=%c scan=%u decoded_A=%lu decoded_F=%lu decoded_Z=%lu uturns=%lu lefts=%lu rights=%lu edge_overflow=%lu\n",
                duration/1000,test_mode,circuit.phase,phase_name(circuit.phase),guard.active,distance,
                (double)(velocity[0]+velocity[1])*0.5,duration?distance/((double)duration/1e6):0,
                (double)velocity[0],(double)velocity[1],guard.active?demand.target[0]:0,guard.active?demand.target[1]:0,
                guard.active?(long)output[0]:0,guard.active?(long)output[1]:0,analog_raw[0],analog_raw[1],
                line_level(analog_raw[0],calibration.white[0],calibration.black[0]),
                line_level(analog_raw[1],calibration.white[1],calibration.black[1]),side_analog.black,
                (unsigned long)circuit.turns,guard.active?(int)assist_code:0,
                circuit_memory(current_tracker(),now),current_tracker()->core.white,
                current_tracker()->reacquire_timing,feedback_mask,arm.state,rearm.ready,usb_pending,(unsigned long)runs,
                nav.last_code?nav.last_code:'-',nav.pending?nav.pending:'-',nav.busy?barcode_nav_action(&nav,nav.action):'-',nav.external_scanning,
                (unsigned long)nav.decoded_a,(unsigned long)nav.decoded_f,(unsigned long)nav.decoded_z,(unsigned long)nav.uturns,
                (unsigned long)nav.lefts,(unsigned long)nav.rights,
                0ul);
            printf("ANALOG: ADC=%u selected_threshold=%u candidates=%lu selections=%lu max_gap_us=%lu gaps_over10ms=%lu\n",
                side_adc,barcode_analog_threshold(side_analog.selected),(unsigned long)side_analog.candidates,(unsigned long)side_analog.selections,
                (unsigned long)side_analog.max_gap_us,(unsigned long)side_analog.gap_count);
            MotionStatus state=nav_get_status();
            printf("NAV_COORD: mode=%u owner=%u session=%u stopped=%u command_id=%lu command_state=%u fault=0x%03x targets=%d,%d\n",
                state.mode,state.owner,state.session_active,state.stopped,(unsigned long)state.command_id,
                state.command_state,state.fault,state.target_mm_s[0],state.target_mm_s[1]);
            last_status=now;
        }
        sleep_us(100);
    }
}
