/* Stationary diagnostic: barcode A0 -> GP27/ADC1, Pico W physical pin32.
 * Front sensors unchanged. GP19/D0 unused. Motors always LOW; k saves calibration. */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "probe_record.h"
#include "../left_motor_bench/bench_logic.h"
#include "../dual_wheel_bench/dual_logic.h"

enum { ANALOG_PIN=27 };
_Static_assert(ANALOG_PIN==27,"Barcode A0 must match GP27 / ADC1");
typedef struct {
    uint32_t n;
    uint16_t lo, hi;
    uint64_t sum, squares;
} Stats;
static Calibration calibration;
static double stats_mean(const Stats *s) { return s->n?(double)s->sum/s->n:0; }
static double stats_sd(const Stats *s) {
    if(!s->n) return 0;
    double mean=stats_mean(s),variance=(double)s->squares/s->n-mean*mean;
    return sqrt(variance>0?variance:0);
}
static bool calibration_valid(void) { return probe_cal_valid(&calibration); }
static int level_1000(double adc) { return probe_level(&calibration,adc); }
static bool calibration_saved,calibration_restored;
static int saved_slot=-1;static uint32_t saved_sequence;
#define CAL_OFFSET (PICO_FLASH_SIZE_BYTES-4u*65536u)
extern char __flash_binary_end;
static union { uint8_t bytes[FLASH_SECTOR_SIZE];ProbeRecord record; } flash_page;
_Static_assert(sizeof(ProbeRecord)<=FLASH_PAGE_SIZE,"Calibration record must fit a flash page");
static bool storage_fits(void) { return (uintptr_t)&__flash_binary_end<=XIP_BASE+CAL_OFFSET; }
static void restore_calibration(void) {
    if(!storage_fits()) return;
    for(unsigned slot=0;slot<2;++slot) {
        const ProbeRecord *p=(const ProbeRecord *)(XIP_BASE+CAL_OFFSET+slot*FLASH_SECTOR_SIZE);
        if(probe_record_valid(p) && (saved_slot<0 || probe_sequence_newer(p->sequence,saved_sequence))) {
            calibration=p->calibration;saved_slot=(int)slot;saved_sequence=p->sequence;
            calibration_saved=calibration_restored=true;
        }
    }
}
static volatile BenchEncoder enc[2];
static const unsigned ea[2]={0,4},eb[2]={1,5};
static uint8_t read_ab(unsigned w) {
    uint32_t v=gpio_get_all();return (uint8_t)((((v>>ea[w])&1u)<<1)|((v>>eb[w])&1u));
}
static void encoder_irq(uint pin,uint32_t events) {
    (void)events;unsigned w=pin==ea[0]||pin==eb[0]?0:1;
    bench_encoder_update(&enc[w],read_ab(w));
}
static bool save_calibration(void) {
    if(!calibration_valid() || !storage_fits()) return false;
    unsigned slot=saved_slot==0?1:0;memset(&flash_page,0xff,sizeof(flash_page));
    probe_record_make(&flash_page.record,&calibration,saved_sequence+1);
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(CAL_OFFSET+slot*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
    flash_range_program(CAL_OFFSET+slot*FLASH_SECTOR_SIZE,flash_page.bytes,FLASH_PAGE_SIZE);
    /* No scan is active; rebase quadrature state after the flash IRQ pause. */
    for(unsigned w=0;w<2;++w) enc[w].ab=read_ab(w);
    restore_interrupts(irq);
    const ProbeRecord *p=(const ProbeRecord *)(XIP_BASE+CAL_OFFSET+slot*FLASH_SECTOR_SIZE);
    if(!probe_record_valid(p) || memcmp(p,&flash_page.record,sizeof(*p))) return false;
    saved_slot=(int)slot;saved_sequence=p->sequence;calibration_saved=true;calibration_restored=false;
    return true;
}
/* Raw capture is buffered in RAM; serial formatting occurs only after recording. */
enum { SWEEP_ROWS=10000,SWEEP_PERIOD_US=2000,SWEEP_LIMIT_US=20000000 };
typedef struct { uint32_t us;int32_t ticks[2];uint16_t adc,padding; } SweepRow;
_Static_assert(sizeof(SweepRow)==16,"Sweep row layout");
static SweepRow sweep[SWEEP_ROWS];static unsigned sweep_count;
static bool sweep_active,sweep_waiting,sweep_have;
static uint64_t sweep_arm,sweep_start,sweep_stop,sweep_next,sweep_previous;
static uint32_t sweep_max_interval,sweep_missed,sweep_invalid[2],sweep_bad[2];
static int32_t sweep_origin[2];static Calibration sweep_cal;
static double sweep_mm(unsigned w,int32_t ticks) { return (double)ticks*mm_per_tick(w)*(w==0?-1:1); }
static void begin_sweep(uint64_t now) {
    uint32_t irq=save_and_disable_interrupts();
    for(unsigned w=0;w<2;++w) { sweep_origin[w]=enc[w].position;sweep_invalid[w]=enc[w].invalid; }
    restore_interrupts(irq);
    sweep_start=sweep_next=now;sweep_previous=0;sweep_count=sweep_max_interval=sweep_missed=0;
    sweep_cal=calibration;sweep_active=true;sweep_waiting=false;sweep_have=false;
    puts("SWEEP_BEGIN: motors=OFF roll_FORWARD_straight_by_hand=1 wheels_on_mat=1 max_s=20 sample_hz=500");
}
static void end_sweep(uint64_t now,const char *why) {
    sweep_active=false;sweep_have=sweep_count>0;sweep_stop=now;
    uint32_t irq=save_and_disable_interrupts();
    for(unsigned w=0;w<2;++w) sweep_bad[w]=enc[w].invalid-sweep_invalid[w];
    restore_interrupts(irq);
    printf("SWEEP_END: reason=%s samples=%u duration_ms=%" PRIu64 " max_interval_us=%" PRIu32
           " missed_slots=%" PRIu32 " invalid_L=%" PRIu32 " invalid_R=%" PRIu32 " RAM_ONLY=1 press_p=1 motors=OFF\n",
           why,sweep_count,(now-sweep_start)/1000,sweep_max_interval,sweep_missed,sweep_bad[0],sweep_bad[1]);
}
static void sample_sweep(uint64_t now) {
    if(!sweep_active || now<sweep_next) return;
    if(now-sweep_start>=SWEEP_LIMIT_US || sweep_count==SWEEP_ROWS) { end_sweep(now,"limit");return; }
    SweepRow *r=&sweep[sweep_count++];r->us=(uint32_t)(now-sweep_start);r->adc=adc_read();r->padding=0;
    uint32_t irq=save_and_disable_interrupts();
    for(unsigned w=0;w<2;++w) r->ticks[w]=enc[w].position-sweep_origin[w];
    restore_interrupts(irq);
    if(sweep_previous) { uint32_t d=(uint32_t)(now-sweep_previous);if(d>sweep_max_interval) sweep_max_interval=d; }
    sweep_previous=now;
    if(now-sweep_next>=SWEEP_PERIOD_US) sweep_missed+=(uint32_t)((now-sweep_next)/SWEEP_PERIOD_US);
    sweep_next=now+SWEEP_PERIOD_US;
}
static void dump_sweep(void) {
    if(!sweep_have) { puts("No sweep recorded. f=arm2seconds, manually roll over A, s=stop, p=dump.");return; }
    printf("ANALOG_SWEEP v5: samples=%u duration_ms=%" PRIu64 " pin=27 ADC=1 paced_rows_per_s=200 motors=OFF\n",sweep_count,(sweep_stop-sweep_start)/1000);
    printf("SWEEP_CAL: white=%.1f black=%.1f valid=%u max_interval_us=%" PRIu32
           " missed_slots=%" PRIu32 " invalid_L=%" PRIu32 " invalid_R=%" PRIu32 "\n",
           sweep_cal.white,sweep_cal.black,probe_cal_valid(&sweep_cal),sweep_max_interval,sweep_missed,sweep_bad[0],sweep_bad[1]);
    puts("sample,us,ADC,level_1000,ticks_L,ticks_R,L_mm,R_mm,mean_forward_mm");
    uint32_t crc=0xffffffffu;char line[192];
    for(unsigned k=0;k<sweep_count;++k) {
        if(!stdio_usb_connected()) return; /* RAM copy remains available for p after reconnect. */
        const SweepRow *r=&sweep[k];double l=sweep_mm(0,r->ticks[0]),right=sweep_mm(1,r->ticks[1]);
        int length=snprintf(line,sizeof(line),"%u,%" PRIu32 ",%u,%d,%" PRId32 ",%" PRId32 ",%.3f,%.3f,%.3f\n",
               k,r->us,r->adc,probe_level(&sweep_cal,r->adc),r->ticks[0],r->ticks[1],l,right,(l+right)*.5);
        if(length<=0 || (size_t)length>=sizeof(line)) { puts("DUMP_ERROR: row format overflow; retry p.");return; }
        crc=probe_crc_update(crc,line,(size_t)length);
        printf("%s",line);stdio_flush();sleep_ms(5);
    }
    printf("SWEEP_DUMP_END: rows=%u crc32=%08" PRIx32 " (numeric rows including LF; normalize CRLF to LF)\n",sweep_count,~crc);
    puts("END ANALOG_SWEEP. Raw signal, no letter decoded. Encoder distances are estimates. Sweep is RAM only.");
}
static void print_calibration(void) {
    printf("CAL: white=%.1f black=%.1f white_sd=%.1f black_sd=%.1f span=%.1f mask=%u valid=%u polarity=%s saved=%u restored=%u storage=FLASH_ON_K motors=OFF\n",
           calibration.white,calibration.black,calibration.white_sd,calibration.black_sd,
           fabs(calibration.black-calibration.white),calibration.mask,calibration_valid(),
           calibration.mask!=3?"UNKNOWN":calibration.black>calibration.white?"BLACK_HIGH":"BLACK_LOW",
           calibration_saved,calibration_restored);
}
static Stats empty_stats(void) {
    return (Stats){.lo=4095};
}
static void add_sample(Stats *s, uint16_t a) {
    ++s->n;
    s->sum+=a;
    s->squares+=(uint32_t)a*a;
    if(a<s->lo) s->lo=a;
    if(a>s->hi) s->hi=a;
}
static void motors_off(void) {
    for(unsigned pin=8;pin<=11;++pin) gpio_put(pin,0);
}
static void print_stats(const char *kind, const char *label, const Stats *s,
                        uint64_t start, uint64_t end) {
    if(!s->n) return;
    double mean=stats_mean(s);
    printf("%s: label=%s start_ms=%" PRIu64 " end_ms=%" PRIu64
           " n=%" PRIu32 " ADC_mean=%.1f min=%u max=%u sd=%.1f level_1000=%d cal=%u motors=OFF\n",
           kind,label,start/1000,end/1000,s->n,mean,s->lo,s->hi,
           stats_sd(s),level_1000(mean),calibration_valid());
}
static const char *label_for(int ch) {
    switch(ch) {
    case 'w': return "BROAD_WHITE";
    case 'b': return "BROAD_BLACK";
    case 'g': return "NARROW_WHITE_GAP";
    case 'n': return "ADJACENT_BLACK_BAR";
    default: return NULL;
    }
}
static void help(void) {
    puts("\nBUDDY23 BARCODE ANALOG PROBE v5 | motors=OFF always");
    puts("Barcode A0 -> GP27/ADC1 (Pico W physical pin32). Front sensors unchanged; GP19/D0 unused.");
    puts("Sensor VCC=3.3V, common GND. Do not connect a 5V output to the ADC.");
    puts("USB-only motors-OFF probe. k=save calibration (dual CRC slots); restored on boot. No barcode decoding.");
    puts("Live ADC windows every100ms, nominal1kHz sampling; raw0..4095. No black polarity assumed.");
    puts("Keep still: w=capture broad WHITE reference, then b=capture broad BLACK reference; each1second.");
    puts("Then g=narrowest white gap; n=adjacent NARROW black bar; repeat at least3times, normal height.");
    puts("level_1000: white reference0, black reference1000; -1=not calibrated. Polarity learned automatically.");
    puts("CAL valid needs both references, span>=100counts and >=6x largest reference SD; not proof of narrow-bar contrast.");
    puts("Each key records1second then prints CAPTURE. Repeat g/n at least3times, normal mounted height.");
    puts("c=calibration; k=save current calibration while still; r=reset RAM references (saved copy kept).");
    puts("f=arm2s then record500Hz up to20s; roll straight by hand, wheels on mat; s=stop; p=dump sweep.\nq=last stationary capture; v=toggle LIVE (off by default); h=help. Sweep RAM-only; calibration persists after k.");
    puts("V5 p dump is paced5ms/row: wait up to55seconds for SWEEP_DUMP_END + END ANALOG_SWEEP. Row numbers/CRC detect missing output.");
    print_calibration();
}
int main(void) {
    for(unsigned pin=8;pin<=11;++pin) {
        gpio_init(pin);gpio_put(pin,0);gpio_set_dir(pin,GPIO_OUT);
    }
    adc_init();adc_gpio_init(ANALOG_PIN);adc_select_input(ANALOG_PIN-26);(void)adc_read();
    for(unsigned w=0;w<2;++w) {
        gpio_init(ea[w]);gpio_set_dir(ea[w],GPIO_IN);gpio_pull_up(ea[w]);
        gpio_init(eb[w]);gpio_set_dir(eb[w],GPIO_IN);gpio_pull_up(eb[w]);enc[w].ab=read_ab(w);
    }
    gpio_set_irq_enabled_with_callback(ea[0],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true,encoder_irq);
    gpio_set_irq_enabled(eb[0],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    gpio_set_irq_enabled(ea[1],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    gpio_set_irq_enabled(eb[1],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    restore_calibration();stdio_init_all();
    bool was_connected=false, capturing=false, have_last=false, live_enabled=false;
    const char *capture_label="NONE", *last_label="NONE";int capture_key=0;
    Stats live=empty_stats(), capture=empty_stats(), last=empty_stats();
    uint64_t next_sample=0, next_report=0, live_start=0, capture_start=0;
    uint64_t last_start=0, last_end=0;
    for(;;) {
        motors_off();
        bool connected=stdio_usb_connected();
        uint64_t now=time_us_64();
        if(connected&&!was_connected) {
            help();live=empty_stats();live_start=now;next_report=now+100000;next_sample=now;
        }
        if(!connected) {
            capturing=false;sweep_waiting=false;
            if(sweep_active) end_sweep(now,"USB_disconnected");
        }
        was_connected=connected;
        int ch=connected?getchar_timeout_us(0):PICO_ERROR_TIMEOUT;
        if(sweep_active || sweep_waiting) {
            if(ch=='s') {
                if(sweep_active) end_sweep(now,"serial_s");
                else { sweep_waiting=false;puts("SWEEP_CANCELLED: motors=OFF"); }
            }
            /* Avoid serial formatting and all calibration mutations while sampling. */
            if(sweep_waiting && now>=sweep_arm) begin_sweep(now);
            sample_sweep(time_us_64());sleep_us(100);continue;
        }
        const char *label=label_for(ch);
        if(ch=='f') {
            if(capturing) puts("BUSY: wait for stationary capture.");
            else { sweep_waiting=true;sweep_arm=now+2000000;puts("SWEEP_ARMED: starts_in_ms=2000 motors=OFF. Wait for SWEEP_BEGIN, then push forward."); }
        }
        if(ch=='p') {
            if(capturing) puts("BUSY: wait for capture before sweep dump.");
            else dump_sweep();
        }
        if(ch=='v') { live_enabled=!live_enabled;printf("LIVE enabled=%u\n",live_enabled); }

        if(ch=='h') help();
        if(ch=='c') print_calibration();
        if(ch=='r') {
            if(capturing) puts("BUSY: wait for CAPTURE before reset.");
            else { calibration=(Calibration){0};calibration_saved=calibration_restored=false;print_calibration(); }
        }
        if(ch=='k') {
            if(capturing) puts("BUSY: wait for capture before save.");
            else { puts(save_calibration()?"CAL SAVED: CRC/readback verified; restored on next boot.":"CAL SAVE rejected/failed: need valid w/b references and available storage; old valid slot kept.");print_calibration(); }
        }
        if(ch=='q') {
            if(have_last) print_stats("CAPTURE",last_label,&last,last_start,last_end);
            else puts("No completed capture yet.");
            print_calibration();
        }
        if(label) {
            if(capturing) puts("BUSY: keep still until CAPTURE completes.");
            else if(ch=='b' && !(calibration.mask&1)) puts("CAL rejected: capture broad WHITE with w first.");
            else {
                capturing=true;capture=empty_stats();capture_label=label;capture_key=ch;capture_start=now;
                printf("CAPTURE_BEGIN: label=%s duration_ms=1000 keep_still=1 motors=OFF\n",label);
            }
        }
        now=time_us_64();
        if(connected&&now>=next_sample) {
            uint16_t a=adc_read();
            add_sample(&live,a);
            if(capturing) add_sample(&capture,a);
            next_sample=now+1000;
        }
        if(capturing&&now-capture_start>=1000000) {
            last=capture;last_label=capture_label;last_start=capture_start;last_end=now;have_last=true;
            capturing=false;
            if(capture_key=='w' || capture_key=='b') {
                if(last.n<500) puts("CAL rejected: fewer than500 samples; previous references retained. Retry capture.");
                else if(capture_key=='w') {
                    calibration=(Calibration){.white=stats_mean(&last),.white_sd=stats_sd(&last),.mask=1};
                    calibration_saved=calibration_restored=false;
                } else {
                    calibration.black=stats_mean(&last);calibration.black_sd=stats_sd(&last);calibration.mask|=2;
                    calibration_saved=calibration_restored=false;
                }
                print_calibration();
            }
            print_stats("CAPTURE",last_label,&last,last_start,last_end);
        }
        if(connected&&live_enabled&&now>=next_report) {
            print_stats("LIVE","UNLABELLED",&live,live_start,now);
            live=empty_stats();live_start=now;next_report=now+100000;
        }
        if(!live_enabled && now>=next_report) { live=empty_stats();live_start=now;next_report=now+100000; }
        sleep_us(100);
    }
}
