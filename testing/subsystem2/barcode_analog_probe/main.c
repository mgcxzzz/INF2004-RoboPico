/* Stationary diagnostic: barcode A0 -> GP27/ADC1, Pico W physical pin32.
 * Front sensors unchanged. GP19/D0 unused. No PWM, motion or flash writes. */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"

enum { ANALOG_PIN=27 };
_Static_assert(ANALOG_PIN==27,"Barcode A0 must match GP27 / ADC1");
typedef struct {
    uint32_t n;
    uint16_t lo, hi;
    uint64_t sum, squares;
} Stats;
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
    double mean=(double)s->sum/s->n;
    double variance=(double)s->squares/s->n-mean*mean;
    printf("%s: label=%s start_ms=%" PRIu64 " end_ms=%" PRIu64
           " n=%" PRIu32 " ADC_mean=%.1f min=%u max=%u sd=%.1f motors=OFF\n",
           kind,label,start/1000,end/1000,s->n,mean,s->lo,s->hi,
           sqrt(variance>0?variance:0));
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
    puts("\nBUDDY23 BARCODE ANALOG PROBE v2 | motors=OFF always");
    puts("Barcode A0 -> GP27/ADC1 (Pico W physical pin32). Front sensors unchanged; GP19/D0 unused.");
    puts("Sensor VCC=3.3V, common GND. Do not connect a 5V output to the ADC.");
    puts("USB-only stationary test. No motor-start command, calibration save or barcode decoding.");
    puts("Live ADC windows every100ms, nominal1kHz sampling; raw0..4095. No black polarity assumed.");
    puts("Keep still then press: w=broad white; b=broad black; g=narrowest white gap; n=adjacent black bar.");
    puts("Each key records1second then prints CAPTURE. Repeat g/n at least3times, normal mounted height.");
    puts("p=reprint last capture; h=help. Captures are USB/RAM only; save the serial text before power-off.");
}
int main(void) {
    for(unsigned pin=8;pin<=11;++pin) {
        gpio_init(pin);gpio_put(pin,0);gpio_set_dir(pin,GPIO_OUT);
    }
    adc_init();adc_gpio_init(ANALOG_PIN);adc_select_input(ANALOG_PIN-26);(void)adc_read();
    stdio_init_all();
    bool was_connected=false, capturing=false, have_last=false;
    const char *capture_label="NONE", *last_label="NONE";
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
        if(!connected) capturing=false;
        was_connected=connected;
        int ch=connected?getchar_timeout_us(0):PICO_ERROR_TIMEOUT;
        const char *label=label_for(ch);
        if(ch=='h') help();
        if(ch=='p') {
            if(have_last) print_stats("CAPTURE",last_label,&last,last_start,last_end);
            else puts("No completed capture yet.");
        }
        if(label) {
            if(capturing) puts("BUSY: keep still until CAPTURE completes.");
            else {
                capturing=true;capture=empty_stats();capture_label=label;capture_start=now;
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
            capturing=false;print_stats("CAPTURE",last_label,&last,last_start,last_end);
        }
        if(connected&&now>=next_report) {
            print_stats("LIVE","UNLABELLED",&live,live_start,now);
            live=empty_stats();live_start=now;next_report=now+100000;
        }
        sleep_us(100);
    }
}
