/* Analog calibration only. No RTOS, motor PWM, motion commands, or GP19 use.
 * GPIO0/1 remain available to the encoders; stdio uses Pico USB, never UART.
 * One core, one ADC owner. Motor-driver inputs are held low before USB starts. */
#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"

enum { LEFT_AO=26, RIGHT_AO=28, SAMPLES=32 };
_Static_assert(LEFT_AO==26 && RIGHT_AO==28,"Wiring must match ADC0 / ADC2");
typedef struct { uint16_t average,minimum,maximum; } Reading;

static void motors_off(void) {
    for(unsigned pin=8;pin<=11;++pin) gpio_put(pin,0);
}
static Reading sample(unsigned channel) {
    adc_select_input(channel);
    /* Discard the first conversion after multiplexing, then average a burst. */
    (void)adc_read();
    uint32_t sum=0;
    Reading r={.minimum=4095};
    for(unsigned i=0;i<SAMPLES;++i) {
        uint16_t v=adc_read();
        sum+=v;
        if(v<r.minimum) r.minimum=v;
        if(v>r.maximum) r.maximum=v;
        sleep_us(20);
    }
    r.average=(uint16_t)((sum+SAMPLES/2)/SAMPLES);
    return r;
}
static void help(void) {
    puts("\nBUDDY23 ANALOG LINE PROBE v1 | MOTORS OFF | USB STDIO ONLY");
    puts("LEFT sensor A0 -> GP26/ADC0; RIGHT sensor A0 -> GP28/ADC2.");
    puts("Both sensor VCC=3.3V, GND=common GND. D0 unused. GP19 untouched.");
    puts("No button/key can start motors. h=help; 32 ADC samples/channel per line.");
    puts("ADC range 0..4095; black may be HIGHER or LOWER: measure both surfaces.");
    puts("L/R=averages; Lmin/Lmax/Rmin/Rmax=within-burst ranges (noise).");
    puts("L_mV/R_mV assume 3300mV reference: estimates, not multimeter readings.");
    puts("Keep normal mat height. Record BOTH white, BOTH black, then edge sweep.");
    puts("No calibration is stored; this probe never writes flash. Previous test logs are not read.");
}
int main(void) {
    for(unsigned pin=8;pin<=11;++pin) {
        gpio_init(pin);gpio_put(pin,0);gpio_set_dir(pin,GPIO_OUT);
    }
    adc_init();
    adc_gpio_init(LEFT_AO);
    adc_gpio_init(RIGHT_AO);
    stdio_init_all();
    bool previous=false;
    uint32_t seq=0;
    uint64_t next=0;
    for(;;) {
        motors_off();
        bool connected=stdio_usb_connected();
        if(connected && !previous) { help();next=0; }
        previous=connected;
        int ch=connected?getchar_timeout_us(0):PICO_ERROR_TIMEOUT;
        if(ch=='h') help();
        uint64_t now=time_us_64();
        if(connected && now>=next) {
            Reading l=sample(LEFT_AO-26),r=sample(RIGHT_AO-26);
            printf("ADC: seq=%" PRIu32 " ms=%" PRIu32
                " L=%u R=%u Lmin=%u Lmax=%u Rmin=%u Rmax=%u L_mV=%u R_mV=%u motors=OFF\n",
                seq++,(uint32_t)(now/1000),l.average,r.average,
                l.minimum,l.maximum,r.minimum,r.maximum,
                (unsigned)((uint32_t)l.average*3300u/4095u),
                (unsigned)((uint32_t)r.average*3300u/4095u));
            next=now+200000;
        }
        sleep_ms(1);
    }
}
