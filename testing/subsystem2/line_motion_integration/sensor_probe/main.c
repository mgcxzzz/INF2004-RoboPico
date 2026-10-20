/* Read-only IR diagnosis. No RTOS, encoder/barcode GPIO IRQs or motor commands.
   Built with the same Pico SDK USB console path as the working motor bench. */
#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
static void off(void) {
    for(unsigned pin=8;pin<=11;++pin) gpio_put(pin,0);
}
static void help(void) {
    puts("\nBUDDY23 IR SENSOR PROBE v1 | USB ONLY | MOTOR OUTPUTS HELD LOW");
    puts("GP6=left, GP7=right, GP19=side/barcode. Raw electrical levels only.");
    puts("No movement commands. Onboard buttons do not start any test.");
    puts("Every 250 ms: increasing sequence and uptime, L/R/B readings.");
    puts("Use normal mat height; compare white / black under each sensor.");
    puts("h repeats this help. USB local echo is off: typed letters need not show.");
}
int main(void) {
    for(unsigned pin=8;pin<=11;++pin) {
        gpio_init(pin);gpio_put(pin,0);gpio_set_dir(pin,GPIO_OUT);
    }
    const unsigned sensors[]={6,7,19};
    for(unsigned i=0;i<3;++i) {
        gpio_init(sensors[i]);gpio_set_dir(sensors[i],GPIO_IN);gpio_disable_pulls(sensors[i]);
    }
    stdio_init_all();
    bool previous=false;
    uint32_t seq=0;
    while(true) {
        off();
        bool connected=stdio_usb_connected();
        if(connected && !previous) help();
        previous=connected;
        int ch=getchar_timeout_us(0);
        if(ch=='h') help();
        if(connected) {
            uint32_t levels=gpio_get_all();
            printf("IR: seq=%" PRIu32 " ms=%" PRIu32 " rawL=%u rawR=%u rawB=%u motors=OFF\n",
                seq,(uint32_t)to_ms_since_boot(get_absolute_time()),
                (unsigned)((levels>>6)&1u),(unsigned)((levels>>7)&1u),(unsigned)((levels>>19)&1u));
        }
        ++seq;
        sleep_ms(250);
    }
}
