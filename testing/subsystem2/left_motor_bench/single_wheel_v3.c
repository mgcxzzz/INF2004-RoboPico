/* Standalone single-wheel diagnostic. Only one motor is driven at a time. */
#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "bench_logic.h"

enum { STOP_BUTTON=20, HOLD_BUTTON=21, PWM_TOP=999, PWM_HZ=10000 };
_Static_assert(PWM_TOP + 1 == 1000, "bench_levels uses a 1000-count PWM period");
static const uint motor_a[2] = {8,10}, motor_b[2] = {9,11};
static const uint enc_a[2] = {0,4}, enc_b[2] = {1,5};
static volatile BenchControl control;
static volatile BenchEncoder encoder[2];
static volatile uint8_t selected = 0, duty = 25;
static int32_t start_position;
static uint32_t start_invalid;
typedef struct {
    uint8_t wheel, duty;
    int64_t ticks;
    uint32_t invalid_delta;
} RunResult;
static volatile RunResult result;

static uint8_t read_ab(uint wheel) {
    uint32_t pins = gpio_get_all();
    return (uint8_t)((((pins >> enc_a[wheel]) & 1u) << 1) |
                      ((pins >> enc_b[wheel]) & 1u));
}
static void encoder_irq(uint pin, uint32_t events) {
    (void)events;
    uint wheel = (pin == enc_a[0] || pin == enc_b[0]) ? 0 : 1;
    bench_encoder_update(&encoder[wheel], read_ab(wheel));
}
static void drive_output(uint8_t drive) {
    for (uint w=0; w<2; ++w) {
        BenchLevels v = bench_levels(selected, (uint8_t)w, drive, duty);
        pwm_set_both_levels(pwm_gpio_to_slice_num(motor_a[w]),
                            v.a, v.b);
    }
}
/* Main calls these with IRQs masked; timer and GPIO IRQs have the same priority.
   Core 1 is not launched. Volatile does not replace the IRQ locking. */
static void capture_stop(void) {
    result.wheel = selected;
    result.duty = duty;
    result.ticks = (int64_t)encoder[selected].position - start_position;
    result.invalid_delta = encoder[selected].invalid - start_invalid;
}
static void stop_output(uint8_t reason) {
    bool active = control.drive != DRIVE_OFF;
    bench_stop(&control, time_us_64(), reason);
    drive_output(DRIVE_OFF);
    if (active) capture_stop();
}
static bool safety_timer(struct repeating_timer *timer) {
    (void)timer;
    bool active = control.drive != DRIVE_OFF;
    bench_tick(&control, time_us_64(), !gpio_get(HOLD_BUTTON), !gpio_get(STOP_BUTTON));
    drive_output(control.drive);
    if (active && !control.drive) capture_stop();
    return true;
}
static void help(void) {
    puts("\nBuddy 2 SINGLE WHEEL bench v3 | USB only | ONE wheel at a time");
    puts("L = LEFT: M1 GP8/9, encoder A=GP0 B=GP1 (default after reboot).");
    puts("R = RIGHT: M2 GP10/11, encoder A=GP4 B=GP5. Other motor stays off.");
    puts("1 = 15% PWM; 2 = 25% (default); 3 = 40%. Select with GP21 released.");
    puts("Keep wheels raised. Hold GP21 then f/r for opposite electrical directions.");
    puts("Maximum 3000 ms. Release GP21 / press GP20 / send s to stop.");
    puts("Release GP21 after every run; wait 1 second. No automatic runs.");
    puts("z = reset selected encoder while stopped; p = snapshot; h = help.");
    puts("CSV: ms,drive,held,stop,A,B,A_edges,B_edges,invalid,net,delta,cps,wheel,pwm");
    puts("wheel=L/R; pwm=selected duty, NOT proof that drive is on.");
}
static void snapshot(void) {
    uint32_t irq = save_and_disable_interrupts();
    uint8_t w = selected;
    BenchEncoder e = encoder[w];
    restore_interrupts(irq);
    printf("SNAPSHOT: wheel=%c net=%" PRId32 " A_edges=%" PRIu32
           " B_edges=%" PRIu32 " invalid=%" PRIu32 "\n",
           w ? 'R' : 'L', e.position, e.a_edges, e.b_edges, e.invalid);
}
int main(void) {
    for (uint pin=8; pin<=11; ++pin) {
        gpio_init(pin); gpio_put(pin, 0); gpio_set_dir(pin, GPIO_OUT);
    }
    for (uint w=0; w<2; ++w) {
        uint slice = pwm_gpio_to_slice_num(motor_a[w]);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, PWM_TOP);
        pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys)/(PWM_HZ*(PWM_TOP+1)));
        pwm_init(slice, &cfg, false);
        pwm_set_both_levels(slice, 0, 0);
        gpio_set_function(motor_a[w], GPIO_FUNC_PWM);
        gpio_set_function(motor_b[w], GPIO_FUNC_PWM);
        pwm_set_enabled(slice, true);
        gpio_init(enc_a[w]); gpio_set_dir(enc_a[w], GPIO_IN); gpio_pull_up(enc_a[w]);
        gpio_init(enc_b[w]); gpio_set_dir(enc_b[w], GPIO_IN); gpio_pull_up(enc_b[w]);
        encoder[w].ab = read_ab(w);
    }
    gpio_init(STOP_BUTTON); gpio_set_dir(STOP_BUTTON, GPIO_IN); gpio_pull_up(STOP_BUTTON);
    gpio_init(HOLD_BUTTON); gpio_set_dir(HOLD_BUTTON, GPIO_IN); gpio_pull_up(HOLD_BUTTON);
    gpio_set_irq_enabled_with_callback(enc_a[0], GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,
                                       true, encoder_irq);
    gpio_set_irq_enabled(enc_b[0], GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(enc_a[1], GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(enc_b[1], GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    control.consumed = true;
    struct repeating_timer timer;
    if (!add_repeating_timer_us(-1000, safety_timer, NULL, &timer))
        while (true) tight_loop_contents();
    stdio_init_all();
    bool was_connected = false;
    uint32_t reported_stop = 0;
    uint64_t last_report = time_us_64();
    int32_t last_position = 0;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) help();
        if (!connected) {
            uint32_t irq = save_and_disable_interrupts();
            stop_output(STOP_USB);
            restore_interrupts(irq);
        }
        was_connected = connected;
        /* Report the previous stop before accepting a new start/configuration. */
        uint32_t irq = save_and_disable_interrupts();
        uint32_t stops = control.stop_count;
        uint8_t reason = control.stop_reason;
        uint64_t run_us = control.last_run_us;
        RunResult r = result;
        restore_interrupts(irq);
        if (connected && stops != reported_stop) {
            static const char *const names[] = {"none", "GP20 pressed", "GP21 released",
                "3000 ms timeout", "serial s", "USB disconnected"};
            printf("STOP: %s | run_ms=%" PRIu64 " | drive=0 | wheel=%c pwm=%u"
                   " ticks_at_cutoff=%" PRId64 " invalid_delta=%" PRIu32 "\n",
                   names[reason], run_us/1000, r.wheel ? 'R' : 'L', r.duty,
                   r.ticks, r.invalid_delta);
            reported_stop = stops;
        }
        int ch = getchar_timeout_us(0);
        if (ch == 'h') help();
        else if (ch == 'p') snapshot();
        else if (ch == 'L' || ch == 'R' || ch == '1' || ch == '2' || ch == '3') {
            irq = save_and_disable_interrupts();
            bool allowed = !control.drive && gpio_get(HOLD_BUTTON);
            if (allowed) {
                if (ch == 'L' || ch == 'R') selected = ch == 'R';
                else duty = ch == '1' ? 15 : ch == '2' ? 25 : 40;
                drive_output(DRIVE_OFF);
                last_position = encoder[selected].position;
                last_report = time_us_64();
            }
            uint8_t w = selected, d = duty;
            restore_interrupts(irq);
            if (allowed) printf("CONFIG: wheel=%c pwm=%u | drive=0\n", w ? 'R' : 'L', d);
            else puts("CONFIG rejected: stop and release GP21 first.");
        } else if (ch == 'f' || ch == 'r' || ch == 's' || ch == 'z') {
            irq = save_and_disable_interrupts();
            bool accepted = false;
            if (ch == 's') stop_output(STOP_SERIAL);
            else if (ch == 'z' && !control.drive) {
                encoder[selected] = (BenchEncoder){.ab=read_ab(selected)};
                last_position = 0; last_report = time_us_64();
            } else if (connected && (ch == 'f' || ch == 'r')) {
                accepted = bench_start(&control, time_us_64(), !gpio_get(HOLD_BUTTON),
                            !gpio_get(STOP_BUTTON), ch == 'f' ? DRIVE_A : DRIVE_B);
                if (accepted) {
                    start_position = encoder[selected].position;
                    start_invalid = encoder[selected].invalid;
                }
            }
            drive_output(control.drive);
            uint8_t w = selected, d = duty;
            restore_interrupts(irq);
            if (ch == 'f' || ch == 'r') {
                if (accepted) printf("PULSE accepted | wheel=%c pwm=%u command=%c\n",
                                      w ? 'R' : 'L', d, ch);
                else puts("PULSE rejected: release/repress GP21, hold it, release GP20, wait 1 s.");
            }
        }
        uint64_t now = time_us_64();
        if (connected && now-last_report >= 250000) {
            irq = save_and_disable_interrupts();
            uint8_t w = selected, d = duty, drive = control.drive;
            BenchEncoder e = encoder[w];
            restore_interrupts(irq);
            int64_t delta = (int64_t)e.position-last_position;
            int64_t cps = delta*1000000/(int64_t)(now-last_report);
            printf("%" PRIu64 ",%u,%u,%u,%u,%u,%" PRIu32 ",%" PRIu32 ",%" PRIu32
                   ",%" PRId32 ",%" PRId64 ",%" PRId64 ",%c,%u\n", now/1000,
                   drive, !gpio_get(HOLD_BUTTON), !gpio_get(STOP_BUTTON), (e.ab>>1)&1,
                   e.ab&1, e.a_edges, e.b_edges, e.invalid, e.position, delta, cps,
                   w ? 'R' : 'L', d);
            last_report = now; last_position = e.position;
        }
        sleep_ms(1);
    }
}
