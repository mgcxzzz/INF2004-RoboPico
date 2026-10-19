/* Standalone Pico SDK bench firmware, NOT the micro T-Kernel application. */
#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "bench_logic.h"

enum { LEFT_A=8, LEFT_B=9, RIGHT_A=10, RIGHT_B=11,
       ENC_A=0, ENC_B=1, STOP_BUTTON=20, HOLD_BUTTON=21,
       PWM_TOP=999, PWM_HZ=10000, DUTY_PERCENT=25 };
_Static_assert(LEFT_A / 2 == LEFT_B / 2, "M1 must use one PWM slice");
/* Core 1 is never launched. Main/IRQ snapshots use short IRQ-disabled sections;
   volatile alone would NOT make compound accesses safe. */
static volatile BenchControl control;
static volatile BenchEncoder encoder;

static uint8_t read_ab(void) {
    uint32_t pins = gpio_get_all();
    return (uint8_t)((((pins >> ENC_A) & 1u) << 1) |
                      ((pins >> ENC_B) & 1u));
}
static void encoder_irq(uint pin, uint32_t events) {
    (void)pin; (void)events;
    bench_encoder_update(&encoder, read_ab());
}
static void drive_output(uint8_t drive) {
    /* Both PWM channels updated in a single register write. Zero removes drive;
       this is not a promise of an immediate mechanical stop. */
    uint16_t level = (PWM_TOP + 1) * DUTY_PERCENT / 100;
    pwm_set_both_levels(pwm_gpio_to_slice_num(LEFT_A),
                       drive == DRIVE_A ? level : 0,
                       drive == DRIVE_B ? level : 0);
}
static bool safety_timer(struct repeating_timer *timer) {
    (void)timer;
    bench_tick(&control, time_us_64(), !gpio_get(HOLD_BUTTON),
               !gpio_get(STOP_BUTTON));
    drive_output(control.drive);
    return true;
}
static void help(void) {
    puts("\nBuddy 2 LEFT M1 bench v1 | RIGHT M2 disabled | USB only");
    puts("Encoder A=GP0 B=GP1. Rotate gently by hand: watch A_edges/B_edges/net.");
    puts("Raise both wheels. Hold GP21, then send f or r (no Enter required).");
    puts("f = A-side PWM; r = B-side PWM. Chassis direction must be observed.");
    puts("Each pulse: 25% PWM, 500 ms maximum (+ ~1 ms timer response).");
    puts("Release GP21 / press GP20 / send s to stop. Release GP21 to rearm.");
    puts("Wait 1 second between pulses. z resets stopped encoder counters; h help.");
    puts("CSV: ms,drive,held,stop,A,B,A_edges,B_edges,invalid,net,delta,cps");
}
int main(void) {
    /* Establish inactive output latches BEFORE selecting output direction. */
    for (uint pin = LEFT_A; pin <= RIGHT_B; ++pin) {
        gpio_init(pin); gpio_put(pin, 0); gpio_set_dir(pin, GPIO_OUT);
    }
    uint slice = pwm_gpio_to_slice_num(LEFT_A);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_TOP);
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) /
                         (PWM_HZ * (PWM_TOP + 1)));
    pwm_init(slice, &cfg, false);
    pwm_set_both_levels(slice, 0, 0);
    gpio_set_function(LEFT_A, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_B, GPIO_FUNC_PWM);
    pwm_set_enabled(slice, true);
    /* M2 stays SIO output-low throughout this program; no M2 PWM is enabled. */
    gpio_init(STOP_BUTTON); gpio_set_dir(STOP_BUTTON, GPIO_IN); gpio_pull_up(STOP_BUTTON);
    gpio_init(HOLD_BUTTON); gpio_set_dir(HOLD_BUTTON, GPIO_IN); gpio_pull_up(HOLD_BUTTON);
    gpio_init(ENC_A); gpio_set_dir(ENC_A, GPIO_IN); gpio_pull_up(ENC_A);
    gpio_init(ENC_B); gpio_set_dir(ENC_B, GPIO_IN); gpio_pull_up(ENC_B);
    encoder.ab = read_ab();
    gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, encoder_irq);
    gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    control.consumed = true; /* Require GP21 released once after boot. */
    struct repeating_timer timer;
    if (!add_repeating_timer_us(-1000, safety_timer, NULL, &timer)) {
        /* Fail closed: no serial command processing if timeout timer fails. */
        while (true) tight_loop_contents();
    }
    stdio_init_all();
    bool connected_before = false;
    uint64_t last_report = time_us_64();
    int32_t last_position = 0;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !connected_before) help();
        if (!connected) {
            uint32_t irq = save_and_disable_interrupts();
            bench_stop(&control, time_us_64()); drive_output(DRIVE_OFF);
            restore_interrupts(irq);
        }
        connected_before = connected;
        int ch = getchar_timeout_us(0);
        bool accepted = false;
        if (ch == 'f' || ch == 'r' || ch == 's' || ch == 'z') {
            uint32_t irq = save_and_disable_interrupts();
            uint64_t now = time_us_64();
            if (ch == 's') bench_stop(&control, now);
            else if (ch == 'z' && control.drive == DRIVE_OFF) {
                encoder = (BenchEncoder){.ab = read_ab()};
                last_position = 0;
            } else if (connected && (ch == 'f' || ch == 'r')) {
                accepted = bench_start(&control, now, !gpio_get(HOLD_BUTTON),
                    !gpio_get(STOP_BUTTON), ch == 'f' ? DRIVE_A : DRIVE_B);
            }
            drive_output(control.drive);
            restore_interrupts(irq);
            if (ch == 'f' || ch == 'r') puts(accepted ? "PULSE accepted" :
                "PULSE rejected: release/repress GP21, hold it, release GP20, wait 1 s.");
        } else if (ch == 'h') help();
        uint64_t now = time_us_64();
        if (connected && now - last_report >= 250000) {
            uint32_t irq = save_and_disable_interrupts();
            BenchEncoder e = encoder;
            uint8_t drive = control.drive;
            restore_interrupts(irq);
            int64_t delta = (int64_t)e.position - last_position;
            int64_t cps = delta * 1000000 / (int64_t)(now - last_report);
            printf("%" PRIu64 ",%u,%u,%u,%u,%u,%" PRIu32 ",%" PRIu32
                   ",%" PRIu32 ",%" PRId32 ",%" PRId64 ",%" PRId64 "\n",
                   now / 1000, drive, !gpio_get(HOLD_BUTTON), !gpio_get(STOP_BUTTON),
                   (e.ab >> 1) & 1, e.ab & 1, e.a_edges, e.b_edges,
                   e.invalid, e.position, delta, cps);
            last_report = now; last_position = e.position;
        }
        sleep_ms(1);
    }
}
