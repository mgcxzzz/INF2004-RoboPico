#include "src/platform/motor_output.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/structs/pwm.h"
static const unsigned motor_a[2] = {8, 10}, motor_b[2] = {9, 11};
void platform_motor_init(void) {
    for (unsigned p = 8; p <= 11; ++p) {
        gpio_init(p);
        gpio_put(p, 0);
        gpio_set_dir(p, GPIO_OUT);
    }
    for (unsigned w = 0; w < 2; ++w) {
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, 999);
        pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (10000 * 1000));
        unsigned slice = pwm_gpio_to_slice_num(motor_a[w]);
        pwm_init(slice, &cfg, false);
        pwm_set_both_levels(slice, 0, 0);
        gpio_set_function(motor_a[w], GPIO_FUNC_PWM);
        gpio_set_function(motor_b[w], GPIO_FUNC_PWM);
        pwm_set_enabled(slice, true);
    }
}
void platform_motor_write(unsigned w, uint16_t a, uint16_t b) {
    if (w < 2)
        pwm_set_both_levels(pwm_gpio_to_slice_num(motor_a[w]), a, b);
}
void platform_motor_off(void) {
    for (unsigned w = 0; w < 2; ++w)
        platform_motor_write(w, 0, 0);
}
int16_t platform_motor_read(unsigned w) {
    if (w > 1)
        return 0;
    uint32_t cc = pwm_hw->slice[pwm_gpio_to_slice_num(motor_a[w])].cc;
    int a = (int)(cc & 65535u), b = (int)(cc >> 16);
    return (int16_t)(w ? b - a : a - b);
}
