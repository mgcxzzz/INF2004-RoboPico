#ifndef UTK_COMPAT_HARDWARE_PWM_H
#define UTK_COMPAT_HARDWARE_PWM_H
#include <stdbool.h>
#include <stdint.h>
typedef unsigned int uint;
typedef struct { uint32_t csr, div, top; } pwm_config;
typedef struct { volatile uint32_t csr, div, ctr, cc, top; } utk_pwm_slice_hw_t;
typedef struct { utk_pwm_slice_hw_t slice[8]; } utk_pwm_hw_t;
volatile utk_pwm_hw_t *utk_pwm_hardware(void);
#define pwm_hw (utk_pwm_hardware())
pwm_config utk_pwm_get_default_config(void);
void utk_pwm_config_set_wrap(pwm_config *config, uint16_t wrap);
void utk_pwm_config_set_clkdiv(pwm_config *config, float divider);
uint utk_pwm_gpio_to_slice_num(uint gpio);
void utk_pwm_init(uint slice, const pwm_config *config, bool start);
void utk_pwm_set_both_levels(uint slice, uint16_t level_a, uint16_t level_b);
void utk_pwm_set_enabled(uint slice, bool enabled);
#define pwm_get_default_config utk_pwm_get_default_config
#define pwm_config_set_wrap utk_pwm_config_set_wrap
#define pwm_config_set_clkdiv utk_pwm_config_set_clkdiv
#define pwm_gpio_to_slice_num utk_pwm_gpio_to_slice_num
#define pwm_init utk_pwm_init
#define pwm_set_both_levels utk_pwm_set_both_levels
#define pwm_set_enabled utk_pwm_set_enabled
#endif
