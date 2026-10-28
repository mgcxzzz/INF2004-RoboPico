#ifndef UTK_COMPAT_HARDWARE_ADC_H
#define UTK_COMPAT_HARDWARE_ADC_H
#include <stdint.h>
typedef unsigned int uint;
void utk_adc_init(void);
void utk_adc_gpio_init(uint gpio);
void utk_adc_select_input(uint input);
uint16_t utk_adc_read(void);
#define adc_init utk_adc_init
#define adc_gpio_init utk_adc_gpio_init
#define adc_select_input utk_adc_select_input
#define adc_read utk_adc_read
#endif
