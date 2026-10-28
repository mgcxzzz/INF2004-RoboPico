#ifndef UTK_COMPAT_PICO_STDLIB_H
#define UTK_COMPAT_PICO_STDLIB_H

#include <stdbool.h>
#include <stdint.h>
#include "utk_hal.h"
#include "hardware/timer.h"

typedef unsigned int uint;

#define PICO_ERROR_TIMEOUT (-1)
#define GPIO_IN 0u
#define GPIO_OUT 1u
#define GPIO_FUNC_PWM 4u
#define GPIO_FUNC_NULL 31u
#define GPIO_IRQ_EDGE_FALL 0x04u
#define GPIO_IRQ_EDGE_RISE 0x08u

void utk_gpio_init(uint gpio);
void utk_gpio_put(uint gpio, bool value);
void utk_gpio_set_dir(uint gpio, bool output);
void utk_gpio_set_function(uint gpio, uint function);
bool utk_gpio_get(uint gpio);
uint32_t utk_gpio_get_all(void);
void utk_gpio_pull_up(uint gpio);
void utk_gpio_pull_down(uint gpio);
void utk_gpio_disable_pulls(uint gpio);
void utk_gpio_set_irq_enabled_with_callback(uint gpio, uint32_t events,
                                             bool enabled,
                                             void (*callback)(uint, uint32_t));
void utk_gpio_set_irq_enabled(uint gpio, uint32_t events, bool enabled);
uint64_t time_us_64(void);
void utk_sleep_us(uint64_t us);
void utk_sleep_ms(uint32_t ms);
void utk_tight_loop_contents(void);
void utk_stdio_init_all(void);
bool utk_stdio_usb_connected(void);
int utk_getchar_timeout_us(uint32_t timeout_us);
void utk_stdio_flush(void);
int utk_printf(const char *format, ...);
int utk_puts(const char *text);
int utk_putchar(int value);
void watchdog_reboot(uint32_t delay_ms, uint32_t pc, uint32_t sp);

/* Keep the unmodified controller's SDK call sites intact. */
#define gpio_init                         utk_gpio_init
#define gpio_put                          utk_gpio_put
#define gpio_set_dir                      utk_gpio_set_dir
#define gpio_set_function                 utk_gpio_set_function
#define gpio_get                          utk_gpio_get
#define gpio_get_all                      utk_gpio_get_all
#define gpio_pull_up                      utk_gpio_pull_up
#define gpio_pull_down                    utk_gpio_pull_down
#define gpio_disable_pulls                utk_gpio_disable_pulls
#define gpio_set_irq_enabled_with_callback utk_gpio_set_irq_enabled_with_callback
#define gpio_set_irq_enabled              utk_gpio_set_irq_enabled
#define sleep_us                          utk_sleep_us
#define sleep_ms                          utk_sleep_ms
#define tight_loop_contents                utk_tight_loop_contents
#define stdio_init_all                    utk_stdio_init_all
#define stdio_usb_connected               utk_stdio_usb_connected
#define getchar_timeout_us                utk_getchar_timeout_us
#define stdio_flush                       utk_stdio_flush
#define printf                            utk_printf
#define puts                              utk_puts
#define putchar                           utk_putchar

#endif
