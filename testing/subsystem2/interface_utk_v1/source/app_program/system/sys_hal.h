/**
 * @file    sys_hal.h
 * @brief   Small RP2040 hardware helpers shared by the whole team.
 *
 *  - a free-running 1 MHz microsecond clock (RP2040 TIMER block)
 *  - a shared IO_BANK0 GPIO interrupt dispatcher
 *
 * The RP2040 has ONE GPIO interrupt (IO_IRQ_BANK0, IRQ 13) for all pins,
 * and the kernel lets only one handler own it. Buddy 2's encoder edges
 * (GP0/GP1, GP4/GP5) and Buddy 5's echo edges (GP17) must therefore share
 * it. sys_gpio_irq_attach() lets each owner register a callback for its own
 * pins; the dispatcher calls it with the edge type and a timestamp.
 *
 * sys_hal_init() is called once by system_init(), before any subsystem.
 *
 * Group IS28 (shared, written by Buddy 5). Coding standard: Barr-C:2018.
 */

#ifndef SYS_HAL_H
#define SYS_HAL_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Edge flags passed to and from the GPIO dispatcher. */
#define SYS_GPIO_EDGE_FALL   (0x4U)  /* IO_BANK0 EDGE_LOW bit  */
#define SYS_GPIO_EDGE_RISE   (0x8U)  /* IO_BANK0 EDGE_HIGH bit */

/**
 * GPIO edge callback. Runs in interrupt context: keep it short and only
 * call kernel services allowed from a handler (e.g. tk_set_flg).
 * @param gpio    pin number 0..29
 * @param edges   SYS_GPIO_EDGE_RISE and/or SYS_GPIO_EDGE_FALL
 * @param t_us    sys_time_us() captured on entry to the interrupt
 */
typedef void (* sys_gpio_cb_t)(uint32_t gpio, uint32_t edges, uint32_t t_us);

ER       sys_hal_init(void);
uint32_t sys_time_us(void);
void     sys_busy_wait_us(uint32_t us);
ER       sys_gpio_irq_attach(uint32_t gpio, uint32_t edges, sys_gpio_cb_t cb,
                            INT irq_level);
void     sys_unreset(uint32_t mask);

#endif /* SYS_HAL_H */
