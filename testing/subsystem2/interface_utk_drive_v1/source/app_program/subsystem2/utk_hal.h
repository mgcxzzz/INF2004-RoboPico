#ifndef UTK_DRIVE_HAL_H
#define UTK_DRIVE_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* The controller is a single kernel task on processor 1.  These callbacks
 * are adapters to the existing UTK shared GPIO IRQ and USB console services;
 * this header deliberately does not include T-Kernel or libc headers. */

/* A bounded RX dequeue supplied by the UTK USB CDC console service. */
int tm_usb_try_read(void);
uint32_t tm_usb_state(void);

void utk_console_write(const char *data, unsigned int length);
int utk_printf(const char *format, ...);
int utk_puts(const char *text);
int utk_putchar(int value);
int32_t utk_kernel_delay(uint32_t delay_ms);
int32_t utk_kernel_define_int(uint32_t intno, void (*handler)(unsigned int));
void utk_kernel_enable_int(uint32_t intno, int32_t level);
void utk_kernel_clear_int(uint32_t intno);
int32_t utk_kernel_gpio_attach(uint32_t gpio, uint32_t edges,
                               void (*callback)(uint32_t, uint32_t, uint32_t),
                               int32_t level);
void utk_kernel_unreset(uint32_t mask);
int32_t utk_kernel_spinlock_claim_unused(void);
int32_t utk_kernel_spinlock_lock(uint32_t number, uint32_t *irq_state);
int32_t utk_kernel_spinlock_unlock(uint32_t number, uint32_t irq_state);
bool utk_flash_begin(void);
void utk_flash_end(void);
void utk_flash_service(void);

#endif
