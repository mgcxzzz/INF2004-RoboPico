#ifndef UTK_COMPAT_HARDWARE_IRQ_H
#define UTK_COMPAT_HARDWARE_IRQ_H
#include <stdbool.h>
#include <stdint.h>
typedef unsigned int uint;
enum { IO_IRQ_BANK0 = 13u };
void utk_irq_set_priority(uint irq_num, uint8_t priority);
void utk_irq_set_enabled(uint irq_num, bool enabled);
#define irq_set_priority utk_irq_set_priority
#define irq_set_enabled utk_irq_set_enabled
#endif
