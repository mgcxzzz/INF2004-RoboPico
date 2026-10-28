/* Narrow kernel adapters kept separate from libc-facing Pico shims. */
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tk/smp.h>

#include "system/sys_hal.h"
#include "utk_hal.h"

#define UTK_HAL_SPINLOCK_COUNT 32u

static T_SPLOCK g_hal_spinlocks[UTK_HAL_SPINLOCK_COUNT];
static UW g_hal_spinlocks_claimed;

int32_t utk_kernel_delay(uint32_t delay_ms)
{
    return (int32_t)tk_dly_tsk(delay_ms);
}

void utk_kernel_timebase_report(void)
{
    sys_timebase_report();
}

int32_t utk_kernel_define_int(uint32_t intno, void (*handler)(unsigned int))
{
    T_DINT definition;
    definition.intatr = TA_HLNG;
    definition.inthdr = (FP)handler;
    return (int32_t)tk_def_int(intno, &definition);
}

void utk_kernel_enable_int(uint32_t intno, int32_t level)
{
    EnableInt(intno, level);
}

void utk_kernel_clear_int(uint32_t intno)
{
    ClearInt(intno);
}

int32_t utk_kernel_gpio_attach(uint32_t gpio, uint32_t edges,
                               void (*callback)(uint32_t, uint32_t, uint32_t),
                               int32_t level)
{
    return (int32_t)sys_gpio_irq_attach(gpio, edges, callback, level);
}

void utk_kernel_unreset(uint32_t mask)
{
    sys_unreset(mask);
}

int32_t utk_kernel_spinlock_claim_unused(void)
{
    for (uint32_t number = 0u; number < UTK_HAL_SPINLOCK_COUNT; number++) {
        UW bit = (UW)1u << number;
        UW previous = atomic_bitset(&g_hal_spinlocks_claimed, bit);
        if ((previous & bit) == 0u) {
            ER result = InitSpinLock(&g_hal_spinlocks[number]);
            if (result == E_OK) return (int32_t)number;
            (void)atomic_bitclr(&g_hal_spinlocks_claimed, bit);
            return -1;
        }
    }
    return -1;
}

int32_t utk_kernel_spinlock_lock(uint32_t number, uint32_t *irq_state)
{
    UINT int_state;
    ER result;
    if (number >= UTK_HAL_SPINLOCK_COUNT || irq_state == NULL) return (int32_t)E_PAR;
    result = ISpinLock(&g_hal_spinlocks[number], &int_state);
    if (result == E_OK) *irq_state = (uint32_t)int_state;
    return (int32_t)result;
}

int32_t utk_kernel_spinlock_unlock(uint32_t number, uint32_t irq_state)
{
    if (number >= UTK_HAL_SPINLOCK_COUNT) return (int32_t)E_PAR;
    return (int32_t)ISpinUnlock(&g_hal_spinlocks[number], (UINT)irq_state);
}
