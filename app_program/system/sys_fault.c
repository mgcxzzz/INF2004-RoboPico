/**
 * @file    sys_fault.c
 * @brief   Shared fault bitmask in a µT-Kernel event flag (see sys_fault.h).
 *
 * Group IS28 (team). Coding standard: Barr-C:2018.
 */

#include "sys_fault.h"

static ID g_fault_flg = 0;

/**
 * @brief  Create the event flag that holds the mask (all bits clear).
 */
ER
sys_fault_init (void)
{
    T_CFLG cflg = { 0 };

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_fault_flg  = tk_cre_flg(&cflg);

    return (g_fault_flg < E_OK) ? g_fault_flg : E_OK;
}

/**
 * @brief  Set fault bits. The kernel ORs them in as one operation.
 */
ER
sys_fault_set (uint32_t bits)
{
    if (g_fault_flg <= 0)
    {
        return E_NOEXS;
    }

    return tk_set_flg(g_fault_flg, (UINT)(bits & SYS_FAULT_ALL));
}

/**
 * @brief  Clear fault bits (tk_clr_flg keeps the bits that are 1 in the
 *         pattern, so pass the complement).
 */
ER
sys_fault_clear (uint32_t bits)
{
    if (g_fault_flg <= 0)
    {
        return E_NOEXS;
    }

    return tk_clr_flg(g_fault_flg, (UINT)~(bits & SYS_FAULT_ALL));
}

/**
 * @brief  Current mask, read without waiting.
 */
uint32_t
sys_fault_get (void)
{
    T_RFLG rflg;

    if ((g_fault_flg <= 0) || (tk_ref_flg(g_fault_flg, &rflg) != E_OK))
    {
        return 0U;
    }

    return (uint32_t)rflg.flgptn & SYS_FAULT_ALL;
}
