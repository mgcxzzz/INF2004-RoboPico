/**
 * @file    s3_ir.c
 * @brief   The three IR sensors: front pair polled, barcode sensor timed
 *          by edge interrupts.
 *
 * GP19 interrupt: every edge is stored as ONE 32-bit word,
 *     bits 31..1  sys_time_us() at the edge (bit 0 of the time dropped)
 *     bit  0      1 = black after the edge
 * in a ring that ONLY the interrupt writes (head) and ONLY the task reads
 * (tail). The word is written first, then a barrier, then the head index
 * (one byte store), so the task can never see a head that points at a
 * half-written entry, on either core. A full ring drops the edge and
 * counts it (overflow counter, written only by the interrupt).
 *
 * The interrupt goes through the shared GPIO dispatcher
 * (system/sys_hal.c), the same one Buddy 2's encoders and Buddy 5's echo
 * use: the RP2040 has a single GPIO interrupt.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "../system/sys_hal.h"
#include "s3_config.h"
#include "s3_iface.h"
#include "s3_line.h"

#define RING_MASK       (S3_BC_EDGE_RING - 1U)
#define S3_DMB()        __asm volatile ("dmb" ::: "memory")

_Static_assert((S3_BC_EDGE_RING & RING_MASK) == 0U,
               "edge ring size must be a power of two");
_Static_assert(S3_BC_EDGE_RING <= 256U, "ring indices are 8 bits");

static volatile uint32_t g_ring[S3_BC_EDGE_RING];
static volatile uint8_t  g_head = 0U;       /* written by the ISR only  */
static uint8_t           g_tail = 0U;       /* written by the task only */
static volatile uint8_t  g_overflows = 0U;  /* written by the ISR only  */

static bool
is_black (uint32_t pin)
{
    return (gpio_get_val(pin) == S3_IR_BLACK_LEVEL);
}

/* GP19 edge (interrupt context): timestamp it, nothing more. */
static void
barcode_edge (uint32_t gpio, uint32_t edges, uint32_t t_us)
{
    uint8_t  head = g_head;
    uint8_t  next = (uint8_t)((head + 1U) & RING_MASK);
    uint32_t word;

    (void)gpio;
    (void)edges;

    /* The level now, not the edge flag: if two edges latched before the
     * dispatcher ran, the level is still right and the reader simply
     * sees no change. */
    word = (t_us & ~1U) | (is_black(S3_PIN_BARCODE) ? 1U : 0U);

    if (next == g_tail)
    {
        g_overflows = (uint8_t)(g_overflows + 1U);
        return;
    }

    g_ring[head] = word;
    S3_DMB();
    g_head = next;
}

ER
s3_ir_init (void)
{
    ER er = gpio_set_pin(S3_PIN_LINE_LEFT, GPIO_MODE_IN);

    if (er == E_OK)
    {
        er = gpio_set_pin(S3_PIN_LINE_RIGHT, GPIO_MODE_IN);
    }

    if (er == E_OK)
    {
        er = gpio_set_pin(S3_PIN_BARCODE, GPIO_MODE_IN);
    }

    if (er == E_OK)
    {
        er = sys_gpio_irq_attach(S3_PIN_BARCODE,
                                 SYS_GPIO_EDGE_RISE | SYS_GPIO_EDGE_FALL,
                                 barcode_edge, S3_BARCODE_IRQ_LEVEL);
    }

    return er;
}

uint32_t
s3_ir_read (void)
{
    uint32_t bits = 0U;

    bits |= is_black(S3_PIN_LINE_LEFT)  ? S3_SENSE_LEFT    : 0U;
    bits |= is_black(S3_PIN_LINE_RIGHT) ? S3_SENSE_RIGHT   : 0U;
    bits |= is_black(S3_PIN_BARCODE)    ? S3_SENSE_BARCODE : 0U;

    return bits;
}

bool
s3_ir_pop_edge (uint32_t * p_t_us, bool * p_b_black)
{
    uint32_t word;

    if (g_tail == g_head)
    {
        return false;
    }

    S3_DMB();                           /* entry after the head read */
    word       = g_ring[g_tail];
    g_tail     = (uint8_t)((g_tail + 1U) & RING_MASK);
    *p_t_us    = word & ~1U;
    *p_b_black = ((word & 1U) != 0U);

    return true;
}

uint32_t
s3_ir_overflows (void)
{
    return (uint32_t)g_overflows;
}

uint32_t
s3_time_us (void)
{
    return sys_time_us();
}
