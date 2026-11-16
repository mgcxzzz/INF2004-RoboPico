/**
 * @file board_port.c
 * @brief Board port.
 * @details Core1 owns normal motor commits. Safety paths may only remove
 * output; command/status access uses the existing kernel locks and copied
 * records.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* RP2040 port. Motor outputs have one task owner and a safety-only OFF path.
 * Encoder wiring, quadrature table, PWM polarity/frequency and conversion
 * constants are the unchanged main-branch values. */
#include "bringup.h"
#include "src/motion/encoder_and_guard.h"

#define BR_REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define BR_PWM(slice, offset) BR_REG(0x40050000U + 0x14U * (slice) + (offset))
#define BR_IO_INTR(n) (0x400140f0U + 4U * (n))
#define BR_IO_INTE(n) (0x40014130U + 4U * (n)) /* processor 2, core 1 */
#define BR_IO_INTS(n) (0x40014150U + 4U * (n))
#define BR_IO_IRQ_BANK0 13U

static T_SPLOCK g_motion_lock;
static volatile bench_encoder_t g_encoders[2];
volatile uint32_t g_bringup_irq_stop;
static const unsigned g_encoder_a[2] = {0U, 4U};
static const unsigned g_encoder_b[2] = {1U, 5U};

uintptr_t board_motion_lock (void * p_user_value)
{
    UINT key;
    (void)p_user_value;
    (void)ISpinLock(&g_motion_lock, &key);
    return key;
}

void board_motion_unlock (void * p_user_value, uintptr_t key)
{
    (void)p_user_value;
    (void)ISpinUnlock(&g_motion_lock, (UINT)key);
}

uint64_t board_time_us (void)
{
    uint32_t high, low, after;
    do
    {
        high = BR_REG(0x40054024U);
        low = BR_REG(0x40054028U);
        after = BR_REG(0x40054024U);
    } while (high != after);
    return ((uint64_t)high << 32U) | low;
}

uint32_t board_kernel_ms (void)
{
    SYSTIM now;
    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

bool board_button_pressed (unsigned pin)
{
    return 0U == (in_w(GPIO_IN) & (1U << pin));
}

static uint8_t read_ab (unsigned wheel)
{
    uint32_t pins = in_w(GPIO_IN);
    return (uint8_t)((((pins >> g_encoder_a[wheel]) & 1U) << 1U) |
                     ((pins >> g_encoder_b[wheel]) & 1U));
}

void board_motor_off (void)
{
    BR_PWM(4U, 0x0cU) = 0U;
    BR_PWM(5U, 0x0cU) = 0U;
}

void board_motor_init (void)
{
    /* Drive low as SIO before changing PWM muxes; never expose a stale CC. */
    out_w(GPIO_OUT_CLR, 0xf00U);
    out_w(GPIO_OE_SET, 0xf00U);
    for (unsigned pin = 8U; pin <= 11U; ++pin)
    {
        clr_w(GPIO(pin), GPIO_OD);
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    }
    /* Release the block without resetting any kernel-owned PWM slice. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
    }
    for (unsigned slice = 4U; slice <= 5U; ++slice)
    {
        BR_PWM(slice, 0U) = 0U;
        BR_PWM(slice, 0x04U) =
            200U; /* 12.5 in 8.4: 125MHz / 12.5 / 1000 = 10kHz. */
        BR_PWM(slice, 0x08U) = 0U;
        BR_PWM(slice, 0x0cU) = 0U;
        BR_PWM(slice, 0x10U) = 999U;
    }
    for (unsigned pin = 8U; pin <= 11U; ++pin)
    {
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PWM);
    }
    BR_PWM(4U, 0U) = 1U;
    BR_PWM(5U, 0U) = 1U;
}

void board_motor_write (unsigned wheel, unsigned pwm)
{
    if (wheel > 1U)
    {
        return;
    }
    if (pwm > 420U)
    {
        pwm = 420U;
    }
    BR_PWM(4U + wheel, 0x0cU) = 0U == wheel ? pwm : pwm << 16U;
}

static void board_gpio_isr (UINT vector)
{
    (void)vector;
    uint32_t first = BR_REG(BR_IO_INTS(0U));
    uint32_t buttons = BR_REG(BR_IO_INTS(2U));
    BR_REG(BR_IO_INTR(0U)) = first & 0x00ff00ffU;
    BR_REG(BR_IO_INTR(2U)) = buttons & 0x00ff0000U;
    if ((first & 0xffU) != 0U)
    {
        bench_encoder_update(&g_encoders[0], read_ab(0U));
    }
    if ((first & 0x00ff0000U) != 0U)
    {
        bench_encoder_update(&g_encoders[1], read_ab(1U));
    }
    if (((buttons & (4U << 16U)) != 0U) || (board_button_pressed(20U)))
    {
        /* IRQ never calls the task-only motion API. Its latch is checked
         * inside every generation-checked commit before PWM is written. */
        g_bringup_irq_stop = 1U;
        board_motor_off();
    }
}

bool board_gpio_init (void)
{
    const unsigned pins[] = {0U, 1U, 4U, 5U, 20U, 21U};
    for (unsigned index = 0U; index < sizeof(pins) / sizeof(pins[0]); ++index)
    {
        unsigned pin = pins[index];
        out_w(GPIO_OE_CLR, 1U << pin);
        clr_w(GPIO(pin), GPIO_OD | GPIO_PDE);
        set_w(GPIO(pin), GPIO_IE | GPIO_PUE);
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    }
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
        g_encoders[wheel].ab = read_ab(wheel);
    }
    T_DINT definition = {0};
    definition.intatr = TA_HLNG;
    definition.inthdr = board_gpio_isr;
    if (tk_def_int(BR_IO_IRQ_BANK0, &definition) != E_OK)
    {
        g_bringup_irq_stop = 1U;
        return false;
    }
    BR_REG(BR_IO_INTR(0U)) = 0x00cc00ccU;
    BR_REG(BR_IO_INTR(2U)) = 4U << 16U;
    BR_REG(BR_IO_INTE(0U)) = 0x00cc00ccU;
    BR_REG(BR_IO_INTE(2U)) = 4U << 16U;
    ClearInt(BR_IO_IRQ_BANK0);
    EnableInt(BR_IO_IRQ_BANK0, 0U);
    return true;
}

void board_encoder_drain (int32_t delta[2], uint32_t edges[2],
                          uint32_t invalid[2])
{
    UINT key;
    DI(key); /* Only the owning core's GPIO IRQ changes encoder state. */
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
        delta[wheel] = g_encoders[wheel].position;
        edges[wheel] = g_encoders[wheel].a_edges + g_encoders[wheel].b_edges;
        invalid[wheel] = g_encoders[wheel].invalid;
        g_encoders[wheel].position = 0;
    }
    EI(key);
}

bool board_clock_registers_ok (void)
{
    return 0x60cU == (BR_REG(0x4005802cU) & 0x7ffU) &&
           4U == BR_REG(0x40008038U) && 0x100U == BR_REG(0x40008034U) &&
           2U == BR_REG(0x40008044U) && 0x100U == BR_REG(0x40008040U);
}

void board_clock_report (void)
{
    tm_printf((UB *)"CLOCK_REG core=%u tick=0x%x ref_selected=0x%x "
                    "ref_div=0x%x sys_selected=0x%x sys_div=0x%x "
                    "systick_reload=%u pll_fb=%u pll_prim=0x%x\n",
              tk_get_prc() - 1U, BR_REG(0x4005802cU), BR_REG(0x40008038U),
              BR_REG(0x40008034U), BR_REG(0x40008044U), BR_REG(0x40008040U),
              BR_REG(0xe000e014U), BR_REG(0x40028008U), BR_REG(0x4002800cU));
}

/* End of file. */
