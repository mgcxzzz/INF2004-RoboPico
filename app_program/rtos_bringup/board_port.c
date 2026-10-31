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

static T_SPLOCK motion_lock;
static volatile BenchEncoder encoders[2];
volatile uint32_t bringup_irq_stop;
static const unsigned encoder_a[2] = {0U, 4U}, encoder_b[2] = {1U, 5U};

uintptr_t board_motion_lock(void *user) {
    UINT key;
    (void)user;
    (void)ISpinLock(&motion_lock, &key);
    return key;
}

void board_motion_unlock(void *user, uintptr_t key) {
    (void)user;
    (void)ISpinUnlock(&motion_lock, (UINT)key);
}

uint64_t board_time_us(void) {
    uint32_t hi, lo, after;
    do {
        hi = BR_REG(0x40054024U);
        lo = BR_REG(0x40054028U);
        after = BR_REG(0x40054024U);
    } while (hi != after);
    return ((uint64_t)hi << 32U) | lo;
}

uint32_t board_kernel_ms(void) {
    SYSTIM now;
    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

bool board_button_pressed(unsigned pin) {
    return (in_w(GPIO_IN) & (1U << pin)) == 0U;
}

static uint8_t read_ab(unsigned wheel) {
    uint32_t pins = in_w(GPIO_IN);
    return (uint8_t)((((pins >> encoder_a[wheel]) & 1U) << 1U) | ((pins >> encoder_b[wheel]) & 1U));
}

void board_motor_off(void) {
    BR_PWM(4U, 0x0cU) = 0U;
    BR_PWM(5U, 0x0cU) = 0U;
}

void board_motor_init(void) {
    /* Drive low as SIO before changing PWM muxes; never expose a stale CC. */
    out_w(GPIO_OUT_CLR, 0xf00U);
    out_w(GPIO_OE_SET, 0xf00U);
    for (unsigned pin = 8U; pin <= 11U; ++pin) {
        clr_w(GPIO(pin), GPIO_OD);
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    }
    /* Release the block without resetting any kernel-owned PWM slice. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
    }
    for (unsigned slice = 4U; slice <= 5U; ++slice) {
        BR_PWM(slice, 0U) = 0U;
        BR_PWM(slice, 0x04U) = 200U; /* 12.5 in 8.4: 125MHz / 12.5 / 1000 = 10kHz. */
        BR_PWM(slice, 0x08U) = 0U;
        BR_PWM(slice, 0x0cU) = 0U;
        BR_PWM(slice, 0x10U) = 999U;
    }
    for (unsigned pin = 8U; pin <= 11U; ++pin) {
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PWM);
    }
    BR_PWM(4U, 0U) = 1U;
    BR_PWM(5U, 0U) = 1U;
}

void board_motor_write(unsigned wheel, unsigned pwm) {
    if (wheel > 1U)
        return;
    if (pwm > 420U)
        pwm = 420U;
    BR_PWM(4U + wheel, 0x0cU) = wheel == 0U ? pwm : pwm << 16U;
}

static void gpio_dispatch(UINT vector) {
    (void)vector;
    uint32_t first = BR_REG(BR_IO_INTS(0U));
    uint32_t buttons = BR_REG(BR_IO_INTS(2U));
    BR_REG(BR_IO_INTR(0U)) = first & 0x00ff00ffU;
    BR_REG(BR_IO_INTR(2U)) = buttons & 0x00ff0000U;
    if ((first & 0xffU) != 0U)
        bench_encoder_update(&encoders[0], read_ab(0U));
    if ((first & 0x00ff0000U) != 0U)
        bench_encoder_update(&encoders[1], read_ab(1U));
    if ((buttons & (4U << 16U)) != 0U || board_button_pressed(20U)) {
        /* IRQ never calls the task-only motion API. Its latch is checked
         * inside every generation-checked commit before PWM is written. */
        bringup_irq_stop = 1U;
        board_motor_off();
    }
}

bool board_gpio_init(void) {
    const unsigned pins[] = {0U, 1U, 4U, 5U, 20U, 21U};
    for (unsigned i = 0U; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        unsigned pin = pins[i];
        out_w(GPIO_OE_CLR, 1U << pin);
        clr_w(GPIO(pin), GPIO_OD | GPIO_PDE);
        set_w(GPIO(pin), GPIO_IE | GPIO_PUE);
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    }
    for (unsigned w = 0U; w < 2U; ++w)
        encoders[w].ab = read_ab(w);
    T_DINT definition = {0};
    definition.intatr = TA_HLNG;
    definition.inthdr = gpio_dispatch;
    if (tk_def_int(BR_IO_IRQ_BANK0, &definition) != E_OK) {
        bringup_irq_stop = 1U;
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

void board_encoder_drain(int32_t delta[2], uint32_t edges[2], uint32_t invalid[2]) {
    UINT key;
    DI(key); /* Only the owning core's GPIO IRQ changes encoder state. */
    for (unsigned w = 0U; w < 2U; ++w) {
        delta[w] = encoders[w].position;
        edges[w] = encoders[w].a_edges + encoders[w].b_edges;
        invalid[w] = encoders[w].invalid;
        encoders[w].position = 0;
    }
    EI(key);
}

bool board_clock_registers_ok(void) {
    return (BR_REG(0x4005802cU) & 0x7ffU) == 0x60cU && BR_REG(0x40008038U) == 4U &&
           BR_REG(0x40008034U) == 0x100U && BR_REG(0x40008044U) == 2U &&
           BR_REG(0x40008040U) == 0x100U;
}

void board_clock_report(void) {
    tm_printf(
        (UB *)"CLOCK_REG core=%u tick=0x%x ref_selected=0x%x ref_div=0x%x sys_selected=0x%x sys_div=0x%x systick_reload=%u pll_fb=%u pll_prim=0x%x\n",
        tk_get_prc() - 1U, BR_REG(0x4005802cU), BR_REG(0x40008038U), BR_REG(0x40008034U),
        BR_REG(0x40008044U), BR_REG(0x40008040U), BR_REG(0xe000e014U), BR_REG(0x40028008U),
        BR_REG(0x4002800cU));
}
