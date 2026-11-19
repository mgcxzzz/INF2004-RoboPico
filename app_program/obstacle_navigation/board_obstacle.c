/**
 * @file board_obstacle.c
 * @brief Buddy5 RP2040 port: HC-SR04 TRIG GP16 / ECHO GP17, servo GP12.
 * @details Register writes only; no Pico SDK runtime. GP17 edges are
 * dispatched by the existing core1 IO_IRQ_BANK0 handler in board_port.c; this
 * file never installs an interrupt handler. Pad/SIO register names are the
 * micro T-Kernel RP2040 sysdef.h names also used by board_port.c. Pins and the
 * servo law are the unchanged SDK values (b5_config.h). Owner: core1 (Buddy5
 * task and the GPIO IRQ); the motor PWM slices 4/5 are never touched.
 * @see app_program/obstacle_navigation/README.md
 */
#include "obstacle_service.h"

#define RB5_REG(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define RB5_PWM6(offset) RB5_REG(0x40050000U + (0x14U * 6U) + (offset))
#define RB5_IO_INTR2 (0x400140f8U)

uint32_t board_obstacle_init (void)
{
    const unsigned trig = B5_PIN_US_TRIG;
    const unsigned echo = B5_PIN_US_ECHO;
    const unsigned servo = B5_PIN_SERVO_PWM;
    /* TRIG: SIO output, driven low before the pad output is enabled. */
    out_w(GPIO_OUT_CLR, 1U << trig);
    out_w(GPIO_OE_SET, 1U << trig);
    clr_w(GPIO(trig), GPIO_OD);
    out_w(GPIO_CTRL(trig), GPIO_CTRL_FUNCSEL_SIO);
    /* ECHO: SIO input with pull-down (SDK gpio_pull_down), never driven. */
    out_w(GPIO_OE_CLR, 1U << echo);
    clr_w(GPIO(echo), GPIO_OD | GPIO_PUE);
    set_w(GPIO(echo), GPIO_IE | GPIO_PDE);
    out_w(GPIO_CTRL(echo), GPIO_CTRL_FUNCSEL_SIO);
    /* Servo: slice 6, 125 MHz / 125 = 1 MHz, wrap 19999 -> 50 Hz, centre
     * 1450 us. board_motor_init() has already released the PWM block from
     * reset without touching other slices; motors use slices 4/5 only. */
    RB5_PWM6(0x00U) = 0U;
    RB5_PWM6(0x04U) = 125U << 4U;
    RB5_PWM6(0x08U) = 0U;
    RB5_PWM6(0x10U) = 19999U;
    RB5_PWM6(0x0cU) = (RB5_PWM6(0x0cU) & 0xffff0000U) | rb5_servo_pulse_us(0);
    clr_w(GPIO(servo), GPIO_OD);
    out_w(GPIO_CTRL(servo), GPIO_CTRL_FUNCSEL_PWM);
    RB5_PWM6(0x00U) = 1U;
    RB5_REG(RB5_IO_INTR2) = RB5_ECHO_EDGES; /* discard stale latched edges */
    return RB5_ECHO_EDGES;
}

bool board_obstacle_echo_high (void)
{
    return (in_w(GPIO_IN) & (1U << B5_PIN_US_ECHO)) != 0U;
}

void board_obstacle_echo_ack (void)
{
    RB5_REG(RB5_IO_INTR2) = RB5_ECHO_EDGES;
}

void board_obstacle_servo_us (unsigned pulse_us)
{
    if ((pulse_us < B5_SERVO_MIN_US) || (pulse_us > B5_SERVO_MAX_US))
    {
        return;
    }
    RB5_PWM6(0x0cU) = (RB5_PWM6(0x0cU) & 0xffff0000U) | pulse_us;
}

/* 12 us TRIG (datasheet minimum 10 us). The caller disables dispatch, so no
 * task can stretch it; interrupts (encoders, GP20, ECHO) stay enabled, as in
 * the SDK. Returns the realised width in us (diagnostic only). */
uint32_t board_obstacle_trigger (void)
{
    out_w(GPIO_OUT_SET, 1U << B5_PIN_US_TRIG);
    uint64_t start = board_time_us();
    while ((board_time_us() - start) < (uint64_t)B5_TRIG_PULSE_US)
    {
    }
    out_w(GPIO_OUT_CLR, 1U << B5_PIN_US_TRIG);
    return (uint32_t)(board_time_us() - start);
}

/* End of file. */
