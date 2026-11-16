/**
 * @file board_line.c
 * @brief Board line.
 * @details ADC/control ownership covers sensor acquisition, planning and
 * calibration. Other tasks consume protected copies; flash and stored exports
 * require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "line_service.h"

#define RL_REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define RL_RAM __attribute__((section(".rtos_ramfunc"), noinline))
static volatile uint32_t g_flash_request;
static volatile uint32_t g_flash_ack;
typedef void (*FlashVoid)(void);
typedef void (*FlashErase)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*FlashProgram)(uint32_t, const uint8_t *, size_t);
typedef void * (*RomLookup)(uint16_t *, uint32_t);
static struct
{
    FlashVoid connect;
    FlashVoid exit_xip;
    FlashVoid flush;
    FlashVoid boot;
    FlashErase erase;
    FlashProgram program;
    uint32_t boot2[64];
    uint8_t page[256];
} g_flash_ram;
extern uint32_t __boot2_start__;
static rl_adc_diagnostics_t g_adc_diagnostics;

/* ADC owner calls these only on core1; publication to core0 uses the service
 * snapshot lock. First failure is retained even after successful recovery. */
void board_line_adc_diagnostics (rl_adc_diagnostics_t * p_out_value)
{
    *p_out_value = g_adc_diagnostics;
}

static void adc_failure (rl_adc_reason_t reason, unsigned channel,
                         rl_adc_stage_t stage, unsigned sample,
                         uint32_t cs_value, uint32_t elapsed)
{
    rl_adc_failure_t flags = {.cs = cs_value,
                              .elapsed_us = elapsed,
                              .timer_us = (uint32_t)board_time_us(),
                              .clk_ctrl = RL_REG(0x40008060U),
                              .clk_div = RL_REG(0x40008064U),
                              .adc_div = RL_REG(0x4004c010U),
                              .resets_done = RL_REG(0x4000c008U),
                              .reason = reason,
                              .channel = channel > 2 ? 7 : channel,
                              .stage = stage,
                              .sample = sample};
    if (RL_ADC_NONE == g_adc_diagnostics.first.reason)
    {
        g_adc_diagnostics.first = flags;
    }
    g_adc_diagnostics.last = flags;
    if (RL_ADC_TIMEOUT == reason)
    {
        ++g_adc_diagnostics.timeouts;
    }
    else if (RL_ADC_CONVERSION_ERR == reason)
    {
        ++g_adc_diagnostics.errors;
    }
    else if (RL_ADC_ARGUMENT == reason)
    {
        ++g_adc_diagnostics.bad_arguments;
    }
    else
    {
        ++g_adc_diagnostics.init_errors;
    }
}

static uint16_t rom_halfword (uintptr_t address)
{
    uint32_t value;
    __asm volatile("ldrh %0, [%1]" : "=r"(value) : "r"(address) : "memory");
    return (uint16_t)value;
}

void board_motor_levels (unsigned wheel, uint16_t first_value,
                         uint16_t second_value)
{
    if ((((wheel > 1) || (first_value && second_value)) ||
         (first_value > M_PEAK)) ||
        (second_value > M_PEAK))
    {
        board_motor_off();
        return;
    }
    RL_REG(0x4005000cU + 0x14U * (4U + wheel)) =
        first_value | ((uint32_t)second_value << 16);
}

dual_levels_t board_motor_current (unsigned wheel)
{
    if (wheel > 1)
    {
        return (dual_levels_t){0};
    }
    uint32_t cc_value = RL_REG(0x4005000cU + 0x14U * (4U + wheel));
    return (dual_levels_t){.a = (uint16_t)cc_value,
                           .b = (uint16_t)(cc_value >> 16)};
}

bool board_line_adc_init (void)
{
    RL_REG(0x4000e000U) =
        1U; /* Reset only the ADC; no motor/kernel peripheral reset. */
    RL_REG(0x4000f000U) = 1U; /* RESETS atomic clear: ADC reset bit 0. */
    uint64_t start = board_time_us();
    while (0U == (RL_REG(0x4000c008U) & 1U))
    {
        if ((board_time_us() - start > 1000) && (!(RL_REG(0x4000c008U) & 1U)))
        {
            adc_failure(RL_ADC_RESET_TIMEOUT, 7, RL_ADC_INIT, 0,
                        RL_REG(0x4004c000U),
                        (uint32_t)(board_time_us() - start));
            return false;
        }
    }
    for (unsigned pin = 26; pin <= 28; ++pin)
    {
        RL_REG(0x4001c004U + pin * 4U) = 1U
                                         << 7; /* OD; no digital input/pulls. */
        RL_REG(0x40014004U + pin * 8U) = 31U;
    }
    RL_REG(0x4004c010U) = 0U; /* full-speed divider, no FIFO/DREQ */
    RL_REG(0x4004c008U) = 0U;
    RL_REG(0x4004c000U) = 1U;
    start = board_time_us();
    while (!(RL_REG(0x4004c000U) & RL_ADC_READY))
    {
        if ((board_time_us() - start > 1000) &&
            (!(RL_REG(0x4004c000U) & RL_ADC_READY)))
        {
            adc_failure(RL_ADC_INIT_TIMEOUT, 7, RL_ADC_INIT, 0,
                        RL_REG(0x4004c000U),
                        (uint32_t)(board_time_us() - start));
            return false;
        }
    }
    return true;
}

static bool adc_once (unsigned channel, rl_adc_stage_t stage, unsigned index,
                      uint16_t * p_value)
{
    const rl_adc_port_t port = {.cs = (volatile uint32_t *)0x4004c000U,
                                .result = (volatile uint32_t *)0x4004c004U,
                                .now = board_time_us};
    uint32_t elapsed = 0, cs_value = 0;
    rl_adc_reason_t reason =
        rl_adc_conversion(&port, p_value, &elapsed, &cs_value,
                          &g_adc_diagnostics.avoided_timeout);
    if (reason != RL_ADC_NONE)
    {
        adc_failure(reason, channel, stage, index, cs_value, elapsed);
        return false;
    }
    ++g_adc_diagnostics.conversions;
    return true;
}

bool board_line_adc_read (unsigned channel, unsigned count, bool spaced,
                          uint16_t * p_value_value)
{
    if ((((channel > 2) || (!count)) || (count > 32)) || (!p_value_value))
    {
        adc_failure(RL_ADC_ARGUMENT, channel, RL_ADC_ARGUMENT_STAGE, 0,
                    RL_REG(0x4004c000U), 0);
        return false;
    }
    RL_REG(0x4004c000U) = 1U | (channel << 12);
    uint16_t sample;
    if (!adc_once(channel, RL_ADC_DISCARD, 0, &sample))
    {
        ++g_adc_diagnostics.read_fail[channel];
        return false; /* discard the first conversion after changing input */
    }
    uint32_t sum = 0;
    for (unsigned count_value = 0; count_value < count; ++count_value)
    {
        if (!adc_once(channel, RL_ADC_SAMPLE, count_value + 1, &sample))
        {
            ++g_adc_diagnostics.read_fail[channel];
            return false;
        }
        sum += sample;
        if (spaced)
        {
            uint64_t until = board_time_us() + 20;
            while (board_time_us() < until)
            {
            }
        }
    }
    *p_value_value = (uint16_t)((sum + count / 2) / count);
    ++g_adc_diagnostics.read_ok[channel];
    return true;
}

/* No flash fetch, kernel call, literal in ROM or library operation may occur
 * between ACK and release. Both functions are linked into copied .data. */
static RL_RAM void rl_flash_park (void)
{
    uint32_t irq;
    __asm volatile("mrs %0, primask\ncpsid i" : "=r"(irq)::"memory");
    g_flash_ack = 1;
    __asm volatile("dmb" ::: "memory");
    while (g_flash_request)
    {
        __asm volatile("nop" ::: "memory");
    }
    __asm volatile("dmb" ::: "memory");
    g_flash_ack = 0;
    __asm volatile("msr primask, %0" ::"r"(irq) : "memory");
}

static RL_RAM void rl_flash_write (void)
{
    uint32_t irq;
    __asm volatile("mrs %0, primask\ncpsid i" : "=r"(irq)::"memory");
    __asm volatile("dmb" ::: "memory");
    g_flash_ram.connect();
    g_flash_ram.exit_xip();
    g_flash_ram.erase(RL_CAL_OFFSET, 4096, 4096, 0x20);
    g_flash_ram.program(RL_CAL_OFFSET, g_flash_ram.page, 256);
    g_flash_ram.flush();
    g_flash_ram.boot(); /* SDK boot2 copy restores the original QSPI mode. */
    g_flash_request = 0;
    __asm volatile("dmb" ::: "memory");
    __asm volatile("msr primask, %0" ::"r"(irq) : "memory");
}

void board_line_flash_worker (void)
{
    if (g_flash_request)
    {
        rl_flash_park();
    }
}

bool board_line_save (const uint8_t page[256])
{
    bringup_status_t state;
    bringup_get_status(&state);
    if (((((state.flags & BR_ACTIVE) || (board_button_pressed(20))) ||
          (board_button_pressed(21))) ||
         g_bringup_irq_stop) ||
        g_flash_ack)
    {
        return false;
    }
    board_motor_off();
    RomLookup p_lookup = (RomLookup)(uintptr_t)rom_halfword(0x18U);
    uint16_t * p_table = (uint16_t *)(uintptr_t)rom_halfword(0x14U);
    g_flash_ram.connect = (FlashVoid)p_lookup(p_table, 'I' | ('F' << 8));
    g_flash_ram.exit_xip = (FlashVoid)p_lookup(p_table, 'E' | ('X' << 8));
    g_flash_ram.erase = (FlashErase)p_lookup(p_table, 'R' | ('E' << 8));
    g_flash_ram.program = (FlashProgram)p_lookup(p_table, 'R' | ('P' << 8));
    g_flash_ram.flush = (FlashVoid)p_lookup(p_table, 'F' | ('C' << 8));
    if (((((!g_flash_ram.connect) || (!g_flash_ram.exit_xip)) ||
          (!g_flash_ram.erase)) ||
         (!g_flash_ram.program)) ||
        (!g_flash_ram.flush))
    {
        return false;
    }
    memcpy(g_flash_ram.boot2, &__boot2_start__, sizeof(g_flash_ram.boot2));
    memcpy(g_flash_ram.page, page, sizeof(g_flash_ram.page));
    g_flash_ram.boot = (FlashVoid)((uintptr_t)g_flash_ram.boot2 | 1U);
    __asm volatile("dmb" ::: "memory");
    g_flash_request = 1;
    uint64_t start = board_time_us();
    /* Do not sleep: the parked core stops its kernel tick during lockout. */
    while (!g_flash_ack)
    {
        if (board_time_us() - start >= 20000)
        {
            g_flash_request = 0;
            __asm volatile("dmb" ::: "memory");
            return false;
        }
    }
    rl_flash_write();
    /* Read-back is after XIP has been restored. */
    return 0 == memcmp((const void *)(uintptr_t)(0x10000000U + RL_CAL_OFFSET),
                       page, 256);
}

/* End of file. */
