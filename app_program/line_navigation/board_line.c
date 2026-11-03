#include "line_service.h"

#define RL_REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define RL_RAM __attribute__((section(".rtos_ramfunc"), noinline))
static volatile uint32_t flash_request, flash_ack;
typedef void (*FlashVoid)(void);
typedef void (*FlashErase)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*FlashProgram)(uint32_t, const uint8_t *, size_t);
typedef void *(*RomLookup)(uint16_t *, uint32_t);
static struct {
    FlashVoid connect, exit_xip, flush, boot;
    FlashErase erase;
    FlashProgram program;
    uint32_t boot2[64];
    uint8_t page[256];
} flash_ram;
extern uint32_t __boot2_start__;
static RlAdcDiagnostics adc_diagnostics;

/* ADC owner calls these only on core1; publication to core0 uses the service
 * snapshot lock. First failure is retained even after successful recovery. */
void board_line_adc_diagnostics(RlAdcDiagnostics *out) {
    *out = adc_diagnostics;
}

static void adc_failure(RlAdcReason reason, unsigned channel, RlAdcStage stage, unsigned sample,
                        uint32_t cs, uint32_t elapsed) {
    RlAdcFailure f = {.cs = cs,
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
    if (adc_diagnostics.first.reason == RL_ADC_NONE)
        adc_diagnostics.first = f;
    adc_diagnostics.last = f;
    if (reason == RL_ADC_TIMEOUT)
        ++adc_diagnostics.timeouts;
    else if (reason == RL_ADC_CONVERSION_ERR)
        ++adc_diagnostics.errors;
    else if (reason == RL_ADC_ARGUMENT)
        ++adc_diagnostics.bad_arguments;
    else
        ++adc_diagnostics.init_errors;
}

static uint16_t rom_halfword(uintptr_t address) {
    uint32_t value;
    __asm volatile("ldrh %0, [%1]" : "=r"(value) : "r"(address) : "memory");
    return (uint16_t)value;
}

void board_motor_levels(unsigned wheel, uint16_t a, uint16_t b) {
    if (wheel > 1 || (a && b) || a > M_PEAK || b > M_PEAK) {
        board_motor_off();
        return;
    }
    RL_REG(0x4005000cU + 0x14U * (4U + wheel)) = a | ((uint32_t)b << 16);
}

DualLevels board_motor_current(unsigned wheel) {
    if (wheel > 1)
        return (DualLevels){0};
    uint32_t cc = RL_REG(0x4005000cU + 0x14U * (4U + wheel));
    return (DualLevels){.a = (uint16_t)cc, .b = (uint16_t)(cc >> 16)};
}

bool board_line_adc_init(void) {
    RL_REG(0x4000e000U) = 1U; /* Reset only the ADC; no motor/kernel peripheral reset. */
    RL_REG(0x4000f000U) = 1U; /* RESETS atomic clear: ADC reset bit 0. */
    uint64_t start = board_time_us();
    while ((RL_REG(0x4000c008U) & 1U) == 0U)
        if (board_time_us() - start > 1000 && !(RL_REG(0x4000c008U) & 1U)) {
            adc_failure(RL_ADC_RESET_TIMEOUT, 7, RL_ADC_INIT, 0, RL_REG(0x4004c000U),
                        (uint32_t)(board_time_us() - start));
            return false;
        }
    for (unsigned pin = 26; pin <= 28; ++pin) {
        RL_REG(0x4001c004U + pin * 4U) = 1U << 7; /* OD; no digital input/pulls. */
        RL_REG(0x40014004U + pin * 8U) = 31U;
    }
    RL_REG(0x4004c010U) = 0U; /* full-speed divider, no FIFO/DREQ */
    RL_REG(0x4004c008U) = 0U;
    RL_REG(0x4004c000U) = 1U;
    start = board_time_us();
    while (!(RL_REG(0x4004c000U) & RL_ADC_READY)) {
        if (board_time_us() - start > 1000 && !(RL_REG(0x4004c000U) & RL_ADC_READY)) {
            adc_failure(RL_ADC_INIT_TIMEOUT, 7, RL_ADC_INIT, 0, RL_REG(0x4004c000U),
                        (uint32_t)(board_time_us() - start));
            return false;
        }
    }
    return true;
}

static bool adc_once(unsigned channel, RlAdcStage stage, unsigned index, uint16_t *value) {
    const RlAdcPort port = {.cs = (volatile uint32_t *)0x4004c000U,
                            .result = (volatile uint32_t *)0x4004c004U,
                            .now = board_time_us};
    uint32_t elapsed = 0, cs = 0;
    RlAdcReason reason =
        rl_adc_conversion(&port, value, &elapsed, &cs, &adc_diagnostics.avoided_timeout);
    if (reason != RL_ADC_NONE) {
        adc_failure(reason, channel, stage, index, cs, elapsed);
        return false;
    }
    ++adc_diagnostics.conversions;
    return true;
}

bool board_line_adc_read(unsigned channel, unsigned count, bool spaced, uint16_t *value) {
    if (channel > 2 || !count || count > 32 || !value) {
        adc_failure(RL_ADC_ARGUMENT, channel, RL_ADC_ARGUMENT_STAGE, 0, RL_REG(0x4004c000U), 0);
        return false;
    }
    RL_REG(0x4004c000U) = 1U | (channel << 12);
    uint16_t sample;
    if (!adc_once(channel, RL_ADC_DISCARD, 0, &sample)) {
        ++adc_diagnostics.read_fail[channel];
        return false; /* discard the first conversion after changing input */
    }
    uint32_t sum = 0;
    for (unsigned n = 0; n < count; ++n) {
        if (!adc_once(channel, RL_ADC_SAMPLE, n + 1, &sample)) {
            ++adc_diagnostics.read_fail[channel];
            return false;
        }
        sum += sample;
        if (spaced) {
            uint64_t until = board_time_us() + 20;
            while (board_time_us() < until) {
            }
        }
    }
    *value = (uint16_t)((sum + count / 2) / count);
    ++adc_diagnostics.read_ok[channel];
    return true;
}

/* No flash fetch, kernel call, literal in ROM or library operation may occur
 * between ACK and release. Both functions are linked into copied .data. */
static RL_RAM void rl_flash_park(void) {
    uint32_t irq;
    __asm volatile("mrs %0, primask\ncpsid i" : "=r"(irq)::"memory");
    flash_ack = 1;
    __asm volatile("dmb" ::: "memory");
    while (flash_request) {
        __asm volatile("nop" ::: "memory");
    }
    __asm volatile("dmb" ::: "memory");
    flash_ack = 0;
    __asm volatile("msr primask, %0" ::"r"(irq) : "memory");
}

static RL_RAM void rl_flash_write(void) {
    uint32_t irq;
    __asm volatile("mrs %0, primask\ncpsid i" : "=r"(irq)::"memory");
    __asm volatile("dmb" ::: "memory");
    flash_ram.connect();
    flash_ram.exit_xip();
    flash_ram.erase(RL_CAL_OFFSET, 4096, 4096, 0x20);
    flash_ram.program(RL_CAL_OFFSET, flash_ram.page, 256);
    flash_ram.flush();
    flash_ram.boot(); /* SDK boot2 copy restores the original QSPI mode. */
    flash_request = 0;
    __asm volatile("dmb" ::: "memory");
    __asm volatile("msr primask, %0" ::"r"(irq) : "memory");
}

void board_line_flash_worker(void) {
    if (flash_request)
        rl_flash_park();
}

bool board_line_save(const uint8_t page[256]) {
    BringupStatus s;
    bringup_get_status(&s);
    if ((s.flags & BR_ACTIVE) || board_button_pressed(20) || board_button_pressed(21) ||
        bringup_irq_stop || flash_ack)
        return false;
    board_motor_off();
    RomLookup lookup = (RomLookup)(uintptr_t)rom_halfword(0x18U);
    uint16_t *table = (uint16_t *)(uintptr_t)rom_halfword(0x14U);
    flash_ram.connect = (FlashVoid)lookup(table, 'I' | ('F' << 8));
    flash_ram.exit_xip = (FlashVoid)lookup(table, 'E' | ('X' << 8));
    flash_ram.erase = (FlashErase)lookup(table, 'R' | ('E' << 8));
    flash_ram.program = (FlashProgram)lookup(table, 'R' | ('P' << 8));
    flash_ram.flush = (FlashVoid)lookup(table, 'F' | ('C' << 8));
    if (!flash_ram.connect || !flash_ram.exit_xip || !flash_ram.erase || !flash_ram.program ||
        !flash_ram.flush)
        return false;
    memcpy(flash_ram.boot2, &__boot2_start__, sizeof(flash_ram.boot2));
    memcpy(flash_ram.page, page, sizeof(flash_ram.page));
    flash_ram.boot = (FlashVoid)((uintptr_t)flash_ram.boot2 | 1U);
    __asm volatile("dmb" ::: "memory");
    flash_request = 1;
    uint64_t start = board_time_us();
    /* Do not sleep: the parked core stops its kernel tick during lockout. */
    while (!flash_ack) {
        if (board_time_us() - start >= 20000) {
            flash_request = 0;
            __asm volatile("dmb" ::: "memory");
            return false;
        }
    }
    rl_flash_write();
    /* Read-back is after XIP has been restored. */
    return memcmp((const void *)(uintptr_t)(0x10000000U + RL_CAL_OFFSET), page, 256) == 0;
}
