/* Pico API compatibility layer for the Continuous V2 controller under UTK.
 * This file uses direct RP2040 MMIO for GPIO/PWM/ADC and kernel services only
 * for task delays, IRQ registration, and the shared USB CDC console.  It does
 * not initialize the SDK runtime or start a second USB/network service. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "utk_hal.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"

typedef unsigned int uint;

extern uint64_t time_us_64(void);
extern void tm_snd_dat(const unsigned char *data, int32_t length);
extern uint32_t tm_usb_pending(void);

#define BOOTROM_TABLE_LOOKUP_OFFSET 0x18u
#define BOOTROM_FUNC_TABLE_OFFSET 0x14u
#define ROM_TABLE_CODE(c1, c2) ((c1) | ((c2) << 8))
#define ROM_FUNC_FLASH_EXIT_XIP ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_FLUSH_CACHE ROM_TABLE_CODE('F', 'C')
#define ROM_FUNC_CONNECT_INTERNAL_FLASH ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_RANGE_ERASE ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM ROM_TABLE_CODE('R', 'P')
#define TIMER_BASE              0x40054000u
#define TIMER_ALARM3            (TIMER_BASE + 0x1cu)
#define TIMER_TIMERAWL          (TIMER_BASE + 0x28u)
#define TIMER_INTR              (TIMER_BASE + 0x34u)
#define TIMER_INTE              (TIMER_BASE + 0x38u)
#define TIMER_IRQ3              3u
#define TIMER_IRQ3_MASK         (1u << 3)
#define NVIC_IPR_BASE           0xe000e400u

#define SIO_BASE                0xd0000000u
#define SIO_GPIO_IN             (SIO_BASE + 0x04u)
#define SIO_GPIO_OUT_SET        (SIO_BASE + 0x14u)
#define SIO_GPIO_OUT_CLR        (SIO_BASE + 0x18u)
#define SIO_GPIO_OE_SET         (SIO_BASE + 0x24u)
#define SIO_GPIO_OE_CLR         (SIO_BASE + 0x28u)
#define IO_BANK0_BASE           0x40014000u
#define PADS_BANK0_BASE         0x4001c000u
#define GPIO_CTRL(gpio)         (IO_BANK0_BASE + 0x04u + (8u * (gpio)))
#define GPIO_PAD(gpio)          (PADS_BANK0_BASE + 0x04u + (4u * (gpio)))
#define RESETS_BASE             0x4000c000u
#define RESETS_RESET            (RESETS_BASE + 0x00u)
#define RESETS_RESET_DONE       (RESETS_BASE + 0x08u)
#define RESETS_RESET_CLR        (RESETS_RESET + 0x3000u)
#define RESETS_PWM              (1u << 14)
#define RESETS_PADS_BANK0       (1u << 8)
#define RESETS_IO_BANK0         (1u << 5)
#define RESETS_ADC              (1u << 0)
#define PADS_QSPI_BASE          0x40020000u
#define PADS_QSPI_COUNT         6u

#define ADC_BASE                0x4004c000u
#define ADC_CS                  (ADC_BASE + 0x00u)
#define ADC_RESULT              (ADC_BASE + 0x04u)
#define ADC_CS_EN               (1u << 0)
#define ADC_CS_START_ONCE       (1u << 2)
#define ADC_CS_READY            (1u << 8)
#define ADC_CS_AINSEL_LSB       12u
#define ADC_CS_AINSEL_MASK      (7u << ADC_CS_AINSEL_LSB)

#define PWM_BASE                0x40050000u
#define PWM_CSR_EN              (1u << 0)
#define PWM_GPIO_FUNC           4u
#define GPIO_FUNC_SIO           5u
#define GPIO_FUNC_NULL          31u
#define GPIO_IRQ_EDGE_FALL      0x04u
#define GPIO_IRQ_EDGE_RISE      0x08u
#define GPIO_IN                 0u
#define GPIO_OUT                1u
#define GPIO_PULL_UP_BIT        (1u << 3)
#define GPIO_PULL_DOWN_BIT      (1u << 2)
#define PICO_ERROR_TIMEOUT      (-1)
#define FLASH_SECTOR_SIZE       4096u
#define FLASH_PAGE_SIZE         256u
#define FLASH_BLOCK_SIZE        65536u
#define FLASH_BLOCK_ERASE_CMD   0xd8u
#define FLASH_WAIT_TIMEOUT_US   500000u
#define BOOT2_WORDS             64u
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES   (2u * 1024u * 1024u)
#endif
#define RAM_CODE                __attribute__((noinline, section(".time_critical.utk_flash_code")))
#define RAM_DATA                __attribute__((section(".data.utk_flash_data")))

#define MMIO32(address) (*(volatile uint32_t *)(uintptr_t)(address))

typedef void *(*utk_rom_table_lookup_fn)(uint16_t *table, uint32_t code);
typedef void (*rom_connect_internal_flash_fn)(void);
typedef void (*rom_flash_exit_xip_fn)(void);
typedef void (*rom_flash_flush_cache_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_flash_range_program_fn)(uint32_t, const uint8_t *, size_t);

static inline __attribute__((always_inline)) void *rom_func_lookup_inline(uint32_t code)
{
    uint16_t lookup_address;
    uint16_t table_address;
    __asm__ volatile ("ldrh %0, [%1]" : "=r" (lookup_address)
                      : "r" ((uintptr_t)BOOTROM_TABLE_LOOKUP_OFFSET));
    __asm__ volatile ("ldrh %0, [%1]" : "=r" (table_address)
                      : "r" ((uintptr_t)BOOTROM_FUNC_TABLE_OFFSET));
    utk_rom_table_lookup_fn lookup =
        (utk_rom_table_lookup_fn)(uintptr_t)lookup_address;
    uint16_t *table = (uint16_t *)(uintptr_t)table_address;
    return lookup(table, code);
}

typedef struct {
    volatile uint32_t csr;
    volatile uint32_t div;
    volatile uint32_t ctr;
    volatile uint32_t cc;
    volatile uint32_t top;
} utk_pwm_slice_hw_t;

typedef struct {
    utk_pwm_slice_hw_t slice[8];
} utk_pwm_hw_t;

typedef struct {
    uint32_t csr;
    uint32_t div;
    uint32_t top;
} pwm_config;

typedef struct repeating_timer repeating_timer;
struct repeating_timer {
    int64_t delay_us;
    bool (*callback)(repeating_timer *timer);
    void *user_data;
    bool active;
};

typedef bool (*utk_repeating_timer_callback_t)(repeating_timer *timer);

static utk_repeating_timer_callback_t g_safety_callback;
static repeating_timer *g_safety_timer;
static void (*g_gpio_callback)(uint gpio, uint32_t events);
static volatile uint32_t g_flash_request;
static volatile uint32_t g_flash_parked;
static volatile uint32_t g_flash_release;
static volatile uint32_t g_flash_error;
static uint32_t g_boot2_copyout[BOOT2_WORDS] RAM_DATA;
static bool g_boot2_copyout_valid RAM_DATA;
static uint32_t g_qspi_pads[PADS_QSPI_COUNT] RAM_DATA;
static bool g_flash_session_active;

void utk_kernel_clear_int(uint32_t intno);
int32_t utk_kernel_define_int(uint32_t intno, void (*handler)(unsigned int));
void utk_kernel_enable_int(uint32_t intno, int32_t level);
int32_t utk_kernel_gpio_attach(uint32_t gpio, uint32_t edges,
                               void (*callback)(uint32_t, uint32_t, uint32_t),
                               int32_t level);
void utk_kernel_unreset(uint32_t mask);
int32_t utk_kernel_delay(uint32_t delay_ms);

void utk_gpio_set_dir(uint gpio, bool output);
void utk_gpio_set_function(uint gpio, uint function);
void utk_gpio_pull_up(uint gpio);
void utk_gpio_put(uint gpio, bool value);
void utk_gpio_set_irq_enabled(uint gpio, uint32_t events, bool enabled);
uint utk_pwm_gpio_to_slice_num(uint gpio);
void utk_pwm_set_enabled(uint slice, bool enabled);

static inline __attribute__((always_inline)) uint32_t irq_save_disable(void)
{
    uint32_t state;
    __asm__ volatile ("mrs %0, primask" : "=r" (state));
    __asm__ volatile ("cpsid i" ::: "memory");
    return state;
}

static inline __attribute__((always_inline)) void irq_restore(uint32_t state)
{
    __asm__ volatile ("dmb" ::: "memory");
    __asm__ volatile ("msr primask, %0" :: "r" (state) : "memory");
}

static void gpio_irq_forward(uint32_t gpio, uint32_t edges, uint32_t timestamp_us)
{
    (void)timestamp_us;
    if (g_gpio_callback != NULL) {
        g_gpio_callback((uint)gpio, edges);
    }
}

static void safety_alarm3_handler(unsigned int intno)
{
    bool keep_running = false;
    uint32_t now;
    uint32_t interval;

    MMIO32(TIMER_INTR) = TIMER_IRQ3_MASK;
    if (g_safety_timer == NULL || !g_safety_timer->active ||
        g_safety_callback == NULL) {
        MMIO32(TIMER_INTE) &= ~TIMER_IRQ3_MASK;
        utk_kernel_clear_int(intno);
        return;
    }

    keep_running = g_safety_callback(g_safety_timer);
    if (keep_running) {
        interval = (uint32_t)((g_safety_timer->delay_us < 0)
                               ? -g_safety_timer->delay_us
                               : g_safety_timer->delay_us);
        if (interval == 0u) interval = 1u;
        now = MMIO32(TIMER_TIMERAWL);
        MMIO32(TIMER_ALARM3) = now + interval;
    } else {
        g_safety_timer->active = false;
        MMIO32(TIMER_INTE) &= ~TIMER_IRQ3_MASK;
    }
    utk_kernel_clear_int(intno);
}

static spin_lock_t g_spin_locks[32];

static void spinlock_fail_stop(void)
{
    /* A required synchronization primitive must never fail open around
       motor commands.  Force the four drive outputs inactive before halting. */
    for (uint gpio = 8u; gpio <= 11u; gpio++) {
        uint slice = utk_pwm_gpio_to_slice_num(gpio);
        utk_pwm_set_enabled(slice, false);
        utk_gpio_set_function(gpio, GPIO_FUNC_SIO);
        utk_gpio_set_dir(gpio, true);
        utk_gpio_put(gpio, false);
    }
    for (;;) __asm__ volatile ("wfi");
}

spin_lock_t *utk_spin_lock_instance(uint lock_num)
{
    if (lock_num >= 32u) return NULL;
    return &g_spin_locks[lock_num];
}

uint32_t utk_spin_lock_claim_unused(bool required)
{
    int32_t number = utk_kernel_spinlock_claim_unused();
    if (number >= 0) {
        g_spin_locks[(uint32_t)number].number = (uint32_t)number;
        return (uint32_t)number;
    }
    if (required) {
        spinlock_fail_stop();
    }
    return UINT32_MAX;
}

uint32_t utk_spin_lock_blocking(spin_lock_t *lock)
{
    uint32_t state = 0u;
    if (lock == NULL ||
        utk_kernel_spinlock_lock(lock->number, &state) != 0) spinlock_fail_stop();
    return state;
}

void utk_spin_unlock(spin_lock_t *lock, uint32_t irq_state)
{
    if (lock == NULL || utk_kernel_spinlock_unlock(lock->number, irq_state) != 0)
        spinlock_fail_stop();
}

uint32_t utk_save_and_disable_interrupts(void)
{
    return irq_save_disable();
}

void utk_restore_interrupts(uint32_t state)
{
    irq_restore(state);
}

void utk_gpio_init(uint gpio)
{
    if (gpio >= 30u) return;
    utk_kernel_unreset(RESETS_PADS_BANK0 | RESETS_IO_BANK0);
    utk_gpio_set_function(gpio, GPIO_FUNC_SIO);
    utk_gpio_set_dir(gpio, GPIO_IN);
    utk_gpio_pull_up(gpio);
}

void utk_gpio_put(uint gpio, bool value)
{
    if (gpio >= 30u) return;
    MMIO32(value ? SIO_GPIO_OUT_SET : SIO_GPIO_OUT_CLR) = 1u << gpio;
}

void utk_gpio_set_dir(uint gpio, bool output)
{
    if (gpio >= 30u) return;
    MMIO32(output ? SIO_GPIO_OE_SET : SIO_GPIO_OE_CLR) = 1u << gpio;
}

void utk_gpio_set_function(uint gpio, uint function)
{
    uint32_t value;
    if (gpio >= 30u) return;
    value = MMIO32(GPIO_CTRL(gpio));
    MMIO32(GPIO_CTRL(gpio)) = (value & ~0x1fu) | (function & 0x1fu);
}

bool utk_gpio_get(uint gpio)
{
    return (gpio < 30u) && ((MMIO32(SIO_GPIO_IN) & (1u << gpio)) != 0u);
}

uint32_t utk_gpio_get_all(void)
{
    return MMIO32(SIO_GPIO_IN);
}

void utk_gpio_pull_up(uint gpio)
{
    uint32_t value;
    if (gpio >= 30u) return;
    value = MMIO32(GPIO_PAD(gpio));
    MMIO32(GPIO_PAD(gpio)) = (value | GPIO_PULL_UP_BIT) & ~GPIO_PULL_DOWN_BIT;
}

void utk_gpio_pull_down(uint gpio)
{
    uint32_t value;
    if (gpio >= 30u) return;
    value = MMIO32(GPIO_PAD(gpio));
    MMIO32(GPIO_PAD(gpio)) = (value | GPIO_PULL_DOWN_BIT) & ~GPIO_PULL_UP_BIT;
}

void utk_gpio_disable_pulls(uint gpio)
{
    if (gpio >= 30u) return;
    MMIO32(GPIO_PAD(gpio)) &= ~(GPIO_PULL_UP_BIT | GPIO_PULL_DOWN_BIT);
}

void utk_gpio_set_irq_enabled_with_callback(uint gpio, uint32_t events,
                                            bool enabled,
                                            void (*callback)(uint, uint32_t))
{
    g_gpio_callback = callback;
    utk_gpio_set_irq_enabled(gpio, events, enabled);
}

void utk_gpio_set_irq_enabled(uint gpio, uint32_t events, bool enabled)
{
    uint32_t edges = 0u;
    if ((events & GPIO_IRQ_EDGE_FALL) != 0u) edges |= GPIO_IRQ_EDGE_FALL;
    if ((events & GPIO_IRQ_EDGE_RISE) != 0u) edges |= GPIO_IRQ_EDGE_RISE;
    if (!enabled) return; /* This controller only enables encoder edges. */
    (void)utk_kernel_gpio_attach(gpio, edges, gpio_irq_forward, 0x40);
}

void utk_irq_set_priority(uint irq_num, uint8_t priority)
{
    if (irq_num < 32u) {
        *(volatile uint8_t *)(uintptr_t)(NVIC_IPR_BASE + irq_num) = priority;
    }
}

void utk_irq_set_enabled(uint irq_num, bool enabled)
{
    const uintptr_t iser = 0xe000e100u;
    const uintptr_t icer = 0xe000e180u;
    if (irq_num >= 32u) return;
    MMIO32(enabled ? iser : icer) = 1u << irq_num;
}

pwm_config utk_pwm_get_default_config(void)
{
    pwm_config config = {0u, 16u, 0xffffu};
    return config;
}

void utk_pwm_config_set_wrap(pwm_config *config, uint16_t wrap)
{
    if (config != NULL) config->top = wrap;
}

void utk_pwm_config_set_clkdiv(pwm_config *config, float divider)
{
    uint32_t fixed;
    if (config == NULL) return;
    if (divider < 1.0f) divider = 1.0f;
    if (divider > 255.9375f) divider = 255.9375f;
    fixed = (uint32_t)(divider * 16.0f + 0.5f);
    if (fixed < 16u) fixed = 16u;
    config->div = fixed;
}

uint utk_pwm_gpio_to_slice_num(uint gpio)
{
    return (gpio >> 1u) & 7u;
}

void utk_pwm_init(uint slice, const pwm_config *config, bool start)
{
    volatile utk_pwm_hw_t *pwm = (volatile utk_pwm_hw_t *)(uintptr_t)PWM_BASE;
    if (slice >= 8u || config == NULL) return;
    utk_kernel_unreset(RESETS_PWM);
    pwm->slice[slice].csr = 0u;
    pwm->slice[slice].div = config->div;
    pwm->slice[slice].ctr = 0u;
    pwm->slice[slice].cc = 0u;
    pwm->slice[slice].top = config->top;
    pwm->slice[slice].csr = config->csr | (start ? PWM_CSR_EN : 0u);
}

void utk_pwm_set_both_levels(uint slice, uint16_t level_a, uint16_t level_b)
{
    volatile utk_pwm_hw_t *pwm = (volatile utk_pwm_hw_t *)(uintptr_t)PWM_BASE;
    if (slice < 8u) {
        pwm->slice[slice].cc = (uint32_t)level_a | ((uint32_t)level_b << 16u);
    }
}

void utk_pwm_set_enabled(uint slice, bool enabled)
{
    volatile utk_pwm_hw_t *pwm = (volatile utk_pwm_hw_t *)(uintptr_t)PWM_BASE;
    if (slice >= 8u) return;
    if (enabled) pwm->slice[slice].csr |= PWM_CSR_EN;
    else pwm->slice[slice].csr &= ~PWM_CSR_EN;
}

volatile utk_pwm_hw_t *utk_pwm_hardware(void)
{
    return (volatile utk_pwm_hw_t *)(uintptr_t)PWM_BASE;
}

void utk_adc_init(void)
{
    utk_kernel_unreset(RESETS_ADC);
    MMIO32(ADC_CS) = ADC_CS_EN;
}

void utk_adc_gpio_init(uint gpio)
{
    if (gpio < 26u || gpio > 29u) return;
    utk_gpio_disable_pulls(gpio);
    utk_gpio_set_function(gpio, GPIO_FUNC_NULL);
}

void utk_adc_select_input(uint input)
{
    uint32_t cs;
    if (input > 4u) return;
    cs = MMIO32(ADC_CS);
    MMIO32(ADC_CS) = (cs & ~ADC_CS_AINSEL_MASK) |
                     (input << ADC_CS_AINSEL_LSB) | ADC_CS_EN;
}

uint16_t utk_adc_read(void)
{
    uint32_t cs = MMIO32(ADC_CS);
    MMIO32(ADC_CS) = cs | ADC_CS_EN | ADC_CS_START_ONCE;
    while ((MMIO32(ADC_CS) & ADC_CS_READY) == 0u) {
        __asm__ volatile ("nop");
    }
    return (uint16_t)(MMIO32(ADC_RESULT) & 0x0fffu);
}

uint32_t utk_clock_get_hz(uint clock)
{
    (void)clock;
    return 125000000u;
}

bool utk_add_repeating_timer_us(int64_t delay_us,
                                utk_repeating_timer_callback_t callback,
                                void *user_data,
                                repeating_timer *timer)
{
    uint32_t state;
    if (delay_us == 0 || callback == NULL || timer == NULL ||
        g_safety_timer != NULL) return false;

    timer->delay_us = delay_us;
    timer->callback = callback;
    timer->user_data = user_data;
    timer->active = true;
    g_safety_callback = callback;
    g_safety_timer = timer;

    if (utk_kernel_define_int(TIMER_IRQ3, safety_alarm3_handler) != 0) {
        timer->active = false;
        g_safety_timer = NULL;
        g_safety_callback = NULL;
        return false;
    }

    state = irq_save_disable();
    MMIO32(TIMER_INTR) = TIMER_IRQ3_MASK;
    MMIO32(TIMER_INTE) &= ~TIMER_IRQ3_MASK;
    MMIO32(TIMER_ALARM3) = MMIO32(TIMER_TIMERAWL) +
                          (uint32_t)((delay_us < 0) ? -delay_us : delay_us);
    utk_kernel_clear_int(TIMER_IRQ3);
    MMIO32(TIMER_INTE) |= TIMER_IRQ3_MASK;
    irq_restore(state);
    utk_kernel_enable_int(TIMER_IRQ3, 2);
    return true;
}

void utk_tight_loop_contents(void)
{
    __asm__ volatile ("nop");
}

void utk_sleep_us(uint64_t us)
{
    /* The baseline's only 100 us call is its cooperative main-loop yield.
       Let the 1 ms kernel tick schedule it; shorter ADC settling delays need
       a precise busy wait so the sampling cadence remains stable. */
    if (us >= 100u) {
        (void)utk_kernel_delay(1u);
    } else {
        uint64_t deadline = time_us_64() + us;
        while ((int64_t)(deadline - time_us_64()) > 0) {
            __asm__ volatile ("nop");
        }
    }
}

void utk_sleep_ms(uint32_t ms)
{
    if (ms != 0u) (void)utk_kernel_delay(ms);
}

void utk_stdio_init_all(void)
{
    /* The kernel owns USB CDC from startup.  Never initialize TinyUSB here. */
}

bool utk_stdio_usb_connected(void)
{
    return tm_usb_state() >= 3u;
}

bool utk_tud_mounted(void)
{
    return tm_usb_state() >= 2u;
}

int utk_getchar_timeout_us(uint32_t timeout_us)
{
    (void)timeout_us;
    int result = tm_usb_try_read();
    return (result >= 0 && result <= 255) ? result : PICO_ERROR_TIMEOUT;
}

void utk_stdio_flush(void)
{
    uint64_t deadline = time_us_64() + 50000u;
    while (tm_usb_pending() != 0u && (int64_t)(deadline - time_us_64()) > 0) {
        (void)utk_kernel_delay(1u);
    }
}

void utk_console_write(const char *data, unsigned int length)
{
    unsigned char chunk[64];
    unsigned int used = 0u;
    if (data == NULL) return;
    for (unsigned int i = 0u; i < length; i++) {
        if (data[i] == '\n') chunk[used++] = '\r';
        chunk[used++] = (unsigned char)data[i];
        if (used >= sizeof(chunk) - 1u) {
            tm_snd_dat(chunk, (int32_t)used);
            used = 0u;
        }
    }
    if (used != 0u) tm_snd_dat(chunk, (int32_t)used);
}

/* The core-0 USB task calls this outside TinyUSB and console locks.  The
 * request/park handshake leaves core 0 executing only this RAM-resident loop
 * while core 1 temporarily disables XIP for a flash operation. */
void RAM_CODE utk_flash_service(void)
{
    uint32_t irq_state;
    if (g_flash_request == 0u || g_flash_parked != 0u) return;
    __asm__ volatile ("dmb" ::: "memory");
    irq_state = irq_save_disable();
    g_flash_parked = 1u;
    __asm__ volatile ("dmb" ::: "memory");
    while (g_flash_release == 0u) {
        __asm__ volatile ("nop");
    }
    g_flash_request = 0u;
    g_flash_release = 0u;
    __asm__ volatile ("dmb" ::: "memory");
    /* Let the writer observe parked==0 only after every handshake field is
       back in its idle state, so a following flash_begin cannot overlap this
       task's cleanup tail. */
    g_flash_parked = 0u;
    __asm__ volatile ("dmb" ::: "memory");
    irq_restore(irq_state);
}

bool utk_flash_begin(void)
{
    uint64_t deadline;
    if (g_flash_session_active || g_flash_request != 0u) return false;
    g_flash_release = 0u;
    g_flash_parked = 0u;
    g_flash_error = 0u;
    __asm__ volatile ("dmb" ::: "memory");
    g_flash_request = 1u;
    __asm__ volatile ("dmb" ::: "memory");
    deadline = time_us_64() + FLASH_WAIT_TIMEOUT_US;
    while (g_flash_parked == 0u) {
        if ((int64_t)(deadline - time_us_64()) <= 0) {
            g_flash_release = 1u;
            __asm__ volatile ("dmb" ::: "memory");
            g_flash_request = 0u;
            __asm__ volatile ("dmb" ::: "memory");
            /* The USB task's RAM park loop sees release immediately; wait
               for that cleanup before allowing the caller to continue. */
            while (g_flash_parked != 0u) __asm__ volatile ("nop");
            return false;
        }
        (void)utk_kernel_delay(1u);
    }
    g_flash_session_active = true;
    return true;
}

void RAM_CODE utk_flash_end(void)
{
    if (!g_flash_session_active) return;
    g_flash_session_active = false;
    __asm__ volatile ("dmb" ::: "memory");
    g_flash_release = 1u;
    __asm__ volatile ("dmb" ::: "memory");
    while (g_flash_parked != 0u) __asm__ volatile ("nop");
}

static RAM_CODE void flash_copy_boot2(void)
{
    if (g_boot2_copyout_valid) return;
    const volatile uint32_t *source =
        (const volatile uint32_t *)(uintptr_t)XIP_BASE;
    for (uint32_t i = 0u; i < BOOT2_WORDS; i++) {
        g_boot2_copyout[i] = source[i];
    }
    __asm__ volatile ("dmb" ::: "memory");
    g_boot2_copyout_valid = true;
}

static RAM_CODE void flash_save_qspi_pads(void)
{
    for (uint32_t i = 0u; i < PADS_QSPI_COUNT; i++) {
        g_qspi_pads[i] = MMIO32(PADS_QSPI_BASE + 4u + (4u * i));
    }
}

static RAM_CODE void flash_restore_qspi_pads(void)
{
    for (uint32_t i = 0u; i < PADS_QSPI_COUNT; i++) {
        MMIO32(PADS_QSPI_BASE + 4u + (4u * i)) = g_qspi_pads[i];
    }
}

static RAM_CODE bool flash_range_valid(uint32_t offset, uint32_t count,
                                       uint32_t alignment)
{
    if (count == 0u || (offset & (alignment - 1u)) != 0u ||
        (count & (alignment - 1u)) != 0u) return false;
    return offset < PICO_FLASH_SIZE_BYTES &&
           count <= PICO_FLASH_SIZE_BYTES - offset;
}

static RAM_CODE void flash_enter_write_mode(void)
{
    rom_connect_internal_flash_fn connect_internal_flash =
        (rom_connect_internal_flash_fn)rom_func_lookup_inline(
            ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_xip =
        (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    flash_save_qspi_pads();
    if (connect_internal_flash == NULL || exit_xip == NULL) {
        g_flash_error = 1u;
        return;
    }
    connect_internal_flash();
    exit_xip();
}

static RAM_CODE void flash_exit_write_mode(void)
{
    rom_flash_flush_cache_fn flush =
        (rom_flash_flush_cache_fn)rom_func_lookup_inline(
            ROM_FUNC_FLASH_FLUSH_CACHE);
    if (flush == NULL) g_flash_error = 1u;
    else flush();
    ((void (*)(void))((uintptr_t)g_boot2_copyout | 1u))();
    flash_restore_qspi_pads();
}

void RAM_CODE flash_range_erase(uint32_t flash_offs, size_t count)
{
    rom_flash_range_erase_fn erase =
        (rom_flash_range_erase_fn)rom_func_lookup_inline(
            ROM_FUNC_FLASH_RANGE_ERASE);
    if (!g_flash_session_active || g_flash_error != 0u || count == 0u ||
        !flash_range_valid(flash_offs, (uint32_t)count, FLASH_SECTOR_SIZE) ||
        erase == NULL) {
        g_flash_error = 1u;
        return;
    }
    flash_copy_boot2();
    flash_enter_write_mode();
    if (g_flash_error == 0u) {
        erase(flash_offs, count, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
        flash_exit_write_mode();
    }
}

void RAM_CODE flash_range_program(uint32_t flash_offs, const uint8_t *data,
                                  size_t count)
{
    rom_flash_range_program_fn program =
        (rom_flash_range_program_fn)rom_func_lookup_inline(
            ROM_FUNC_FLASH_RANGE_PROGRAM);
    if (!g_flash_session_active || g_flash_error != 0u || data == NULL || count == 0u ||
        !flash_range_valid(flash_offs, (uint32_t)count, FLASH_PAGE_SIZE) ||
        program == NULL) {
        g_flash_error = 1u;
        return;
    }
    flash_copy_boot2();
    flash_enter_write_mode();
    if (g_flash_error == 0u) {
        program(flash_offs, data, count);
        flash_exit_write_mode();
    }
}

void watchdog_reboot(uint32_t delay_ms, uint32_t pc, uint32_t sp)
{
    const uintptr_t watchdog = 0x40058000u;
    MMIO32(watchdog + 0x04u) = delay_ms * 1000u;
    MMIO32(watchdog + 0x0cu) = pc;
    MMIO32(watchdog + 0x10u) = sp;
    MMIO32(watchdog + 0x00u) = (1u << 31) | (1u << 30);
    for (;;) __asm__ volatile ("wfi");
}

