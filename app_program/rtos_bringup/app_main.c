#include "bringup.h"

static uint64_t port_now(void *user) {
    (void)user;
    return board_time_us();
}
static void port_off(void *user) {
    (void)user;
    board_motor_off();
}

static ER start(FP entry, PRI priority, UW processor, const char *name) {
    T_CTSK definition = {0};
    definition.task = entry;
    definition.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
    definition.assprc = processor;
    definition.itskpri = priority;
    definition.stksz = 4096;
    ID id = tk_cre_tsk(&definition);
    if (id < E_OK)
        return id;
    ER result = tk_sta_tsk(id, 0);
    tm_printf((UB *)"TASK name=%s id=%d priority=%d core=%d result=%d\n", name, id, priority,
              processor - 1U, result);
    return result;
}

void bringup_help(void) {
    tm_printf((UB *)"\nROBOPICO RTOS MOTOR BRINGUP v0.1 | Step 1 only | micro T-Kernel SMP\n");
    tm_printf(
        (UB *)"Motors OFF on boot. USB bench only; Wi-Fi, line/barcode/IMU/Buddy5 not running.\n");
    tm_printf(
        (UB *)"1=left forward, 2=right forward, 3=both forward: 45mm/s targets, 3s maximum.\n");
    tm_printf(
        (UB *)"s=stop; e=latched emergency stop; x=clear latch while stopped/buttons released.\n");
    tm_printf((UB *)"GP20=latched emergency stop (IRQ plus supervisor); GP21=normal stop.\n");
    tm_printf((UB *)"h=help; p/n=clock+motor diagnostics; z=nav requests/results.\n");
    tm_printf(
        (UB *)"First test timing and encoders with wheels lifted; do not use on a circuit yet.\n");
    tm_printf((
        UB *)"No calibration or Wi-Fi settings are read/written. CLOCK requires 3 good windows.\n");
}

void bringup_report(void) {
    BringupStatus s;
    bringup_get_status(&s);
    tm_printf(
        (UB *)"CLOCK kernel_ms=%u timer_us=%u window_kernel_ms=%u window_timer_us=%u samples=%u good=%u status=%s\n",
        s.kernel_ms, s.timer_us, s.clock_kernel_ms, s.clock_timer_us, s.clock_samples, s.clock_good,
        s.flags & BR_CLOCK_OK ? "PASS"
        : s.clock_samples     ? "CHECK"
                              : "PENDING");
    tm_printf(
        (UB *)"CONTROL count=%u dt_us=%u min_dt_us=%u max_dt_us=%u max_cost_us=%u late=%u flags=0x%x active=%u reason=%u usb=%u tx_drop=%u rx_drop=%u\n",
        s.control_count, s.dt_us, s.min_dt_us, s.max_dt_us, s.max_cost_us, s.late_count, s.flags,
        (s.flags & BR_ACTIVE) != 0U, s.reason, tm_usb_state(), tm_usb_dropped, tm_usb_rx_dropped);
    tm_printf(
        (UB *)"WHEELS ticks=%d,%d edges=%u,%u invalid=%u,%u speed_x10=%d,%d target=%d,%d pwm=%d,%d gp20=%u gp21=%u irq_stop=%u\n",
        s.ticks[0], s.ticks[1], s.edges[0], s.edges[1], s.invalid[0], s.invalid[1], s.speed_x10[0],
        s.speed_x10[1], s.target[0], s.target[1], s.pwm[0], s.pwm[1], board_button_pressed(20U),
        board_button_pressed(21U), bringup_irq_stop);
}

void bringup_status_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    uint32_t next = board_kernel_ms();
    unsigned reports = 0U;
    for (;;) {
        uint32_t now = board_kernel_ms();
        if ((int32_t)(now - next) >= 0) {
            bringup_report();
            if ((reports++ % 5U) == 0U)
                board_clock_report();
            next += 1000U;
            if ((int32_t)(now - next) >= 0)
                next = now + 1000U;
        }
        (void)tk_dly_tsk(10);
    }
}

EXPORT INT usermain(void) {
    T_RVER version;
    (void)tk_ref_ver(&version);
    bringup_help();
    tm_printf((UB *)"KERNEL spec=0x%x product=0x%x tick_ms=%u initial_core=%u\n", version.spver,
              version.prver, CNF_TIMER_PERIOD, tk_get_prc() - 1U);
    board_clock_report();
    const MotionPort port = {.lock = board_motion_lock,
                             .unlock = board_motion_unlock,
                             .outputs_off = port_off,
                             .now_us = port_now};
    motion_init(&port);
    ER result = bringup_create_objects();
    if (result == E_OK)
        result = start((FP)bringup_control_task, 14, TP_PRC2, "motion");
    if (result == E_OK)
        result = start((FP)bringup_safety_task, 6, TP_PRC2, "safety");
    if (result == E_OK)
        result = start((FP)bringup_nav_task, 10, TP_PRC2, "nav-bench");
    if (result == E_OK)
        result = start((FP)bringup_console_task, 12, TP_PRC1, "usb-command");
    if (result == E_OK)
        result = start((FP)bringup_status_task, 18, TP_PRC1, "diagnostics");
    if (result != E_OK) {
        bringup_stop(BR_REASON_API, BR_ESTOP);
        tm_printf((UB *)"INIT FAILED result=%d; motors disabled.\n", result);
    }
    for (;;)
        (void)tk_slp_tsk(TMO_FEVR);
}
