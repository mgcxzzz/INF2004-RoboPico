#include "bringup.h"
#ifdef APP_RTOS_IMU
#include "../imu_navigation/imu_service.h"
#endif
#ifdef APP_RTOS_NETWORK
#include "../network_bridge/network_bridge.h"
#endif
#ifdef APP_RTOS_LINE
#include "../line_navigation/line_service.h"
#endif

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
#if defined(APP_RTOS_IMU)
    tm_printf(
        (UB *)"\nROBOPICO RTOS CONNECTED IMU v0.1.4 | Step5 EXPERIMENTAL | micro T-Kernel SMP\n");
    tm_printf((
        UB *)"Based on Step4 v0.1.2 (physical acceptance pending). Buddy1-4; Buddy5 NOT integrated.\n");
    tm_printf((
        UB *)"GY-511 I2C1 GP2/GP3,50Hz detector6 task; keep still2s. Dashboard Reset restarts IMU calibration.\n");
    tm_printf((
        UB *)"Ramp modifier: Buddy4 -> NAV -> MOTION; bounded boost/hold/push;420 cap. IMU faults remove assistance, not an unconditional global stop.\n");
    tm_printf((
        UB *)"Continuous V2/front calibration/barcode/junction110mm and relaxed gate retained. USB DTR supervision REQUIRED.\n");
    tm_printf((
        UB *)"Same dashboard/capture and stopped b/d exports. n=status, s/GP21=Stop,e/GP20=E-stop; no powerbank driving yet.\n");
#elif defined(APP_RTOS_BARCODE)
    tm_printf((UB *)"\nROBOPICO RTOS CONNECTED BARCODE v0.1.2 | Step4 | micro T-Kernel SMP\n");
    tm_printf((UB *)"Independent contrast junction sensor; 4mm white re-arm; sensor ahead110mm.\n");
    tm_printf((
        UB *)"Gate at confirmed cross; no300ms/3deg/speed dwell. Reject flags:1=legacy approach,2=clearance,4=line lost,8=not visible,16=search.\n");
    tm_printf((UB *)"Original63-bank + peak Code39, Buddy3 -> NAV -> MOTION; live A-Z mapping.\n");
    tm_printf((
        UB *)"Continuous V2/profile1/calibration/Step2 junction policy retained; IMU/ramp/Buddy5 deferred.\n");
    tm_printf(
        (UB *)"USB DTR supervision REQUIRED; dashboard Start2s/Pause/Resume/Stop/E-stop/Reset.\n");
    tm_printf((
        UB *)"w/l/r/k calibration retained; do NOT recalibrate for comparison. b=stopped barcode/raw log, d=line log, n=status.\n");
    tm_printf((
        UB *)"Line RAM640 rows (6.4s); raw2048 GP27 samples freeze on first decode. No automatic restart after loss.\n");
    tm_printf((
        UB *)"Exact-size TCP allocations; heap/delivery diagnostics. Manual canonical USB L/R/U disabled.\n");
#elif defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
    tm_printf((
        UB *)"\nROBOPICO RTOS CONNECTED LINE v0.1.2 | exact-size TCP allocation | Step3B | micro T-Kernel SMP\n");
    tm_printf((UB *)"Step2 v0.1.2 Continuous V2/profile1/calibration/junction policy retained.\n");
    tm_printf(
        (UB *)"USB DTR supervision REQUIRED. Dashboard Start2s/Stop/E-stop/Pause/Resume/Reset.\n");
    tm_printf((
        UB *)"Resume only on visible line after ordinary tracking pause; interrupted junction requires Stop/new Start.\n");
    tm_printf((UB *)"g=Start; s/GP21=Stop; e/GP20=E-stop; x=clear; L/R/U=queue canonical route.\n");
    tm_printf((
        UB *)"w/l/r=RAM calibration; k=save stopped; c=preset; d=stopped USB log; n=status; z=nav.\n");
    tm_printf((
        UB *)"Network loss cancels Start/stops session; no automatic restart. Barcode/IMU/Buddy5 deferred.\n");
#elif defined(APP_RTOS_NETWORK)
    tm_printf((UB *)"\nROBOPICO RTOS NETWORK BENCH v0.1 | Step3A | micro T-Kernel SMP\n");
    tm_printf((UB *)"MOTORS DISABLED. 100Hz clock/encoder/stop foundation + kernel-owned MQTT.\n");
    tm_printf((
        UB *)"Dashboard: heartbeat, Ping, RAM mapping, Stop/E-stop only. Start/Resume unsupported.\n");
    tm_printf((
        UB *)"p/n=status; x=clear local E-stop; h=help. No ADC/line/barcode/IMU or calibration writes.\n");
#elif defined(APP_RTOS_LINE)
    tm_printf((
        UB *)"\nROBOPICO RTOS LINE FOLLOWING v0.1.2 | ANGLE-ONLY JUNCTION EXPERIMENT | Step2 | micro T-Kernel SMP\n");
    tm_printf(
        (UB *)"Continuous V2 profile1 only. Motors OFF on boot. USB connected while running.\n");
    tm_printf((
        UB *)"Junction: >10deg for30ms arms; prior both-white NOT required. Branch angles/30ms/acquisition/stops retained.\n");
    tm_printf((UB *)"g=follow after2s; s/GP21=stop; e/GP20=latched stop; x=clear latch.\n");
    tm_printf(
        (UB *)"L/R/U=queue left/right/U-turn during a run; separate bare cross triggers it.\n");
    tm_printf((
        UB *)"w=both white; l=left black; r=right black; k=save; c=restore original preset to RAM.\n");
    tm_printf(
        (UB *)"Calibration priority: RTOS saved page, valid SDK record28, original preset.\n");
    tm_printf(
        (UB *)"p/n=status+calibration; z=nav audit; d=export stopped RAM drive log; h=help.\n");
    tm_printf(
        (UB *)"No Wi-Fi/MQTT, barcode decoding, IMU/ramp assist or Buddy5 in this checkpoint.\n");
    tm_printf((
        UB *)"Keep sensors at current low/tilted mounting. Test straight, bend, then junction separately.\n");
#else
    tm_printf((UB *)"\nROBOPICO RTOS MOTOR BRINGUP v0.1.1 | Step 1 only | micro T-Kernel SMP\n");
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
#endif
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
#ifdef APP_RTOS_LINE
    tm_printf((UB *)"ENCODER_GUARD bad=%u,%u changes=%u,%u trip_kind=%u trip_wheel=%u\n",
              s.encoder_bad[0], s.encoder_bad[1], s.encoder_changes[0], s.encoder_changes[1],
              s.encoder_kind, s.encoder_wheel);
    rtos_line_report();
#endif
#ifdef APP_RTOS_NETWORK
    rn_report();
#endif
#ifdef APP_RTOS_IMU
    ri_report();
    UW log[4];
    extern void tm_usb_log_snapshot(UW out[4]);
    tm_usb_log_snapshot(log);
    tm_printf((UB *)"LOGGING atomic=1 records=%u dropped=%u oversize=%u max_copy_us=%u\n",
              log[0], log[1], log[2], log[3]);
#endif
}

void bringup_status_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    uint32_t next = board_kernel_ms();
    unsigned reports = 0U;
    for (;;) {
#ifdef APP_RTOS_LINE
        rtos_line_dump_if_requested();
#endif
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
#ifdef APP_RTOS_IMU
    ri_init();
#endif
#ifdef APP_RTOS_NETWORK
    if (result == E_OK)
        result = rn_init();
    if (result == E_OK)
        result = start((FP)rn_app_task, 16, TP_PRC1, "network-bridge");
#endif
#ifdef APP_RTOS_LINE
    if (result == E_OK)
        result = start((FP)rtos_line_flash_task, 5, TP_PRC1, "flash-park");
#endif
    if (result == E_OK)
        result = start((FP)bringup_control_task, 14, TP_PRC2, "motion");
#ifdef APP_RTOS_IMU
    if (result == E_OK)
        result = start((FP)ri_task, RI_TASK_PRIORITY, RI_TASK_PROCESSOR, "imu-terrain");
#endif
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
