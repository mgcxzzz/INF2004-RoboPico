/**
 * @file app_main.c
 * @brief App main.
 * @details Core1 owns normal motor commits. Safety paths may only remove
 * output; command/status access uses the existing kernel locks and copied
 * records.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
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

static uint64_t port_now (void * p_user)
{
    (void)p_user;
    return board_time_us();
}
static void port_off (void * p_user)
{
    (void)p_user;
    board_motor_off();
}

static ER start (FP p_entry, PRI priority, UW processor, const char * p_name)
{
    T_CTSK definition = {0};
    definition.task = p_entry;
    definition.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
    definition.assprc = processor;
    definition.itskpri = priority;
    definition.stksz = 4096;
    ID identifier = tk_cre_tsk(&definition);
    if (identifier < E_OK)
    {
        return identifier;
    }
    ER result = tk_sta_tsk(identifier, 0);
    tm_printf((UB *)"TASK name=%s id=%d priority=%d core=%d result=%d\n",
              p_name, identifier, priority, processor - 1U, result);
    return result;
}

void bringup_help (void)
{
#if defined(APP_RTOS_RELEASE)
    tm_printf((UB *)"\nROBOPICO BUDDY1-4 RTOS v0.2.9 CAL REPORTING | "
                    "PROVISIONAL | micro "
                    "T-Kernel SMP | Buddy5 NOT integrated\n");
    tm_printf((UB *)"Tested Step5 v0.1.5 logic: Continuous V2 Profile1 45mm/s; "
                    "barcode/junction110mm; IMU/ramp via NAV.\n");
    tm_printf((
        UB *)"Powerbank supported: no USB DTR requirement. Fresh MQTT "
             "link/service and TCP delivery required; no automatic restart.\n");
    tm_printf(
        (UB *)"Motors OFF at boot. Dashboard Start2s/Pause/Resume/Stop/Reset; "
              "s/GP21 Stop, e/GP20 E-stop.\n");
    tm_printf((UB *)"Saved calibration retained; w/l/r/k available, "
                    "c=fallback. Keep still for IMU calibration.\n");
    tm_printf(
        (UB *)"v0.2.2 retained: cross7..30mm; persistent bend assist; bounded "
              "stationary recovery. Physical acceptance pending.\n");
    tm_printf((UB *)"IMU end-run: sustained lift or >60deg tilt latches motors "
                    "OFF. Upright/still500ms then Reset/recalibrate; explicit "
                    "Start. C9 safety always reported.\n");
    tm_printf((UB *)"Stopped wireless logs; latest barcode attempt and latched "
                    "encoder fault; RAM-only, export before Start/reboot. "
                    "n=status,b/d=USB exports.\n");
#elif defined(APP_RTOS_IMU)
    tm_printf((UB *)"\nROBOPICO RTOS CONNECTED IMU v0.1.5 | Step5 EXPERIMENTAL "
                    "| micro T-Kernel SMP\n");
    tm_printf((UB *)"Based on Step4 v0.1.2 (physical acceptance pending). "
                    "Buddy1-4; Buddy5 NOT integrated.\n");
    tm_printf((UB *)"GY-511 I2C1 GP2/GP3,50Hz detector6 task; keep still2s. "
                    "Dashboard Reset restarts IMU calibration.\n");
    tm_printf((UB *)"Ramp modifier: Buddy4 -> NAV -> MOTION; bounded "
                    "boost/hold/push;420 cap. IMU faults remove assistance, "
                    "not an unconditional global stop.\n");
    tm_printf((UB *)"Continuous V2/front calibration/barcode/junction110mm and "
                    "relaxed gate retained. USB DTR supervision REQUIRED.\n");
    tm_printf((UB *)"Same dashboard/capture and stopped b/d exports. n=status, "
                    "s/GP21=Stop,e/GP20=E-stop; no powerbank driving yet.\n");
#elif defined(APP_RTOS_BARCODE)
    tm_printf((UB *)"\nROBOPICO RTOS CONNECTED BARCODE v0.1.2 | Step4 | micro "
                    "T-Kernel SMP\n");
    tm_printf((UB *)"Independent contrast junction sensor; 4mm white re-arm; "
                    "sensor ahead110mm.\n");
    tm_printf((UB *)"Gate at confirmed cross; no300ms/3deg/speed dwell. Reject "
                    "flags:1=legacy approach,2=clearance,4=line lost,8=not "
                    "visible,16=search.\n");
    tm_printf((UB *)"Original63-bank + peak Code39, Buddy3 -> NAV -> MOTION; "
                    "live A-Z mapping.\n");
    tm_printf((UB *)"Continuous V2/profile1/calibration/Step2 junction policy "
                    "retained; IMU/ramp/Buddy5 deferred.\n");
    tm_printf((UB *)"USB DTR supervision REQUIRED; dashboard "
                    "Start2s/Pause/Resume/Stop/E-stop/Reset.\n");
    tm_printf(
        (UB *)"w/l/r/k calibration retained; do NOT recalibrate for "
              "comparison. b=stopped barcode/raw log, d=line log, n=status.\n");
    tm_printf((UB *)"Line RAM640 rows (6.4s); raw2048 GP27 samples freeze on "
                    "first decode. No automatic restart after loss.\n");
    tm_printf((UB *)"Exact-size TCP allocations; heap/delivery diagnostics. "
                    "Manual canonical USB L/R/U disabled.\n");
#elif defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
    tm_printf((UB *)"\nROBOPICO RTOS CONNECTED LINE v0.1.2 | exact-size TCP "
                    "allocation | Step3B | micro T-Kernel SMP\n");
    tm_printf((UB *)"Step2 v0.1.2 Continuous V2/profile1/calibration/junction "
                    "policy retained.\n");
    tm_printf((UB *)"USB DTR supervision REQUIRED. Dashboard "
                    "Start2s/Stop/E-stop/Pause/Resume/Reset.\n");
    tm_printf((UB *)"Resume only on visible line after ordinary tracking "
                    "pause; interrupted junction requires Stop/new Start.\n");
    tm_printf((UB *)"g=Start; s/GP21=Stop; e/GP20=E-stop; x=clear; L/R/U=queue "
                    "canonical route.\n");
    tm_printf((UB *)"w/l/r=RAM calibration; k=save stopped; c=preset; "
                    "d=stopped USB log; n=status; z=nav.\n");
    tm_printf((UB *)"Network loss cancels Start/stops session; no automatic "
                    "restart. Barcode/IMU/Buddy5 deferred.\n");
#elif defined(APP_RTOS_NETWORK)
    tm_printf((UB *)"\nROBOPICO RTOS NETWORK BENCH v0.1 | Step3A | micro "
                    "T-Kernel SMP\n");
    tm_printf((UB *)"MOTORS DISABLED. 100Hz clock/encoder/stop foundation + "
                    "kernel-owned MQTT.\n");
    tm_printf((UB *)"Dashboard: heartbeat, Ping, RAM mapping, Stop/E-stop "
                    "only. Start/Resume unsupported.\n");
    tm_printf((UB *)"p/n=status; x=clear local E-stop; h=help. No "
                    "ADC/line/barcode/IMU or calibration writes.\n");
#elif defined(APP_RTOS_LINE)
    tm_printf((UB *)"\nROBOPICO RTOS LINE FOLLOWING v0.1.2 | ANGLE-ONLY "
                    "JUNCTION EXPERIMENT | Step2 | micro T-Kernel SMP\n");
    tm_printf((UB *)"Continuous V2 profile1 only. Motors OFF on boot. USB "
                    "connected while running.\n");
    tm_printf(
        (UB *)"Junction: >10deg for30ms arms; prior both-white NOT required. "
              "Branch angles/30ms/acquisition/stops retained.\n");
    tm_printf((UB *)"g=follow after2s; s/GP21=stop; e/GP20=latched stop; "
                    "x=clear latch.\n");
    tm_printf((UB *)"L/R/U=queue left/right/U-turn during a run; separate bare "
                    "cross triggers it.\n");
    tm_printf((UB *)"w=both white; l=left black; r=right black; k=save; "
                    "c=restore original preset to RAM.\n");
    tm_printf((UB *)"Calibration priority: RTOS saved page, valid SDK "
                    "record28, original preset.\n");
    tm_printf((UB *)"p/n=status+calibration; z=nav audit; d=export stopped RAM "
                    "drive log; h=help.\n");
    tm_printf((UB *)"No Wi-Fi/MQTT, barcode decoding, IMU/ramp assist or "
                    "Buddy5 in this checkpoint.\n");
    tm_printf((UB *)"Keep sensors at current low/tilted mounting. Test "
                    "straight, bend, then junction separately.\n");
#else
    tm_printf((UB *)"\nROBOPICO RTOS MOTOR BRINGUP v0.1.1 | Step 1 only | "
                    "micro T-Kernel SMP\n");
    tm_printf((UB *)"Motors OFF on boot. USB bench only; Wi-Fi, "
                    "line/barcode/IMU/Buddy5 not running.\n");
    tm_printf((UB *)"1=left forward, 2=right forward, 3=both forward: 45mm/s "
                    "targets, 3s maximum.\n");
    tm_printf((UB *)"s=stop; e=latched emergency stop; x=clear latch while "
                    "stopped/buttons released.\n");
    tm_printf((UB *)"GP20=latched emergency stop (IRQ plus supervisor); "
                    "GP21=normal stop.\n");
    tm_printf(
        (UB *)"h=help; p/n=clock+motor diagnostics; z=nav requests/results.\n");
    tm_printf((UB *)"First test timing and encoders with wheels lifted; do not "
                    "use on a circuit yet.\n");
    tm_printf((UB *)"No calibration or Wi-Fi settings are read/written. CLOCK "
                    "requires 3 good windows.\n");
#endif
}

static void report_pause (bool paced)
{
#ifdef APP_RTOS_IMU
    /* Reports run only in the USB console or low-priority diagnostics task. */
    if (paced)
    {
        (void)tk_dly_tsk(10);
    }
#else
    (void)paced;
#endif
}

static void bringup_report_impl (bool paced)
{
    bringup_status_t state;
    bringup_get_status(&state);
    tm_printf((UB *)"CLOCK kernel_ms=%u timer_us=%u window_kernel_ms=%u "
                    "window_timer_us=%u samples=%u good=%u status=%s\n",
              state.kernel_ms, state.timer_us, state.clock_kernel_ms,
              state.clock_timer_us, state.clock_samples, state.clock_good,
              state.flags & BR_CLOCK_OK ? "PASS"
              : state.clock_samples     ? "CHECK"
                                        : "PENDING");
    report_pause(paced);
    tm_printf((UB *)"CONTROL count=%u dt_us=%u min_dt_us=%u max_dt_us=%u "
                    "max_cost_us=%u late=%u flags=0x%x active=%u reason=%u "
                    "usb=%u tx_drop=%u rx_drop=%u\n",
              state.control_count, state.dt_us, state.min_dt_us,
              state.max_dt_us, state.max_cost_us, state.late_count, state.flags,
              (state.flags & BR_ACTIVE) != 0U, state.reason, tm_usb_state(),
              tm_usb_dropped, tm_usb_rx_dropped);
    report_pause(paced);
    tm_printf(
        (UB *)"WHEELS ticks=%d,%d edges=%u,%u invalid=%u,%u speed_x10=%d,%d "
              "target=%d,%d pwm=%d,%d gp20=%u gp21=%u irq_stop=%u\n",
        state.ticks[0], state.ticks[1], state.edges[0], state.edges[1],
        state.invalid[0], state.invalid[1], state.speed_x10[0],
        state.speed_x10[1], state.target[0], state.target[1], state.pwm[0],
        state.pwm[1], board_button_pressed(20U), board_button_pressed(21U),
        g_bringup_irq_stop);
#ifdef APP_RTOS_LINE
    tm_printf((UB *)"ENCODER_GUARD bad=%u,%u changes=%u,%u trip_kind=%u "
                    "trip_wheel=%u\n",
              state.encoder_bad[0], state.encoder_bad[1],
              state.encoder_changes[0], state.encoder_changes[1],
              state.encoder_kind, state.encoder_wheel);
    rtos_line_report();
    report_pause(paced);
#endif
#ifdef APP_RTOS_NETWORK
    rn_report();
#endif
#ifdef APP_RTOS_IMU
    report_pause(paced);
    ri_report();
    report_pause(paced);
    UW log[4];
    extern void tm_usb_log_snapshot(UW out[4]);
    tm_usb_log_snapshot(log);
    tm_printf((UB *)"LOGGING atomic=1 records=%u dropped=%u oversize=%u "
                    "max_copy_us=%u\n",
              log[0], log[1], log[2], log[3]);
#endif
}

void bringup_report (void)
{
    bringup_report_impl(false);
}

void bringup_status_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    uint32_t next = board_kernel_ms();
    unsigned reports = 0U;
    for (;;)
    {
#ifdef APP_RTOS_LINE
        rtos_line_dump_if_requested();
#endif
        uint32_t now = board_kernel_ms();
        if ((int32_t)(now - next) >= 0)
        {
#ifdef APP_RTOS_RELEASE
            /* No absent-host log flood while running from a powerbank. */
            if (3U == tm_usb_state())
#endif
                bringup_report_impl(true);
            if (0U == (reports++ % 5U)
#ifdef APP_RTOS_RELEASE
                && 3U == tm_usb_state()
#endif
            )
                board_clock_report();
            next += 1000U;
            if ((int32_t)(now - next) >= 0)
            {
                next = now + 1000U;
            }
        }
        (void)tk_dly_tsk(10);
    }
}

EXPORT INT usermain (void)
{
    T_RVER version;
    (void)tk_ref_ver(&version);
    bringup_help();
    tm_printf(
        (UB *)"KERNEL spec=0x%x product=0x%x tick_ms=%u initial_core=%u\n",
        version.spver, version.prver, CNF_TIMER_PERIOD, tk_get_prc() - 1U);
    board_clock_report();
    const motion_port_t port = {.lock = board_motion_lock,
                                .unlock = board_motion_unlock,
                                .outputs_off = port_off,
                                .now_us = port_now};
    motion_init(&port);
    ER result = bringup_create_objects();
#ifdef APP_RTOS_IMU
    ri_init();
#endif
#ifdef APP_RTOS_NETWORK
    if (E_OK == result)
    {
        result = rn_init();
    }
    if (E_OK == result)
    {
        result = start((FP)rn_app_task, 16, TP_PRC1, "network-bridge");
    }
#endif
#ifdef APP_RTOS_LINE
    if (E_OK == result)
    {
        result = start((FP)rtos_line_flash_task, 5, TP_PRC1, "flash-park");
    }
#endif
    if (E_OK == result)
    {
        result = start((FP)bringup_control_task, 14, TP_PRC2, "motion");
    }
#ifdef APP_RTOS_IMU
    if (E_OK == result)
    {
        result = start((FP)ri_task, RI_TASK_PRIORITY, RI_TASK_PROCESSOR,
                       "imu-terrain");
    }
#endif
    if (E_OK == result)
    {
        result = start((FP)bringup_safety_task, 6, TP_PRC2, "safety");
    }
    if (E_OK == result)
    {
        result = start((FP)bringup_nav_task, 10, TP_PRC2, "nav-bench");
    }
    if (E_OK == result)
    {
        result = start((FP)bringup_console_task, 12, TP_PRC1, "usb-command");
    }
    if (E_OK == result)
    {
        result = start((FP)bringup_status_task, 18, TP_PRC1, "diagnostics");
    }
    if (result != E_OK)
    {
        bringup_stop(BR_REASON_API, BR_ESTOP);
        tm_printf((UB *)"INIT FAILED result=%d; motors disabled.\n", result);
    }
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/* End of file. */
