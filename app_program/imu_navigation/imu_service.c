#ifdef APP_RTOS_IMU
#include "imu_service.h"
#include "../network_bridge/network_bridge.h"
#include "src/imu/imu_iface.h"
#include <string.h>

/* One I2C/estimator writer on core0. Motion on core1 supplies observations;
 * all cross-task data are copied under this short lock. No I/O under lock. */
static T_SPLOCK copy_lock;
static struct {
    b4v23_odo_t odo;
    b4v23_tilt_t tilt;
    imu_tlm_t tlm;
    uint64_t observed_us;
    uint32_t invalid[2], run_serial, ticks, tick_us, max_tick_us, tx_drop;
    unsigned have : 1, running : 1, reset : 1;
} shared;
static UINT enter(void) {
    UINT key;
    (void)ISpinLock(&copy_lock, &key);
    return key;
}
static void leave(UINT key) {
    (void)ISpinUnlock(&copy_lock, key);
}

void ri_init(void) {
    memset(&shared, 0, sizeof(shared));
    /* Static-zero spinlock; initialise before either task is created. */
}
void ri_reset_request(void) {
    UINT key = enter();
    shared.reset = true;
    leave(key);
}
void ri_new_run(void) {
    ri_ramp_halt(board_time_us());
    UINT key = enter();
    ++shared.run_serial;
    leave(key);
}
void ri_observe(const MotionFeedback *f, const uint32_t invalid[2], uint32_t dt_us, bool running) {
    UINT key = enter();
    bool clean = shared.have && dt_us && dt_us <= 50000 && invalid[0] == shared.invalid[0] &&
                 invalid[1] == shared.invalid[1];
    shared.odo = (b4v23_odo_t){.have = true,
                               .clean = clean,
                               .centre_mm = (f->wheel_mm[0] + f->wheel_mm[1]) * .5,
                               .heading_dd = (f->wheel_mm[0] - f->wheel_mm[1]) / TRACK_MM *
                                             (1800.0 / 3.141592653589793),
                               .left_mm_s = f->speed_mm_s[0],
                               .right_mm_s = f->speed_mm_s[1],
                               .invalid_left = invalid[0],
                               .invalid_right = invalid[1]};
    shared.invalid[0] = invalid[0];
    shared.invalid[1] = invalid[1];
    shared.observed_us = f->now_us;
    shared.have = true;
    shared.running = running;
    leave(key);
}
void ri_tilt_snapshot(b4v23_tilt_t *out) {
    UINT key = enter();
    *out = shared.tilt;
    leave(key);
}
static bool send_frame(const uint8_t *frame, uint32_t len) {
    imu_tlm_t t;
    if (imu_tlm_unpack(frame, len, &t)) {
        UINT key = enter();
        shared.tlm = t;
        leave(key);
    }
    if (!rn_drive_telemetry_enabled() || !rn_drive_online())
        return false;
    bool ok = len <= RN_PACKET_MAX && rn_tx_push(RN_TLM, frame, (uint8_t)len, board_kernel_ms());
    if (!ok) {
        UINT key = enter();
        ++shared.tx_drop;
        leave(key);
    }
    return ok;
}
void ri_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    b4v23_init(send_frame, false);
    bool was_running = false;
    uint32_t run_serial = 0;
    for (;;) {
        uint64_t now = board_time_us();
        if (b4v23_due(now)) {
            UINT key = enter();
            b4v23_odo_t odo = shared.odo;
            uint64_t observed = shared.observed_us;
            bool running = shared.running, reset = shared.reset;
            uint32_t serial = shared.run_serial;
            shared.reset = false;
            leave(key);
            odo.age_ms = now >= observed ? (uint32_t)((now - observed) / 1000) : UINT32_MAX;
            b4v23_mission_t mission = B4V23_NONE;
            if (reset) {
                mission = B4V23_RESET;
                was_running = false;
            } else if (running && (!was_running || run_serial != serial)) {
                mission = run_serial == serial ? B4V23_RESUME : B4V23_START;
                run_serial = serial;
                was_running = true;
            } else if (!running && was_running) {
                mission = B4V23_STOP;
                was_running = false;
            }
            b4v23_tick(now, mission, &odo);
            b4v23_tilt_t tilt;
            b4v23_tilt(&tilt);
            key = enter();
            shared.tilt = tilt;
            ++shared.ticks;
            shared.tick_us = b4v23_last_tick_us();
            shared.max_tick_us = b4v23_max_tick_us();
            leave(key);
        }
        (void)tk_dly_tsk(1);
    }
}
void ri_report(void) {
    UINT key = enter();
    imu_tlm_t t = shared.tlm;
    b4v23_tilt_t tilt = shared.tilt;
    uint32_t ticks = shared.ticks, cost = shared.tick_us, maxcost = shared.max_tick_us,
             drop = shared.tx_drop;
    leave(key);
    uint64_t now = board_time_us();
    uint32_t age = now >= tilt.t_us ? (uint32_t)((now - tilt.t_us) / 1000) : UINT32_MAX;
    tm_printf(
        (UB *)"IMU detector=6 state=%u motion=%u valid=%u fault=%u pitch_dd=%d humps=%u rejected=%u max_height_mm=%u i2c_errors=%u missed=%u ticks=%u tick_us=%u max_tick_us=%u tx_drop=%u\n",
        t.state, t.motion, t.b_valid, t.fault, t.pitch_dd, t.humps, t.rejected, t.max_height_mm,
        t.i2c_errors, t.missed, ticks, cost, maxcost, drop);
    tm_printf((UB *)"IMU_TILT seq=%u age_ms=%u valid=%u tracking=%u height_mm=%d quality=%u\n",
              tilt.seq, age, tilt.valid, tilt.tracking, tilt.height_mm, tilt.quality_reasons);
    ri_ramp_report();
}
#endif
