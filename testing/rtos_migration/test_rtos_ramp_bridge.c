/* Real RTOS ramp bridge + controller + coordinator; deterministic OS/I/O. */
#define APP_RTOS_IMU 1
#define APP_RTOS_BARCODE 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#define RTOS_BRINGUP_H
#include <assert.h>
#include <stdio.h>
#include "src/motion/motion_port.h"
typedef unsigned UINT;
typedef unsigned char UB;
typedef unsigned T_SPLOCK;
typedef int INT;
static unsigned depth, packets;
static uint64_t now = 1000000;
static bool publish_while_copying;
#include "app_program/imu_navigation/imu_motion.h"
static b4v23_tilt_t tilt;
static int ISpinLock(T_SPLOCK *p, UINT *key) {
    (void)p;
    *key = 0;
    ++depth;
    return 0;
}
static int ISpinUnlock(T_SPLOCK *p, UINT key) {
    (void)p;
    (void)key;
    assert(depth);
    --depth;
    return 0;
}
static void tm_printf(UB *format, ...) {
    (void)format;
    assert(!depth);
}
uint64_t board_time_us(void) {
    return now;
}
uint32_t board_kernel_ms(void) {
    return (uint32_t)(now / 1000);
}
void ri_tilt_snapshot(b4v23_tilt_t *out) {
    assert(!depth);
    if (publish_while_copying) {
        /* Core0 publishes after core1 cached the control-frame timestamp. */
        now += 1000;
        ++tilt.seq;
        tilt.t_us = now;
        publish_while_copying = false;
    }
    *out = tilt;
}
bool rn_drive_telemetry_enabled(void) {
    return true;
}
bool rn_drive_online(void) {
    return true;
}
uint8_t rn_next_sequence(unsigned source) {
    static uint8_t seq;
    assert(source == 15);
    return seq++;
}
bool rn_tx_push(uint8_t topic, const uint8_t *data, uint8_t len, uint32_t stamp) {
    assert(!depth && topic == 0 && stamp == board_kernel_ms());
    assert(data[0] == 0xF2 && data[1] == len - 3 && (len == 31 || len == 24));
    ++packets;
    return true;
}
#include "app_program/imu_navigation/ramp_bridge.c"
#include "app_program/line_navigation/line_model.h"
static uint64_t clock_now(void *u) {
    (void)u;
    return now;
}
static void follow_tick(LinePlanner *planner, LinePower *power, CircuitInput *in,
                         AnalogDemand *demand, bool fresh) {
    now += 10000;
    in->now = now;
    if (fresh) {
        ++tilt.seq;
        tilt.t_us = now;
    }
    assert(motion_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
    MotionFeedback feedback = {
        .now_us = now, .speed_mm_s = {45, 45}, .line_level = {1000, 1000}};
    (void)motion_update(&feedback);
    LineEnvelope out = rl_power(power, planner, in, demand, C_FOLLOW, .01f);
    assert(out.continuous);
    for (unsigned w = 0; w < 2; ++w) {
        assert(out.output[w] >= 0 && out.output[w] <= 420);
        power->output[w] = out.output[w];
    }
}
int main(void) {
    MotionPort port = {.now_us = clock_now};
    motion_init(&port);
    motion_session_begin(now);
    LinePlanner planner;
    LinePower power = {0};
    CircuitInput in = {.now = now, .adc = {246, 267}, .speed = {45, 45}};
    rl_begin(&planner, &in, 250);
    AnalogDemand demand = {.target = {45, 45}, .level = {1000, 1000}, .decision = SYM_STRAIGHT};
    tilt = (b4v23_tilt_t){.valid = true, .tracking = true, .height_mm = 8};
    for (unsigned k = 0; k < 80; ++k)
        follow_tick(&planner, &power, &in, &demand, k % 2 == 0);
    MotionRampAssist approved = nav_get_ramp_assist();
    assert(approved.boost_milli == 200 && approved.hold_milli == 1000 && packets >= 2);
    assert(power.drive[0].hold_i_max == 300 && motion_get_status().owner == MOTION_BUDDY3);
    /* A fresh sample arriving while snapshotting must not end/restart
     * assistance, reset speed hold, or add unsigned elapsed-time credit. */
    uint8_t activations = state.activations;
    for (unsigned k = 0; k < 12; ++k) {
        publish_while_copying = true;
        follow_tick(&planner, &power, &in, &demand, false);
        approved = nav_get_ramp_assist();
        assert(!publish_while_copying && state.assisting && state.hold_on);
        assert(state.activations == activations && state.last_end != RA_END_STALE);
        assert(state.last_sample_us > in.now && state.last_sample_us <= board_time_us());
        assert(approved.boost_milli == 200 && approved.hold_milli == 1000);
        assert(state.episode_us < 2000000U);
    }
    /* Genuine stopped acquisition still obeys the exact60ms/200ms budgets. */
    uint64_t last_sample = tilt.t_us;
    const float speed[2] = {45, 45};
    const int16_t target[2] = {45, 45};
    now = last_sample + RA_STALE_US;
    (void)ri_ramp_step(.01f, true, speed, target, power.drive);
    assert(state.assisting && state.hold_on);
    now++;
    (void)ri_ramp_step(.01f, true, speed, target, power.drive);
    assert(!state.assisting && state.last_end == RA_END_STALE && state.hold_on);
    now = last_sample + RA_HOLD_STALE_US;
    (void)ri_ramp_step(.01f, true, speed, target, power.drive);
    assert(state.hold_on);
    now++;
    (void)ri_ramp_step(.01f, true, speed, target, power.drive);
    assert(!state.hold_on);

    /* A new session crossing the low32-bit timer boundary keeps full64-bit
     * sample times and ordinary activation. No epoch truncation is allowed. */
    ri_ramp_halt(now);
    motion_session_end(0);
    now = (UINT64_C(1) << 32) - 50000U;
    motion_session_begin(now);
    in.now = now;
    rl_begin(&planner, &in, 250);
    power = (LinePower){0};
    demand = (AnalogDemand){.target={45,45},.level={1000,1000},.decision=SYM_STRAIGHT};
    for (unsigned k = 0; k < 80; ++k)
        follow_tick(&planner, &power, &in, &demand, k % 2 == 0);
    assert(now > UINT32_MAX && state.last_sample_us > UINT32_MAX);
    assert(state.assisting && state.activations == 1 && state.last_end != RA_END_STALE);
    approved = nav_get_ramp_assist();
    assert(approved.boost_milli == 200 && approved.hold_milli == 1000);
    planner.circuit.busy = planner.nav.busy = true;
    planner.circuit.phase = J_TURN;
    planner.nav.action = 'F';
    planner.nav.executing_action = 'L';
    demand.target[0] = 0;
    demand.target[1] = 45;
    now += 10000;
    in.now = now;
    LineEnvelope out = rl_power(&power, &planner, &in, &demand, C_FOLLOW, .01f);
    assert(!out.continuous && !nav_get_ramp_assist().boost_milli);
    assert(!power.drive[0].boost_frac && !power.drive[1].hold_min_speed);
    ri_ramp_halt(now);
    ri_ramp_report();
    assert(!nav_get_ramp_assist().hold_milli && !depth);
    motion_session_end(0);
    puts(
        "PASS actual RTOS ramp bridge/controller/coordinator: newer-sample race, genuine stale budgets, 64-bit clock boundary, approved profile, caps, shared-source packets, no I/O under copy lock, junction/halt removal");
    return 0;
}
