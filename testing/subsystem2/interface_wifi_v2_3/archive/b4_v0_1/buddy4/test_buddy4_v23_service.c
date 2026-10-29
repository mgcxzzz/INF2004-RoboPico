/* Host test of the v2.3 Buddy4 add-on service: real imu_proc/b4_pack/b2_pack
 * through buddy4_service.c, fake LSM303 FIFO, synthetic v2.3 odometry view.
 * Checks calibration, mission mapping, ramp acceptance through the service,
 * odometry-quality gating (unclean intervals), fault + Reset recovery, and
 * that emitted frames unpack with the RTOS decoders. Frames are also written
 * as hex for the JS dashboard decoder. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../buddy4_v23.h"
#include "../subsystem4/imu_iface.h"
#include "../system/sys_tlm.h"
uint32_t host_now_us = 1000000u;
extern int fake_lat_mg, fake_up_mg, fake_fwd_mg; extern bool fake_ok;
static FILE *hexf;
static imu_tlm_t last4; static b2_tlm_t last2; static unsigned n4, n2, bad;
static bool send(const uint8_t *f, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) { fprintf(hexf, "%02x", f[i]); }
    fputc('\n', hexf);
    if (n == IMU_TLM_FRAME_BYTES) { if (imu_tlm_unpack(f, n, &last4)) n4++; else bad++; }
    else if (n == B2_TLM_FRAME_BYTES) { if (b2_tlm_unpack(f, n, &last2)) n2++; else bad++; }
    else bad++;
    return true;
}
static b4v23_odo_t odo = { .have = true, .clean = true };
static void run_ms(uint32_t ms, b4v23_mission_t first, double v_mm_s) {
    b4v23_mission_t m = first;
    for (uint32_t t = 0; t < ms; t += 20) {
        host_now_us += 20000u;
        odo.centre_mm += v_mm_s * 0.020; odo.left_mm_s = odo.right_mm_s = (float)v_mm_s; odo.age_ms = 5;
        assert(b4v23_due(host_now_us));
        b4v23_tick(host_now_us, m, &odo); m = B4V23_NONE;
    }
}
static void tilt_mm(double h) {
    double s = h / 113.0; fake_lat_mg = (int)lround(1000.0 * s); fake_up_mg = (int)lround(1000.0 * sqrt(1.0 - s * s));
}
int main(void) {
    hexf = fopen("buddy4_v23_frames.hex", "w"); assert(hexf);
    setvbuf(stdout, NULL, _IONBF, 0); setvbuf(hexf, NULL, _IOLBF, 0);
    b4v23_init(send, false);
    run_ms(3000, B4V23_NONE, 0);                       /* still: calibrate */
    printf("boot: state=%u fault=%u n4=%u n2=%u bad=%u\n", last4.state, last4.fault, n4, n2, bad);
    assert(last4.state == IMU_ST_IDLE && bad == 0);
    run_ms(400, B4V23_START, 0);
    assert(last4.state == IMU_ST_TRACKING);
    run_ms(600, B4V23_NONE, 45);                       /* flat, moving */
    tilt_mm(10); run_ms(500, B4V23_NONE, 45);          /* right wheel up 10 mm */
    tilt_mm(0);  run_ms(800, B4V23_NONE, 45);          /* back to flat */
    printf("ramp: humps=%u max_h=%u rejected=%u side_h=%d\n", last4.humps, last4.max_height_mm, last4.rejected, last2.side_height_mm);
    assert(last4.humps == 1 && last4.max_height_mm >= 9 && last4.max_height_mm <= 11);
    assert(last2.distance_mm > 0 && (last2.flags & B2_TLM_ODO_VALID) && last2.left_mm_s == 45);
    /* unclean odometry (v2.3 invalid transition): no new hump accepted */
    odo.clean = false; tilt_mm(10); run_ms(500, B4V23_NONE, 45); tilt_mm(0); run_ms(800, B4V23_NONE, 45);
    assert(last4.humps == 1);
    assert(!(last2.flags & B2_TLM_ODO_VALID));
    odo.clean = true;
    run_ms(200, B4V23_STOP, 0);
    assert(last4.state == IMU_ST_IDLE);
    /* Resume semantics are the harness's; STOP->RESUME keeps the run counters */
    run_ms(200, B4V23_RESUME, 0);
    assert(last4.state == IMU_ST_TRACKING && last4.humps == 1);
    run_ms(200, B4V23_STOP, 0);
    /* sensor fault, then accepted Reset recovers (recalibrates while still) */
    fake_ok = false; run_ms(2000, B4V23_NONE, 0);
    assert(last4.state == IMU_ST_FAULT);
    fake_ok = true; run_ms(3000, B4V23_RESET, 0);
    printf("after reset: state=%u fault=%u\n", last4.state, last4.fault);
    assert(last4.state == IMU_ST_IDLE);
    assert(bad == 0 && n4 > 10 && n2 > 10);
    printf("max tick (host) %u us; frames: src4 %u, src2 %u\n", b4v23_max_tick_us(), n4, n2);
    fclose(hexf);
    puts("PASS buddy4 v2.3 service: calibrate, start/stop/resume/reset mapping, 10 mm ramp accepted, unclean odometry gated, fault+reset recovery, frames unpack");
    return 0;
}
