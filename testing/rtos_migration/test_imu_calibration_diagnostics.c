/* Real service + processor, deterministic FIFO. No hardware or threshold changes. */
#define APP_RTOS_IMU 1
#include <assert.h>
#include <stdio.h>
#include "src/imu/imu_service.h"
#include "app_program/imu_navigation/calibration_diagnostics.h"
uint32_t host_now_us = 1000000;
uint64_t board_time_us(void) {
    return host_now_us;
}
extern int fake_lat_mg, fake_fwd_mg, fake_up_mg;
extern bool fake_ok;
void ri_acquisition_record(uint32_t gap, bool ok, bool overrun) {
    (void)gap;
    (void)ok;
    (void)overrun;
}
static bool send_frame(const uint8_t *f, uint32_t n) {
    (void)f;
    (void)n;
    return true;
}
static void tick(const b4v23_odo_t *odo) {
    host_now_us += 20000;
    b4v23_tick(host_now_us, B4V23_NONE, odo);
}
static RiCalibration view(void) {
    RiCalibration d;
    b4v23_calibration(&d);
    return d;
}
static void init(void) {
    fake_ok = true;
    fake_lat_mg = fake_fwd_mg = 0;
    fake_up_mg = 1000;
    b4v23_init(send_frame, false);
}
int main(void) {
    init();
    fake_ok = false;
    tick(NULL);
    assert(view().failed_reads == 1 && view().progress == 0);
    fake_ok = true;
    for (unsigned i = 0; i < 110; ++i)
        tick(NULL);
    RiCalibration d = view();
    assert(d.state == IMU_ST_IDLE && !d.fault && d.progress == 100 && d.best == 100);
    assert(!d.restarts && d.failed_reads == 1);

    init();
    for (unsigned i = 0; i < 510; ++i) {
        fake_lat_mg = (i & 1U) ? 120 : 0;
        tick(NULL);
    }
    d = view();
    assert(d.state == IMU_ST_FAULT && d.fault == IMU_FAULT_CAL_TIMEOUT);
    assert(d.restarts > 400 && d.axis_restarts[1] > 400 && !d.speed_restarts);
    assert((d.last_reason & RI_CAL_LAT) && d.last_span_mg[1] > B4_CAL_STABLE_MG);
    assert(d.best < 100 && d.age_ms == 10000 && !d.failed_reads);
    uint32_t saved = d.restarts;
    tick(NULL);
    assert(view().restarts == saved); /* evidence retained after fault */

    init();
    assert(!view().restarts && !view().best); /* new calibration epoch */
    b4v23_odo_t odo = {.have = true, .clean = true};
    for (unsigned i = 0; i < 80; ++i) {
        odo.centre_mm += 2.0; /* 100mm/s: exact existing stationary guard */
        tick(&odo);
    }
    d = view();
    assert(d.speed_restarts > 0 && (d.last_reason & RI_CAL_SPEED));
    assert(!d.axis_restarts[0] && !d.axis_restarts[1] && !d.axis_restarts[2]);

    init();
    for (unsigned i = 0; i < 110; ++i)
        tick(NULL);
    assert(view().state == IMU_ST_IDLE && !view().fault && !view().restarts);
    puts(
        "PASS real IMU calibration diagnostics: read interruption,100-period success, exact axis/speed restarts,10s timeout evidence retained, fresh reset recovery.");
}
