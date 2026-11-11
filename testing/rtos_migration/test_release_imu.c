#define APP_RTOS_RELEASE 1
#define main stationary_reset_fixture
#include "test_imu_task_reset.c"
#undef main

int main(void) {
    stationary_reset_fixture();
    printf("guard boot: blocked=%u reason=%u reset=%u armed=%u valid=%u upright=%u last=%u now=%u pending=%u\n",
        ri_safety_blocked(),ri_safety_reason(),ri_safety_can_reset(),shared.lift.armed,
        shared.lift.current_valid,shared.lift.upright_samples,shared.lift_sample_ms,
        host_now_us/1000,shared.lift_reset_pending);
    assert(!ri_safety_blocked() && ri_safety_reason()==0U && ri_safety_can_reset());
    const int16_t base[3]={0,0,1000};
    imu_lift_vec_t level[8];
    for(unsigned i=0;i<8;++i) {
        level[i]=(imu_lift_vec_t){.up_mg=1000};
    }
    extern int fake_up_mg;
    fake_up_mg=2600; /* large enough to exercise the old shock classifier */
    /* The task fixture stopped at its next release, already20ms after last FIFO. */
    b4v23_tick(host_now_us,B4V23_NONE,NULL);
    assert(!shared.lift.latched); /* first 20 ms alone cannot qualify */
    host_now_us+=20000;
    b4v23_tick(host_now_us,B4V23_NONE,NULL);
    RiCalibration processing;
    b4v23_calibration(&processing);
    assert(processing.state!=IMU_ST_IMPACT); /* removed collision transition */
    fake_up_mg=1000;
    assert(shared.lift.latched && ri_safety_blocked() && ri_safety_reason()==1U);
    assert(!ri_safety_can_reset() && !ri_safety_reset_latch());
    ri_reset_request(); /* unauthorised raw reset cannot erase a trip */
    assert(!shared.reset && !shared.lift_reset_pending);
    for(unsigned n=0;n<25;++n) {
        host_now_us+=20000;
        ri_lift_guard_samples(base,1000,level,8,2500,true);
    }
    assert(ri_safety_can_reset() && shared.lift.latched);
    uint8_t safety[30];
    ri_safety_packet(17,host_now_us/1000,22,safety);
    assert(safety[0]==0xf2 && safety[3]==0xc9 && (safety[23]&0x3f)==0x3d);
    /* A last-good age just outside the limit must invalidate an earlier Reset check. */
    host_now_us+=101000;
    assert(!ri_safety_can_reset() && !ri_safety_reset_latch());
    ri_lift_guard_samples(base,1000,level,8,2500,true);
    assert(!ri_safety_reset_latch()); /* old upright dwell cannot cross the gap */
    for(unsigned n=0;n<25;++n) {
        host_now_us+=20000;
        ri_lift_guard_samples(base,1000,level,8,2500,true);
    }
    assert(ri_safety_reset_latch());
    assert(!shared.reset && shared.lift_reset_pending && ri_safety_blocked());
    ri_reset_request(); /* the owner queues exactly one actual recalibration */
    assert(shared.reset && shared.lift_reset_pending && ri_safety_blocked());
    assert(ri_safety_reason()==3U && !ri_safety_reset_latch());
    puts("PASS release IMU guard: stopped acquisition, lift latch, raw-reset bypass refusal, fresh/upright recheck and blocked recalibration handoff");
    return 0;
}
