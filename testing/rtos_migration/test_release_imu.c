#define APP_RTOS_RELEASE 1
#define main stationary_reset_fixture
#include "test_imu_task_reset.c"
#undef main

static void publish_newer_sample_while_reader_waits(void) {
    host_now_us=92679000U;
    g_shared.lift_sample_ms=92679U;
    g_shared.lift_have=true;
}

int main(void) {
    stationary_reset_fixture();
    printf("guard boot: blocked=%u reason=%u reset=%u armed=%u valid=%u upright=%u last=%u now=%u pending=%u\n",
        ri_safety_blocked(),ri_safety_reason(),ri_safety_can_reset(),g_shared.lift.armed,
        g_shared.lift.current_valid,g_shared.lift.upright_samples,g_shared.lift_sample_ms,
        host_now_us/1000,g_shared.lift_reset_pending);
    assert(!ri_safety_blocked() && ri_safety_reason()==0U && ri_safety_can_reset());
    uint32_t task_clock_us=host_now_us, task_sample_ms=g_shared.lift_sample_ms;
    /* A writer publishes a sample 6 ms newer after the reader has captured its
     * lock request. Clock-after-copy keeps it fresh without extending 100 ms. */
    g_shared.lift.latched=false;
    g_shared.lift.armed=true;
    g_shared.lift.current_valid=true;
    g_shared.lift.upright_samples=IMU_LIFT_UPRIGHT_SAMPLES;
    g_shared.lift_reset_pending=false;
    g_shared.lift_have=true;
    host_now_us=92673000U;
    g_shared.lift_sample_ms=92673U;
    before_lock_hook=publish_newer_sample_while_reader_waits;
    assert(!ri_safety_blocked());
    assert(ri_safety_reason()==0U);
    host_now_us=92673000U;
    g_shared.lift_sample_ms=92673U;
    before_lock_hook=publish_newer_sample_while_reader_waits;
    assert(ri_safety_can_reset());
    uint8_t raced[30];
    host_now_us=92673000U;
    g_shared.lift_sample_ms=92673U;
    before_lock_hook=publish_newer_sample_while_reader_waits;
    ri_safety_packet(16,22,raced);
    assert(raced[23]&0x10);
    assert(host_now_us/1000U==g_shared.lift_sample_ms);
    /* Restore the real task fixture's timeline before exercising its FIFO
     * acquisition; the independent lock-race case used a later clock epoch. */
    host_now_us=task_clock_us;
    g_shared.lift_sample_ms=task_sample_ms;
    const int16_t base[3]={0,0,1000};
    imu_lift_vec_t level[8];
    for(unsigned i=0;i<8;++i) {
        level[i]=(imu_lift_vec_t){.up_mg=1000};
    }
    extern int fake_up_mg;
    fake_up_mg=2600; /* large enough to exercise the old shock classifier */
    /* The task fixture stopped at its next release, already20ms after last FIFO. */
    b4v23_tick(host_now_us,B4V23_NONE,NULL);
    assert(!g_shared.lift.latched); /* first 20 ms alone cannot qualify */
    host_now_us+=20000;
    b4v23_tick(host_now_us,B4V23_NONE,NULL);
    ri_calibration_t processing;
    b4v23_calibration(&processing);
    assert(processing.state!=IMU_ST_IMPACT); /* removed collision transition */
    fake_up_mg=1000;
    assert(g_shared.lift.latched && ri_safety_blocked() && ri_safety_reason()==1U);
    assert(!ri_safety_can_reset() && !ri_safety_reset_latch());
    ri_reset_request(); /* unauthorised raw reset cannot erase a trip */
    assert(!g_shared.reset && !g_shared.lift_reset_pending);
    for(unsigned n=0;n<25;++n) {
        host_now_us+=20000;
        ri_lift_guard_samples(base,1000,level,8,2500,true);
    }
    assert(ri_safety_can_reset() && g_shared.lift.latched);
    uint8_t safety[30];
    host_kernel_offset_ms=137; /* hardware timer starts before kernel uptime */
    ri_safety_packet(17,22,safety);
    assert(safety[0]==0xf2 && safety[3]==0xc9 && safety[4]==0x25 &&
           (safety[23]&0x3f)==0x3d);
    uint32_t packet_ms=0;
    for(unsigned b=0;b<4;++b) packet_ms|=(uint32_t)safety[5+b]<<(8*b);
    assert(packet_ms==host_now_us/1000 && packet_ms!=board_kernel_ms());
    /* Both offset directions must leave C9 consistent with the actual guard. */
    host_kernel_offset_ms=(uint32_t)-137;
    ri_safety_packet(18,22,safety);
    assert((safety[23]&0x3f)==0x3d && ri_safety_blocked());
    uint8_t peak[RI_LIFT_PEAK_PACKET_BYTES];
    ri_safety_peak_packet(19,peak);
    assert(peak[0]==0xf2 && peak[1]==25 && peak[2]==19 && peak[3]==0xca && peak[4]==1);
    uint32_t peak_report_ms=0;
    for(unsigned b=0;b<4;++b) peak_report_ms|=(uint32_t)peak[5+b]<<(8*b);
    assert(peak_report_ms==host_now_us/1000 && peak_report_ms!=board_kernel_ms());
    assert((peak[21]&0x70)==0x70 && peak[26]==1 && peak[27]==0);
    assert(peak[19]<=100 && peak[20]<=30 && !(peak[19]&0x80) && !(peak[20]&0xe0));
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
    assert(!g_shared.reset && g_shared.lift_reset_pending && ri_safety_blocked());
    ri_reset_request(); /* the owner queues exactly one actual recalibration */
    assert(g_shared.reset && g_shared.lift_reset_pending && ri_safety_blocked());
    assert(ri_safety_reason()==3U && !ri_safety_reset_latch());
    puts("PASS release IMU guard: stopped acquisition, lift latch, raw-reset bypass refusal, fresh/upright recheck and blocked recalibration handoff");
    return 0;
}
