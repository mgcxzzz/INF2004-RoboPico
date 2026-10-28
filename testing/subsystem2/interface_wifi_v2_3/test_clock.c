#include <assert.h>
#include <stdio.h>
#include "nav_api.h"
#include "motion_port.h"
static uint64_t clock_us;
static unsigned off_count;
static uint64_t now(void *p) {(void)p;return clock_us;}
static void off(void *p) {(void)p;++off_count;}
static void begin(MotionFeedback *f) {
    clock_us=1000;off_count=0;MotionPort p={.now_us=now,.outputs_off=off};motion_init(&p);motion_session_begin(clock_us);
    *f=(MotionFeedback){.now_us=clock_us,.line_level={700,700}};
}
static void fresh_after_snapshot(void) {
    const unsigned lag[]={1,2,7,50,500,1000,5000};
    for(unsigned k=0;k<sizeof(lag)/sizeof(lag[0]);++k) {
        MotionFeedback f;begin(&f);clock_us=f.now_us+lag[k];
        assert(nav_submit_navigation(MOTION_BUDDY3,45,45)==MOTION_OK);
        MotionOutput out=motion_update(&f);MotionStatus s=nav_get_status();
        if(s.mode!=MOTION_AUTO)fprintf(stderr,"REGRESSION: request newer than snapshot by%uus causes mode=%u fault=0x%x zero targets=%d,%d\n",lag[k],s.mode,s.fault,out.target_mm_s[0],out.target_mm_s[1]);
        assert(s.mode==MOTION_AUTO && !s.fault && !out.external && out.target_mm_s[0]==45 && out.target_mm_s[1]==45 && !off_count);
        for(unsigned tick=0;tick<2000;++tick) {
            f.now_us+=10000;clock_us=f.now_us+lag[k];
            assert(nav_submit_navigation(MOTION_BUDDY3,45,45)==MOTION_OK);
            out=motion_update(&f);assert(nav_get_status().mode==MOTION_AUTO && out.target_mm_s[0]==45 && !off_count);
        }
        uint64_t last=clock_us;
        clock_us=last+100000;f.now_us=clock_us-50;out=motion_update(&f);assert(nav_get_status().mode==MOTION_AUTO);
        clock_us=last+100001;out=motion_update(&f);
        assert(nav_get_status().mode==MOTION_FAULT && (nav_get_status().fault&NAV_FAULT_CONTROL));
        assert(!out.target_mm_s[0] && !out.target_mm_s[1] && off_count);
    }
}
static void takeover_and_handback(void) {
    MotionFeedback f;begin(&f);clock_us=f.now_us+50;uint32_t lease;
    assert(nav_request_takeover(MOTION_BUDDY5,&lease)==MOTION_OK);
    MotionOutput out=motion_update(&f);
    assert(nav_get_status().mode==MOTION_STOPPING && !nav_get_status().fault && out.external);
    for(unsigned k=0;k<17;++k) {f.now_us+=10000;clock_us=f.now_us+50;out=motion_update(&f);}
    assert(nav_get_status().mode==MOTION_PAUSED && nav_get_status().stopped);
    assert(nav_resume_line(MOTION_BUDDY5,lease,false)==MOTION_OK);
    out=motion_update(&f); /* same snapshot is older than the accepted resume */
    assert(out.resume_navigation && nav_get_status().mode==MOTION_AUTO && !nav_get_status().fault);
    /* A real failure to settle still faults using monotonic request age. */
    begin(&f);clock_us+=50;assert(nav_request_takeover(MOTION_BUDDY5,&lease)==MOTION_OK);
    f.speed_mm_s[0]=20;clock_us+=2000000;f.now_us=clock_us-50;out=motion_update(&f);
    assert(nav_get_status().mode==MOTION_FAULT && (nav_get_status().fault&NAV_FAULT_SETTLE));
    (void)out;
}
int main(void) {
    fresh_after_snapshot();takeover_and_handback();
    puts("PASS real clock ordering:1..5000us newer requests remain AUTO for14000 ticks; stale100ms heartbeat and2s settling still fault; delayed takeover/resume never underflow.");
}
