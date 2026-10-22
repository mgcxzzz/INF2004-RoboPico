#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hump_record.h"

static void baseline_equivalence(void) {
    for(unsigned w=0;w<2;++w) for(unsigned mode=0;mode<=1;++mode) {
        HumpWheel p={0};WheelPI ref={0};LineBreakaway kick={0};
        for(unsigned n=0;n<600;++n) {
            int16_t wanted=n<100?45:n<200?20:n<300?0:n<400?50:n<500?30:45;
            float speed=n%80<20?0:wanted*0.9f;uint64_t now=1000+n*10000;
            int32_t expected=line_speed_step(&ref,w,wanted,speed,0.01f);
            expected=line_breakaway(&kick,now,wanted,ref.ramp,speed,expected);
            int32_t actual=hump_wheel_step(&p,w,wanted,speed,0.01f,now,mode,false);
            assert(actual==effort_scale20(expected) && p.extra==0);
            DualLevels a=hump_levels(w,actual,mode),b=dual_levels(w,expected);
            assert(a.a==effort_scale20(b.a) && a.b==effort_scale20(b.b));
        }
    }
}
static void requested_slowdown_is_not_load(void) {
    HumpWheel p={.pi={.ramp=45,.integral=60}};
    for(unsigned n=0;n<100;++n) {
        /* The wheel follows a requested slowdown. No load grant on that basis. */
        int16_t target=n<30?30:20;float speed=n<15?45.0f-(float)n:(float)target;
        (void)hump_wheel_step(&p,0,target,speed,0.01f,1000+n*10000,HUMP_ASSIST,true);
        assert(p.extra==0 && !p.qualifying);
    }
    p=(HumpWheel){0};
    for(unsigned n=0;n<200;++n) {
        (void)hump_wheel_step(&p,0,45,45,0.01f,1000+n*10000,HUMP_ASSIST,true);
        assert(p.extra==0); /* healthy target45 wheel */
    }
}
static void load_assistance_and_removal(void) {
    HumpWheel left={.pi={.ramp=45,.integral=60}},right=left;
    bool raised=false;uint64_t first=0;
    for(unsigned n=0;n<50;++n) {
        uint64_t now=1000000+n*10000;
        int32_t l=hump_wheel_step(&left,0,45,45,0.01f,now,HUMP_ASSIST,true);
        int32_t r=hump_wheel_step(&right,1,45,0,0.01f,now,HUMP_ASSIST,true);
        assert(l<=300 && left.extra==0 && r<=420 && right.extra<=150);
        if(n<10) assert(right.extra==0);
        if(r>300 && !raised) { raised=true;first=now; }
        DualLevels bridge=hump_levels(1,r,HUMP_ASSIST);
        assert(bridge.b==r && bridge.a==0); /* no hidden shared25% clamp */
    }
    assert(raised && first<1400000 && right.extra==150);
    /* Assistance has time to rise before the retained500ms blocked-wheel stop.
       This verifies controller effort, not that a physical ramp is climbable. */
    (void)hump_wheel_step(&right,1,45,48,0.01f,1510000,HUMP_ASSIST,true);
    assert(right.extra==0 && !right.qualifying); /* overspeed clears immediately */
    right.extra=150;
    for(unsigned n=0;n<30;++n) (void)hump_wheel_step(&right,1,45,45,0.01f,1600000+n*10000,HUMP_ASSIST,true);
    assert(right.extra==0); /* recovered speed removes extra over300ms */
    right.extra=150;
    int32_t u=hump_wheel_step(&right,1,45,0,0.01f,2000000,HUMP_ASSIST,false);
    assert(right.extra==0 && u<=300); /* no extra during optical loss recovery */
    right.extra=150;
    assert(hump_wheel_step(&right,1,0,0,0.01f,2010000,HUMP_ASSIST,true)==0);
    assert(right.pi.ramp==0 && right.pi.integral==0 && !right.kick.active && right.extra==0);
    for(unsigned mode=0;mode<=1;++mode) for(unsigned w=0;w<2;++w) for(int pwm=-1000;pwm<=1000;++pwm) {
        DualLevels v=hump_levels(w,pwm,mode);
        assert(!(v.a && v.b) && v.a<=hump_ceiling(mode) && v.b<=hump_ceiling(mode));
        if(pwm<=0) assert(v.a==0 && v.b==0);
    }
}
static void slow_uphill_wheel_regression(void) {
    /* Reproduce V1's near-crest right target32 ->20 while encoders stop.
       Also exercise a settled20 request, both wheel channels, and the19 floor. */
    for(unsigned w=0;w<2;++w) for(unsigned transition=0;transition<2;++transition) {
        HumpWheel stalled={.pi={.ramp=transition?32:20,.integral=60}};
        HumpWheel healthy={.pi={.ramp=49,.integral=60}};
        bool raised=false;
        for(unsigned n=0;n<50;++n) {
            uint64_t now=1000000+n*10000;
            int32_t u=hump_wheel_step(&stalled,w,20,0,0.01f,now,HUMP_ASSIST,true);
            (void)hump_wheel_step(&healthy,1-w,49,49,0.01f,now,HUMP_ASSIST,true);
            assert(u<=420 && stalled.extra<=150 && healthy.extra==0);
            if(n<10) assert(stalled.extra==0);
            if(u>300) raised=true;
        }
        assert(raised && stalled.extra==150); /* before500ms guard expires */
        int32_t u=hump_wheel_step(&stalled,w,19,0,0.01f,1510000,HUMP_ASSIST,true);
        assert(u<=300 && stalled.extra==0 && !stalled.qualifying);
        assert(hump_wheel_step(&stalled,w,0,0,0.01f,1520000,HUMP_ASSIST,true)==0);
    }
}
static const LineCalibration cal={{184,184},{3464,3692},7};
static void storage_and_import(void) {
    HumpSavedLog v={.magic=HUMP_MAGIC,.version=1,.calibration=cal,.profile=HUMP_ASSIST};
    v.crc=log_crc((const uint8_t *)&v+offsetof(HumpSavedLog,count),sizeof(v)-offsetof(HumpSavedLog,count));
    assert(hump_record_valid(&v) && hump_record_firmware(&v)==1); /* V1 zero padding */
    v.firmware=2;
    assert(!hump_record_valid(&v)); /* revision is CRC protected */
    v.crc=log_crc((const uint8_t *)&v+offsetof(HumpSavedLog,count),sizeof(v)-offsetof(HumpSavedLog,count));
    assert(hump_record_valid(&v) && hump_record_firmware(&v)==2);
    v.firmware=3;
    v.crc=log_crc((const uint8_t *)&v+offsetof(HumpSavedLog,count),sizeof(v)-offsetof(HumpSavedLog,count));
    assert(hump_record_valid(&v) && hump_record_firmware(&v)==3);
    v.firmware=4;
    v.crc=log_crc((const uint8_t *)&v+offsetof(HumpSavedLog,count),sizeof(v)-offsetof(HumpSavedLog,count));
    assert(hump_record_valid(&v) && hump_record_firmware(&v)==4);
    v.firmware=5;
    v.crc=log_crc((const uint8_t *)&v+offsetof(HumpSavedLog,count),sizeof(v)-offsetof(HumpSavedLog,count));
    assert(!hump_record_valid(&v)); /* unsupported future revision */
    v.firmware=2;
    v.crc=log_crc((const uint8_t *)&v+offsetof(HumpSavedLog,count),sizeof(v)-offsetof(HumpSavedLog,count));
    v.calibration.black[0]++;assert(!hump_record_valid(&v));
    HumpLegacyJunction old={.magic=0x314e434au,.version=1,.firmware=9,.calibration=cal,
        .mode=3,.selected_mode=3,.side={0,1,3},.count=15};
    old.crc=log_crc((const uint8_t *)&old+offsetof(HumpLegacyJunction,count),sizeof(old)-offsetof(HumpLegacyJunction,count));
    LineCalibration c={0};assert(hump_import_junction(&old,&c) && memcmp(&c,&cal,sizeof(c))==0);
    old.firmware=10;
    old.crc=log_crc((const uint8_t *)&old+offsetof(HumpLegacyJunction,count),sizeof(old)-offsetof(HumpLegacyJunction,count));
    assert(hump_import_junction(&old,&c));
    old.rows[12]++;assert(!hump_import_junction(&old,&c));old.rows[12]--;
    uint8_t flash[8192],final_before[4096];memset(flash,0xff,sizeof(flash));
    memcpy(flash+4096,&old,sizeof(old));memcpy(final_before,flash+4096,4096);
    memcpy(flash,&v,sizeof(v));assert(memcmp(final_before,flash+4096,4096)==0);
    HumpLegacyLine line={.magic=0x31464c41u,.version=1,.profile=5,.reserved=6,.calibration=cal};
    line.crc=log_crc((const uint8_t *)&line+offsetof(HumpLegacyLine,count),sizeof(line)-offsetof(HumpLegacyLine,count));
    assert(hump_import_line(&line,&c) && memcmp(&c,&cal,sizeof(c))==0);
    line.count=101;assert(!hump_import_line(&line,&c));
}
static void optical_and_stop_guards(void) {
    SymmetricTracker s={0};AnalogDemand d=symmetric_demand(&s,&cal,3464,3692,1000,0);
    assert(!d.stop && d.target[0]==45 && d.target[1]==45);
    d=symmetric_demand(&s,&cal,184,184,11000,1);assert(d.stop);
    DualGuard g={0};assert(guard_start(&g,1000000,true,false));g.deadline_us=11000000;
    guard_tick(&g,1001000,true,true);assert(!g.active && g.reason==WHY_GP20);
    g=(DualGuard){0};assert(guard_start(&g,1000000,true,false));
    guard_tick(&g,1001000,false,false);assert(!g.active && g.reason==WHY_RELEASE);
    g=(DualGuard){0};assert(guard_start(&g,1000000,true,false));
    guard_tick(&g,1100000,true,false);assert(!g.active && g.reason==WHY_LATE);
    assert(profile_start_block(5,false,false,true,false,true,1000,1000,false,LINE_BANK)==START_OK);
    assert(profile_start_block(5,false,false,true,false,true,0,0,false,LINE_BANK)==START_LINE_ABSENT);
}
int main(void) {
    baseline_equivalence();requested_slowdown_is_not_load();load_assistance_and_removal();
    slow_uphill_wheel_regression();
    storage_and_import();optical_and_stop_guards();
    puts("PASS: baseline/V11 equivalence, intended slowdown, stalled20/32->20 regression, scaled30%/42% assistance/removal, V1/V2/V3/V4 record/import/guards");
    return 0;
}
