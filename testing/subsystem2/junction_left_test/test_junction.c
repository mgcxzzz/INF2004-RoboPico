#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "junction_logic.h"
#include "signed_wheel.h"
#include "../line_motion_integration/logic.h"

static const LineCalibration cal={{184,184},{3464,3692},7};
static JunctionInput sample(uint64_t now,float mean,bool side) {
    JunctionInput i={.now=now,.wheel={mean,mean},.adc={3464,3692},.side_black=side};
    return i;
}
static Junction centre_fixture(void) {
    Junction j;JunctionInput i=sample(1000,0,false);j_begin(&j,true,&i);
    (void)junction_step(&j,&cal,&i);i=sample(101000,4,false);(void)junction_step(&j,&cal,&i);
    assert(j.side_armed);
    i=sample(200000,100,true);(void)junction_step(&j,&cal,&i);
    i=sample(220000,101,true);(void)junction_step(&j,&cal,&i);
    assert(j.phase==J_CENTRE && fabsf(j.leading_mm-100)<0.01f);
    i=sample(800000,118,false);(void)junction_step(&j,&cal,&i);
    i=sample(820000,119,false);(void)junction_step(&j,&cal,&i);
    assert(j.striped && fabsf(j.centre_mm-201)<0.01f);
    return j;
}
static void test_geometry_and_qualification(void) {
    Junction j=centre_fixture();JunctionInput i=sample(1000000,199,false);
    AnalogDemand d=junction_step(&j,&cal,&i);assert(j.phase==J_CENTRE && d.target[0]>0);
    i=sample(1010000,200,false);d=junction_step(&j,&cal,&i);
    assert(j.phase==J_SETTLE && d.target[0]==0 && d.target[1]==0);
    i=sample(1020000,200,false);(void)junction_step(&j,&cal,&i);
    i=sample(1310000,200,false);d=junction_step(&j,&cal,&i);
    assert(j.phase==J_TURN && d.target[0]==0 && d.target[1]==0);
    i.now+=10000;d=junction_step(&j,&cal,&i);assert(d.target[0]<0 && d.target[1]>0);
    /* Old-line black cannot immediately finish the turn. */
    assert(!j.off_line && !d.stop);
    j=centre_fixture();i=sample(900000,140,true);i.wheel[0]+=25;
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_CENTRE_DRIFT);
    i=sample(1000,0,true);j_begin(&j,true,&i);(void)junction_step(&j,&cal,&i);
    i.now+=20000;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_NO_CROSS);
    /* A brief black pulse must not count as a cross. */
    i=sample(1000,0,false);j_begin(&j,true,&i);(void)junction_step(&j,&cal,&i);
    i=sample(101000,5,false);(void)junction_step(&j,&cal,&i);
    i=sample(200000,10,true);(void)junction_step(&j,&cal,&i);
    i=sample(210000,11,false);d=junction_step(&j,&cal,&i);assert(j.phase==J_APPROACH && !d.stop);
    j=centre_fixture();j.striped=false;i=sample(900000,151,true);
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_BAD_STRIPE);
    j=centre_fixture();i=sample(900000,120,false);i.path=900;
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_PATH_LIMIT);
}
static void simulate(bool turn,bool branch) {
    Junction j;JunctionInput i=sample(1000,0,false);j_begin(&j,turn,&i);
    bool centred=false,rotated=false,exited=false;
    for(unsigned tick=0;tick<2600;++tick) {
        float mean=j_mean(&i);
        i.side_black=mean>=100 && mean<118;
        i.adc[0]=3464;i.adc[1]=3692;
        if(j.phase==J_TURN) {
            float angle=-(heading_deg(i.wheel[0]-j.turn_origin[0],i.wheel[1]-j.turn_origin[1]));
            if(angle>3 && !(branch && angle>=88 && angle<=95)) i.adc[0]=i.adc[1]=184;
        }
        AnalogDemand d=junction_step(&j,&cal,&i);
        if(j.phase==J_SETTLE) centred=true;
        if(j.phase==J_TURN_SETTLE) rotated=true;
        if(j.phase==J_EXIT) exited=true;
        if(d.stop) {
            assert(centred);
            if(turn && !branch) assert(j.reason==J_BRANCH_MISSING && !exited);
            else {
                assert(j.reason==J_COMPLETE);
                assert(rotated==turn && exited==turn);
                if(turn) assert(j.turn_degrees>=88 && j.turn_degrees<96);
            }
            return;
        }
        for(unsigned w=0;w<2;++w) {
            assert(abs(d.target[w])<=50);
            i.speed[w]=(float)d.target[w];i.wheel[w]+=i.speed[w]*0.01f;
            i.path+=fabsf(i.speed[w])*0.005f;
        }
        i.now+=10000;
    }
    assert(!"sequence failed to terminate");
}
static void test_turn_rejects_false_branch(void) {
    Junction j={.phase=J_TURN,.phase_us=1000};JunctionInput i=sample(1000,0,false);
    i.wheel[0]=-86;i.wheel[1]=86;
    AnalogDemand d=junction_step(&j,&cal,&i);
    assert(!d.stop && !j.off_line && j.phase==J_TURN); /* never left original black */
    j.off_line=true;i.adc[1]=184;
    for(unsigned n=0;n<10;++n) { i.now+=10000;d=junction_step(&j,&cal,&i); }
    assert(j.phase==J_TURN && !j.line_timing); /* only one sensor on branch */
    i.now=6001000;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_TURN_TIMEOUT);
    j=(Junction){.phase=J_SETTLE,.phase_us=1000};i=sample(2001000,100,false);i.speed[0]=10;
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_SETTLE_TIMEOUT);
}
static void test_signed_controller_and_stops(void) {
    for(unsigned w=0;w<2;++w) for(int desired=-50;desired<=50;++desired) {
        SignedWheel p={0};
        for(unsigned n=0;n<100;++n) {
            int32_t u=junction_wheel_step(&p,w,(int16_t)desired,desired*0.8f,0.01f,1000+n*10000,true);
            assert(abs(u)<=J_TURN_PEAK_PWM && (desired==0?u==0:u*desired>=0));
            DualLevels v=junction_levels(w,u,true);
            assert(v.a<=400 && v.b<=400 && !(v.a && v.b));
        }
        assert(junction_wheel_step(&p,w,0,40,0.01f,2000000,true)==0);
        assert(p.pi.integral==0 && p.pi.ramp==0 && !p.kick.active);
    }
    DualGuard g={0};assert(guard_start(&g,1000000,true,false));
    g.deadline_us=26000000;guard_tick(&g,1001000,true,true);assert(!g.active && g.reason==WHY_GP20);
    g=(DualGuard){0};assert(guard_start(&g,1000000,true,false));
    guard_tick(&g,1001000,false,false);assert(!g.active && g.reason==WHY_RELEASE);
    g=(DualGuard){0};assert(guard_start(&g,1000000,true,false));
    guard_tick(&g,1100000,true,false);assert(!g.active && g.reason==WHY_LATE);
    assert(side_valid(&(SideCalibration){0,1,3}));assert(side_valid(&(SideCalibration){1,0,3}));
    assert(!side_valid(&(SideCalibration){1,1,3}));assert(!side_valid(&(SideCalibration){0,1,1}));
}
static void test_loaded_turn_and_unchanged_follow(void) {
    for(unsigned w=0;w<2;++w) {
        SignedWheel loaded={0},unloaded={0};int sign=w==0?-1:1;
        for(unsigned n=0;n<110;++n) {
            uint64_t now=1000+n*10000;
            int32_t u=junction_wheel_step(&loaded,w,(int16_t)(sign*35),0,0.01f,now,true);
            (void)junction_wheel_step(&unloaded,w,(int16_t)(sign*35),0,0.01f,now,false);
            /* Stationary under load: effort must persist after120ms kicks end. */
            if(n>=40) {
                assert(u*sign==350);
                DualLevels actual=junction_levels(w,u,true);
                assert(actual.a+actual.b==350); /* no hidden25% bridge clamp */
            }
        }
        assert(fabsf(loaded.pi.integral)<=LINE_SPEED_I_LIMIT);
        SignedWheel overspeed={.pi={.ramp=35},.sign=(int8_t)sign,.peak_used=true};
        int32_t u=junction_wheel_step(&overspeed,w,(int16_t)(sign*35),sign*80,0.01f,2000000,true);
        assert(u*sign<J_TURN_FEEDFORWARD && u*sign>=0); /* not a forced output minimum */
        SignedWheel follow={0};WheelPI reference={0};LineBreakaway kick={0};
        for(unsigned n=0;n<500;++n) {
            int16_t wanted=n<100?45:n<200?25:n<300?0:n<400?50:35;
            float speed=n<50?0:wanted*0.9f;uint64_t now=1000+n*10000;
            int32_t expected=line_speed_step(&reference,w,wanted,speed,0.01f);
            expected=line_breakaway(&kick,now,wanted,reference.ramp,speed,expected);
            int32_t actual=junction_wheel_step(&follow,w,wanted,speed,0.01f,now,false);
            assert(expected==actual);
            DualLevels reference_levels=dual_levels(w,expected);
            DualLevels actual_levels=junction_levels(w,actual,false);
            assert(actual_levels.a==reference_levels.a && actual_levels.b==reference_levels.b);
        }
    }
    for(unsigned w=0;w<2;++w) for(int pwm=-1000;pwm<=1000;++pwm) {
        DualLevels turn=junction_levels(w,pwm,true),follow=junction_levels(w,pwm,false);
        assert(turn.a<=400 && turn.b<=400 && !(turn.a && turn.b));
        assert(follow.a<=250 && follow.b<=250 && !(follow.a && follow.b));
        DualLevels old=dual_levels(w,pwm);assert(old.a==follow.a && old.b==follow.b);
    }
}
static void test_one_shot_peak_and_timer_cap(void) {
    for(unsigned w=0;w<2;++w) {
        SignedWheel p={0};int sign=w==0?-1:1;uint64_t start=1000000;
        int32_t u=junction_wheel_step(&p,w,(int16_t)(sign*35),0,0.01f,start,true);
        assert(u*sign==400 && p.peak_end_us==start+150000);
        assert(junction_peak_cap(u,p.peak_end_us,start+149999)==u);
        assert(junction_peak_cap(u,p.peak_end_us,start+150000)==sign*350);
        /* Even a stale400 command presented much later gets clamped. */
        assert(junction_peak_cap(u,p.peak_end_us,start+300000)==sign*350);
        u=junction_wheel_step(&p,w,(int16_t)(sign*35),0,0.01f,start+150000,true);
        assert(abs(u)<=350 && !p.peak_active && p.peak_used);
        for(unsigned n=0;n<200;++n) {
            u=junction_wheel_step(&p,w,(int16_t)(sign*35),0,0.01f,start+160000+n*10000,true);
            assert(abs(u)<=350); /* a stalled turn never starts another40% pulse */
        }
        p=(SignedWheel){0};
        (void)junction_wheel_step(&p,w,(int16_t)(sign*35),0,0.01f,start,true);
        u=junction_wheel_step(&p,w,(int16_t)(sign*35),sign*25,0.01f,start+10000,true);
        assert(abs(u)<=350 && !p.peak_active && p.peak_used);
        u=junction_wheel_step(&p,w,(int16_t)(sign*35),0,0.01f,start+20000,true);
        assert(abs(u)<=350); /* no retrigger after early release */
        assert(junction_wheel_step(&p,w,0,0,0.01f,start+30000,true)==0);
        assert(!p.peak_used && !p.peak_active && p.peak_end_us==0);
    }
}
static void test_rolling_geometry_and_sequence(bool branch) {
    Junction j;JunctionInput i=sample(1000,0,false);j_begin_mode(&j,3,&i);
    assert(j.rolling && j.turn_enabled && j_turn_start_offset(&j)==55);
    bool started=false,turned=false;float rotation=0;
    for(unsigned tick=0;tick<2600;++tick) {
        float mean=j_mean(&i);i.side_black=mean>=100 && mean<118;
        i.adc[0]=3464;i.adc[1]=3692;
        if(j.phase==J_TURN) {
            rotation=-heading_deg(i.wheel[0]-j.turn_origin[0],i.wheel[1]-j.turn_origin[1]);
            if(rotation>3 && !(branch && rotation>=88 && rotation<=95)) i.adc[0]=i.adc[1]=184;
        }
        AnalogDemand d=junction_step(&j,&cal,&i);
        if(j.phase==J_TURN && !started) {
            started=true;
            /* 10ms sampling at<=50mm/s quantizes each detected edge by<=0.5mm. */
            assert(fabsf(j.centre_mm-146)<0.6f); /* 100+18/2+92-55 */
            assert(d.target[1]>0); /* no full stop before the rolling corner */
        }
        if(j.phase==J_TURN) {
            assert(d.target[0]==0 && d.target[1]>=0);
            /* Even with turn assistance enabled globally, the inner motor is OFF. */
            SignedWheel inner={0};assert(junction_wheel_step(&inner,0,0,0,0.01f,i.now,true)==0);
        }
        if(j.phase==J_ACQUIRE) { turned=true;assert(j.turn_degrees>=88 && j.turn_degrees<=95); }
        if(d.stop) {
            assert(started);
            if(branch) assert(turned && j.reason==J_COMPLETE);
            else assert(!turned && j.reason==J_BRANCH_MISSING);
            return;
        }
        for(unsigned w=0;w<2;++w) {
            i.speed[w]=(float)d.target[w];i.wheel[w]+=i.speed[w]*0.01f;
            i.path+=fabsf(i.speed[w])*0.005f;
        }
        i.now+=10000;
    }
    assert(!"rolling sequence failed to terminate");
}
static void test_rolling_drift_and_kinematics(void) {
    Junction j={.phase=J_TURN,.phase_us=1000,.rolling=true};JunctionInput i=sample(1000,0,false);
    i.wheel[1]=100;i.adc[0]=i.adc[1]=184;
    AnalogDemand d=junction_step(&j,&cal,&i);
    assert(!d.stop && d.target[0]==0 && d.target[1]>0); /* unequal arcs are intentional */
    i.wheel[0]=13;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_CENTRE_DRIFT);
    j=(Junction){.phase=J_TURN,.phase_us=1000,.rolling=true};i=sample(9001000,0,false);
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_TURN_TIMEOUT);
    /* Ideal 90deg arc, inner wheel fixed: radius55. Start(-55,0), end(0,55),
       heading left90. This is not an axle-centred stationary turn. */
    const float pi=3.14159265358979323846f,r=TRACK_MM/2;
    float outer_arc=TRACK_MM*pi/2;
    assert(fabsf(-heading_deg(0,outer_arc)-90)<0.001f);
    assert(fabsf((-r+r*sinf(pi/2))-0)<0.001f);
    assert(fabsf((r-r*cosf(pi/2))-55)<0.001f);
    SignedWheel moving={.pi={.integral=80,.ramp=30},.sign=1};
    int32_t pwm=junction_wheel_step(&moving,1,45,30,0.01f,1000000,true);
    assert(moving.pi.ramp==31 && fabsf(moving.pi.integral)<1 && pwm<320);
    assert(moving.peak_used && !moving.peak_active); /* already moving: no forced40% */
}
static JunctionInput turn_sample(uint64_t now,float angle,unsigned l,unsigned r) {
    JunctionInput i=sample(now,0,false);
    i.wheel[1]=angle*TRACK_MM*(3.14159265358979323846f/180.0f);
    i.path=i.wheel[1]*0.5f;
    i.adc[0]=(uint16_t)(184+(3464-184)*l/1000);
    i.adc[1]=(uint16_t)(184+(3692-184)*r/1000);
    return i;
}
static Junction acquire_fixture(unsigned l,unsigned r) {
    Junction j={.phase=J_TURN,.phase_us=1000,.rolling=true,.off_line=true};
    JunctionInput i=turn_sample(1000000,92,l,r);
    AnalogDemand d=junction_step(&j,&cal,&i);
    assert(j.phase==J_TURN && j.line_timing && !d.stop);
    i.now+=30000;i.wheel[1]+=0.9f;i.path+=0.45f;
    d=junction_step(&j,&cal,&i);
    assert(j.phase==J_ACQUIRE && !d.stop);
    assert(d.target[0]>=20 && d.target[1]>=20 && d.target[0]<=35 && d.target[1]<=35);
    assert((l>r)?d.target[0]<d.target[1]:d.target[0]>d.target[1]);
    return j;
}
static void test_gradual_handover(void) {
    /* User's V5 failure at7274ms: left721/right3, turn-only92.01deg.
       The same sustained observation must now start alignment, not keep pivoting. */
    Junction j=acquire_fixture(721,3);
    assert(j.follower.direction==1);
    JunctionInput i=turn_sample(j.phase_us+10000,93,700,500);i.path=j.exit_origin+1;
    AnalogDemand d=junction_step(&j,&cal,&i);assert(j.phase==J_ACQUIRE && !d.stop);
    i.now+=70000;d=junction_step(&j,&cal,&i);assert(j.phase==J_ACQUIRE);
    i.now+=10000;d=junction_step(&j,&cal,&i);
    assert(j.phase==J_EXIT && !d.stop && d.target[0]>0 && d.target[1]>0);
    float exit=j.exit_origin;i.now+=10000;i.wheel[0]+=20;i.wheel[1]+=20;
    d=junction_step(&j,&cal,&i);assert(j_mean(&i)-exit>=20 && d.stop && j.reason==J_COMPLETE);
    j=acquire_fixture(3,721);assert(j.follower.direction==-1);
    /* Short flicker, old line, and a line seen before75deg cannot start acquisition. */
    j=(Junction){.phase=J_TURN,.phase_us=1000,.rolling=true,.off_line=true};
    i=turn_sample(1000000,92,900,0);(void)junction_step(&j,&cal,&i);
    i.now+=20000;(void)junction_step(&j,&cal,&i);
    i.now+=10000;i.adc[0]=184;d=junction_step(&j,&cal,&i);
    assert(j.phase==J_TURN && !j.line_timing && !d.stop);
    j=(Junction){.phase=J_TURN,.phase_us=1000,.rolling=true};
    for(unsigned n=0;n<8;++n) { i=turn_sample(1000000+n*10000,92,900,900);(void)junction_step(&j,&cal,&i); }
    assert(j.phase==J_TURN && !j.line_timing && !j.off_line);
    j.off_line=true;
    for(unsigned n=0;n<8;++n) { i=turn_sample(1100000+n*10000,70,900,0);(void)junction_step(&j,&cal,&i); }
    assert(j.phase==J_TURN && !j.line_timing);
    j=(Junction){.phase=J_TURN,.phase_us=1000,.rolling=true,.off_line=true,
                 .line_timing=true,.line_us=8900000};
    i=turn_sample(9001000,92,721,3);d=junction_step(&j,&cal,&i);
    assert(d.stop && j.reason==J_TURN_TIMEOUT); /* candidate cannot bypass turn deadline */
    j=(Junction){.phase=J_TURN,.phase_us=1000,.rolling=true,.off_line=true,
                 .line_timing=true,.line_us=1000000};
    i=turn_sample(1040000,110.1f,721,3);d=junction_step(&j,&cal,&i);
    assert(d.stop && j.reason==J_BRANCH_MISSING);
    /* After transfer the inner wheel may move >12mm: this is forward alignment,
       not the fixed-inner-wheel turn. Remain bounded by60mm/3s/angle limits. */
    j=acquire_fixture(721,3);i=turn_sample(j.phase_us+100000,93,721,3);
    i.wheel[0]+=15;i.wheel[1]+=15;i.path=j.exit_origin+15;
    d=junction_step(&j,&cal,&i);assert(!d.stop && j.phase==J_ACQUIRE);
    i.path=j.exit_origin+60;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_BRANCH_MISSING);
    j=acquire_fixture(721,3);i=turn_sample(j.phase_us+3000000,93,721,3);
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_TURN_TIMEOUT);
    j=acquire_fixture(721,3);i=turn_sample(j.phase_us+100000,116,721,3);
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_BRANCH_MISSING);
    j=acquire_fixture(721,3);i=turn_sample(j.phase_us+10000,93,0,0);
    d=junction_step(&j,&cal,&i);assert(!d.stop && d.target[0]==20 && d.target[1]==35);
    i.now+=149999;d=junction_step(&j,&cal,&i);assert(!d.stop);
    i.now+=1;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_LINE_LOST);
    /* Either strong input alone never qualifies as established two-sensor tracking. */
    j=acquire_fixture(1000,0);i=turn_sample(j.phase_us+200000,93,1000,0);
    d=junction_step(&j,&cal,&i);assert(!d.stop && j.phase==J_ACQUIRE && !j.line_timing);
    /* Dropping below400 or a difference>350 resets the stable-alignment timer. */
    i.now+=10000;i.adc[1]=3692;(void)junction_step(&j,&cal,&i);assert(j.line_timing);
    i.now+=70000;i.adc[1]=184;d=junction_step(&j,&cal,&i);
    assert(!d.stop && j.phase==J_ACQUIRE && !j.line_timing);
}
int main(void) {
    test_geometry_and_qualification();simulate(false,true);simulate(true,true);simulate(true,false);
    test_turn_rejects_false_branch();test_signed_controller_and_stops();
    test_loaded_turn_and_unchanged_follow();
    test_one_shot_peak_and_timer_cap();
    test_rolling_geometry_and_sequence(true);test_rolling_geometry_and_sequence(false);
    test_rolling_drift_and_kinematics();
    test_gradual_handover();
    puts("PASS: V6 single-sensor transfer/forward alignment/stable exit/flicker-loss-path-angle bounds; old modes and PWM guards");
    return 0;
}
