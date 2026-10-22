#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "circuit_logic.h"
#include "circuit_wheel.h"
#include "circuit_record.h"
static const LineCalibration cal={{184,184},{3464,3692},7};
static CircuitInput initial(void) {
    return (CircuitInput){.now=1000,.adc={3464,3692}};
}
static void advance(CircuitInput *i,AnalogDemand d) {
    for(unsigned w=0;w<2;++w) {
        i->speed[w]=d.target[w];i->wheel[w]+=i->speed[w]*0.01;
        i->path+=fabs(i->speed[w])*0.005;
    }
    i->now+=10000;
}
static void circuit_simulation(unsigned profile) {
    Circuit c;CircuitInput i=initial();circuit_begin(&c,&i);
    double next_cross=1000;bool acquire=false;unsigned completed=0;
    for(unsigned n=0;n<120000 && completed<12;++n) {
        i.side_black=i.path>=next_cross && i.path<next_cross+18;
        i.adc[0]=3464;i.adc[1]=3692;
        if(c.busy && c.turn.phase==J_TURN) {
            double angle=((i.wheel[1]-c.edge_wheel[1]-c.turn.turn_origin[1])-
                          (i.wheel[0]-c.edge_wheel[0]-c.turn.turn_origin[0]))/TRACK_MM*180/3.141592653589793;
            if(angle>3 && angle<88) i.adc[0]=i.adc[1]=184;
            if(angle>=88) i.adc[1]=184; /* first one sensor, then both */
        }
        if(c.busy && c.turn.phase==J_ACQUIRE) acquire=true;
        AnalogDemand d=circuit_step(&c,&cal,&i,profile);
        if(d.stop) fprintf(stderr,"profile%u phase%u reason%u time%.2f path%.1f\n",profile,c.phase,c.reason,(double)i.now/1e6,i.path);
        assert(!d.stop && !c.fault);
        assert(d.target[0]>=0 && d.target[1]>=0);
        assert(d.target[0]<=(int)profile*50 && d.target[1]<=(int)profile*50);
        if(c.phase==J_TURN) {
            assert(d.target[0]==0);
            assert(d.target[1]==45+(int)(profile-1)*10 || d.target[1]==30+(int)(profile-1)*5);
        }
        if(c.completed) {
            ++completed;assert(c.turns==completed && c.phase==C_FOLLOW && acquire);
            assert(c.centre_ms>0 && c.turn_ms>0 && c.align_ms>=80);
            next_cross=i.path+1000;acquire=false;
        }
        advance(&i,d);
    }
    assert(completed==12 && i.now>35000000 && i.path>12000); /* three complete route cycles */
}
static void detection_and_bounds(void) {
    Circuit c;CircuitInput i=initial();circuit_begin(&c,&i);
    /* Long follow exceeds the old single-test approach/distance/run ceilings. */
    for(unsigned n=0;n<6000;++n) {
        AnalogDemand d=circuit_step(&c,&cal,&i,1);assert(!d.stop && c.phase==C_FOLLOW);advance(&i,d);
    }
    assert(i.path>2000);
    /* Turning more than3deg within each gate interval rejects a side hit. */
    c.gate_timing=false;
    for(unsigned n=0;n<70;++n) {
        i.speed[0]=20;i.speed[1]=70;i.wheel[0]+=.2;i.wheel[1]+=.7;i.path+=.45;i.now+=10000;
        i.side_black=n>=40;
        AnalogDemand d=circuit_step(&c,&cal,&i,1);assert(!d.stop && !c.busy);
    }
    assert(c.ignored==1);
    /* Old cross sweeps inside the150mm clearance lockout cannot retrigger. */
    c.clear_path=i.path+150;c.gate_timing=false;i.side_black=false;
    for(unsigned n=0;n<50;++n) {
        i.now+=10000;i.speed[0]=i.speed[1]=45;i.wheel[0]+=.45;i.wheel[1]+=.45;i.path+=.45;
        i.side_black=n>=40;assert(!circuit_step(&c,&cal,&i,1).stop && !c.busy);
    }
    assert(c.ignored==2);
    /* Missing branch cannot loop forever. */
    i=initial();circuit_begin(&c,&i);c.busy=true;c.edge_us=i.now;
    c.turn=(Junction){.phase=J_TURN,.rolling=true,.phase_us=i.now};c.phase=J_TURN;
    i.now+=9000000;assert(circuit_step(&c,&cal,&i,1).stop && c.reason==J_TURN_TIMEOUT);
    /* Real measured speed, rather than the lower command, gates a fast entry. */
    i=initial();circuit_begin(&c,&i);c.busy=true;
    c.turn=(Junction){.phase=J_CENTRE,.rolling=true,.striped=true,.centre_mm=45,.phase_us=i.now};
    i.wheel[0]=i.wheel[1]=45;i.path=45;i.speed[0]=i.speed[1]=100;
    assert(circuit_step(&c,&cal,&i,3).stop && c.reason==C_APPROACH_FAST);
    i=initial();circuit_begin(&c,&i);i.adc[0]=i.adc[1]=184;
    assert(circuit_step(&c,&cal,&i,1).stop && c.reason==J_LINE_LOST);
}
static void wheel_caps(void) {
    for(unsigned profile=1;profile<=3;++profile) for(unsigned w=0;w<2;++w) {
        SignedWheel p={0},baseline={0};
        for(unsigned n=0;n<2000;++n) {
            uint64_t now=(uint64_t)n*10000;
            bool turn=n>=700 && n<1000;
            int target=n>=1500?0:turn?65:circuit_cruise(profile);
            int pwm=circuit_wheel_step(&p,w,(int16_t)target,0,.01f,now,turn,profile);
            assert(pwm>=0 && pwm<=(turn?480:(int)circuit_forward_cap(profile)));
            if(turn && now>=7150000) assert(pwm<=420);
            if(!target) assert(pwm==0);
            DualLevels v=circuit_levels(w,pwm,turn,profile);
            assert(w==0?v.a==pwm && !v.b:v.b==pwm && !v.a);
            if(profile==1) assert(pwm==boosted_junction_step(&baseline,w,(int16_t)target,0,.01f,now,turn));
        }
    }
    assert(boosted_junction_peak_cap(480,150000,150000)==420);
}
static void records(void) {
    static CircuitSaved s;
    s.magic=C_MAGIC;s.version=1;s.calibration=cal;s.side=(SideCalibration){0,1,3};s.profile=s.recorded_profile=1;
    s.duration_us=400000000;s.ticks[0]=-500000;s.ticks[1]=500000;s.count=C_ROWS;s.summary_count=C_SUMMARIES;
    s.crc=log_crc((uint8_t *)&s+offsetof(CircuitSaved,count),sizeof(s)-offsetof(CircuitSaved,count));
    assert(circuit_record_valid(&s));s.rows[100].adc[0]^=1;assert(!circuit_record_valid(&s));
    CircuitLegacy old={.magic=0x314e434a,.version=1,.calibration=cal,.side={0,1,3},.mode=3,.selected_mode=3,.firmware=10};
    old.crc=log_crc((uint8_t *)&old+offsetof(CircuitLegacy,count),sizeof(old)-offsetof(CircuitLegacy,count));
    LineCalibration imported={0};SideCalibration side={0};assert(circuit_import(&old,&imported,&side));
    assert(calibration_valid(&imported) && side_valid(&side));old.rows[10]^=1;assert(!circuit_import(&old,&imported,&side));
    /* Chronological ring traversal after more than a full buffer. */
    unsigned head=0,count=0;uint64_t ring[C_ROWS];
    for(unsigned n=0;n<C_ROWS*3+123;++n) { ring[head]=n;head=(head+1)%C_ROWS;if(count<C_ROWS) ++count; }
    for(unsigned n=0;n<count;++n) assert(ring[(head+C_ROWS-count+n)%C_ROWS]==(uint64_t)(C_ROWS*2+123+n));
}
int main(void) {
    for(unsigned p=1;p<=3;++p) circuit_simulation(p);
    detection_and_bounds();wheel_caps();records();
    puts("PASS:12 repeated turns/profile, long follow, edge/clearance gates, approach guard, branch timeout, PWM caps, legacy/CRC and ring wrap.");
    return 0;
}
