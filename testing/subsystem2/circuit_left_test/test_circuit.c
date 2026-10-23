#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "circuit_logic.h"
#include "circuit_wheel.h"
#include "circuit_record.h"
#include "circuit_capture.h"
#include "circuit_feedback.h"
#include "circuit_load.h"
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
            assert(c.centre_ms>0 && c.turn_ms>0 && c.align_ms>=150);
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
    assert(circuit_step(&c,&cal,&i,1).stop && c.reason==C_LOSS_NO_DIRECTION);
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
    SignedWheel p={.pi={.ramp=135,.integral=140}};
    circuit_leave_fast(&p);assert(p.pi.ramp==50 && p.pi.integral==0);
    for(unsigned n=0;n<100;++n) {
        int pwm=circuit_wheel_step(&p,0,25,90,.01f,(uint64_t)n*10000,false,1);
        assert(pwm>=0 && pwm<=300 && p.pi.ramp<=50);
    }
}
static void same_corrections_and_cruise(void) {
    /* Exact calibration lets every normalized error be exercised, including
       the80/250/500/1000 breakpoints, without ADC quantization ambiguity. */
    const LineCalibration unit={{100,100},{1100,1100},7};
    for(unsigned profile=1;profile<=3;++profile) {
        for(int error=-1000;error<=1000;++error) {
            Circuit c;CircuitInput i=initial();circuit_begin(&c,&i);
            i.adc[0]=(uint16_t)(100+(error>=0?1000:1000+error));
            i.adc[1]=(uint16_t)(100+(error<=0?1000:1000-error));
            AnalogDemand d=circuit_step(&c,&unit,&i,profile);
            int16_t base[2];symmetric_targets(error,base);
            assert(!d.stop && d.target[0]==base[0] && d.target[1]==base[1] && !c.fast);
            i.now+=300000;d=circuit_step(&c,&unit,&i,profile);
            if(profile>1 && abs(error)<=80) {
                assert(c.fast && d.decision==C_FAST && d.target[0]==(int)profile*45 && d.target[1]==(int)profile*45);
            } else assert(!c.fast && d.target[0]==base[0] && d.target[1]==base[1]);
        }
        Circuit c;CircuitInput i=initial();circuit_begin(&c,&i);
        (void)circuit_step(&c,&cal,&i,profile);i.now+=300000;
        (void)circuit_step(&c,&cal,&i,profile);
        /* A single asymmetric observation drops fast mode; no lingering boost. */
        i.adc[1]=184;i.now+=10000;
        AnalogDemand d=circuit_step(&c,&cal,&i,profile);
        assert(!c.fast && d.target[0]==0 && d.target[1]==45);
        i.adc[0]=184;i.now+=10000;d=circuit_step(&c,&cal,&i,profile);
        assert(!d.stop && !c.fast && d.target[0]==0 && d.target[1]==45);
        i.now+=C_LOSS_US;assert(circuit_step(&c,&cal,&i,profile).stop && c.reason==C_LOSS_TIME);
        /* Balanced but weak signals never earn fast cruise. */
        i=initial();i.adc[0]=300;i.adc[1]=300;circuit_begin(&c,&i);
        (void)circuit_step(&c,&unit,&i,profile);i.now+=1000000;
        d=circuit_step(&c,&unit,&i,profile);assert(!c.fast && d.target[0]==45 && d.target[1]==45);
    }
}
static void curve_after_branch(void) {
    for(unsigned profile=1;profile<=3;++profile) {
        Circuit c;CircuitInput i=initial();circuit_begin(&c,&i);c.busy=true;
        c.phase=J_ACQUIRE;c.turn=(Junction){.phase=J_ACQUIRE,.rolling=true,.phase_us=i.now};
        /* First outgoing observation is left-only black, never both centred.
           Handover must finish, preserving normal steering and its loss memory. */
        i.adc[1]=184;i.wheel[1]=TRACK_MM*3.141592653589793/2;i.path=i.wheel[1]/2;
        c.turn.exit_origin=(float)i.path;
        for(unsigned n=0;n<16;++n) {
            AnalogDemand d=circuit_step(&c,&cal,&i,profile);
            assert(!d.stop && !c.fast && d.target[0]==0 && d.target[1]==45);
            if(n<15) assert(c.turns==0);
            advance(&i,d);
        }
        assert(c.turns==1 && c.phase==C_FOLLOW && c.align_ms==150);
        /* The next curve moves the stripe toward the right sensor. */
        i.adc[0]=184;i.adc[1]=3692;
        AnalogDemand d=circuit_step(&c,&cal,&i,profile);
        assert(!d.stop && !c.fast && d.target[0]==45 && d.target[1]==0);
        /* Both-white still has a bounded recovery;150ms alone no longer kills it. */
        i.adc[0]=i.adc[1]=184;i.now+=10000;
        d=circuit_step(&c,&cal,&i,profile);assert(!d.stop);
        i.now+=160000;d=circuit_step(&c,&cal,&i,profile);assert(!d.stop);
        i.now+=C_LOSS_US-160000;assert(circuit_step(&c,&cal,&i,profile).stop);
    }
    /* Noise / no persistent outgoing signal cannot count as a completed turn. */
    Circuit c;CircuitInput i=initial();circuit_begin(&c,&i);c.busy=true;
    c.turn=(Junction){.phase=J_ACQUIRE,.phase_us=i.now};
    i.now+=J_ALIGN_US;assert(circuit_step(&c,&cal,&i,1).stop && c.reason==J_ALIGN_TIMEOUT && !c.turns);
}
static void records(void) {
    static CircuitSaved s;
    s.magic=C_MAGIC;s.version=1;s.calibration=cal;s.side=(SideCalibration){0,1,3};s.profile=s.recorded_profile=1;
    s.duration_us=400000000;s.ticks[0]=-500000;s.ticks[1]=500000;s.count=C_ROWS;s.summary_count=C_SUMMARIES;
    s.crc=log_crc((uint8_t *)&s+offsetof(CircuitSaved,count),sizeof(s)-offsetof(CircuitSaved,count));
    assert(circuit_record_valid(&s));s.rows[100].adc[0]^=1;assert(!circuit_record_valid(&s));
    s.rows[100].adc[0]^=1;s.version=2;assert(circuit_record_valid(&s));
    s.version=3;assert(circuit_record_valid(&s));
    s.version=4;assert(circuit_record_valid(&s));
    s.version=5;assert(!circuit_record_valid(&s));
    CircuitLegacy old={.magic=0x314e434a,.version=1,.calibration=cal,.side={0,1,3},.mode=3,.selected_mode=3,.firmware=10};
    old.crc=log_crc((uint8_t *)&old+offsetof(CircuitLegacy,count),sizeof(old)-offsetof(CircuitLegacy,count));
    LineCalibration imported={0};SideCalibration side={0};assert(circuit_import(&old,&imported,&side));
    assert(calibration_valid(&imported) && side_valid(&side));old.rows[10]^=1;assert(!circuit_import(&old,&imported,&side));
    /* Chronological ring traversal after more than a full buffer. */
    unsigned head=0,count=0;uint64_t ring[C_ROWS];
    for(unsigned n=0;n<C_ROWS*3+123;++n) { ring[head]=n;head=(head+1)%C_ROWS;if(count<C_ROWS) ++count; }
    for(unsigned n=0;n<count;++n) assert(ring[(head+C_ROWS-count+n)%C_ROWS]==(uint64_t)(C_ROWS*2+123+n));
}
static void weak_signal_recovery(void) {
    const LineCalibration unit={{100,100},{1100,1100},7};
    CircuitTracker t={0};SymmetricTracker old={0};AnalogDemand d;
    /* Reproduce the observed strong -> weak right-only -> both-white pattern.
       The old policy forgets the right correction; V3 retains it. */
    for(uint64_t now=11780000;now<=12800000;now+=10000) {
        uint16_t l=now<12031000?160:now<12281000?14:0;
        uint16_t r=now<12031000?559:now<12281000?287:now<12532000?305:now<12782000?309:234;
        d=circuit_line_demand(&t,&unit,100+l,100+r,now,100);
        AnalogDemand before=symmetric_demand(&old,&unit,100+l,100+r,now,100);
        assert(!d.stop && !before.stop);
    }
    uint64_t white=12805000;
    assert(symmetric_demand(&old,&unit,100,220,white,101).stop);
    d=circuit_line_demand(&t,&unit,100,220,white,101);
    assert(!d.stop && d.decision==SYM_RECOVER_RIGHT && d.target[0]==45 && d.target[1]==0);
    d=circuit_line_demand(&t,&unit,100,100,white+C_LOSS_US-1,140);assert(!d.stop);
    d=circuit_line_demand(&t,&unit,100,100,white+C_LOSS_US,140);
    assert(d.stop && t.loss_reason==C_LOSS_TIME);
    t=(CircuitTracker){0};(void)circuit_line_demand(&t,&unit,600,100,1000,0);
    (void)circuit_line_demand(&t,&unit,100,100,2000,0);
    d=circuit_line_demand(&t,&unit,100,100,3000,50);
    assert(d.stop && t.loss_reason==C_LOSS_DISTANCE);
    t=(CircuitTracker){0};d=circuit_line_demand(&t,&unit,100,100,1000,0);
    assert(d.stop && t.loss_reason==C_LOSS_NO_DIRECTION);
    t=(CircuitTracker){0};(void)circuit_line_demand(&t,&unit,600,100,1000,0);
    d=circuit_line_demand(&t,&unit,100,100,1000+C_MEMORY_US+1,0);
    assert(d.stop && t.loss_reason==C_LOSS_NO_DIRECTION);
    /* Weak flashes and alternating signs cannot immediately invent a direction. */
    t=(CircuitTracker){0};(void)circuit_line_demand(&t,&unit,100,350,1000,0);
    (void)circuit_line_demand(&t,&unit,350,100,30000,0);
    d=circuit_line_demand(&t,&unit,100,100,31000,0);assert(d.stop);
    t=(CircuitTracker){0};(void)circuit_line_demand(&t,&unit,100,350,1000,0);
    (void)circuit_line_demand(&t,&unit,100,350,31000,0);
    d=circuit_line_demand(&t,&unit,100,100,41000,0);assert(!d.stop && d.target[0]==45 && d.target[1]==0);
    /* Reacquisition resets the white episode and can choose the other direction. */
    (void)circuit_line_demand(&t,&unit,600,100,50000,1);
    d=circuit_line_demand(&t,&unit,100,100,60000,1);assert(!d.stop && d.target[0]==0 && d.target[1]==45);
}
static void capture_effort(void) {
    CircuitCapture s={0};CircuitInput i=initial();i.path=100;
    AnalogDemand d={.level={935,0},.target={3,45}};
    assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1);
    circuit_capture_begin(&s,i.now,i.path);
    assert(circuit_capture_outer(&s,&i,J_ACQUIRE,&d)==1);
    d.target[0]=18;assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==1); /* hysteresis */
    d.target[0]=21;assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1);
    d.target[0]=18;assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1);
    d.target[0]=3;assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==1);
    d.level[0]=0;assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1); /* never blind */
    d.level[0]=935;i.now=s.since+C_CAPTURE_US;
    assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1 && !s.window);
    circuit_capture_begin(&s,i.now,i.path);i.path+=C_CAPTURE_MM;
    assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1 && !s.window);
    circuit_capture_begin(&s,i.now,i.path);d.target[0]=d.target[1]=30;i.speed[0]=i.speed[1]=20;
    (void)circuit_capture_outer(&s,&i,C_FOLLOW,&d);i.now+=C_ROLLING_US;
    assert(circuit_capture_outer(&s,&i,C_FOLLOW,&d)==-1 && !s.window);
    for(unsigned w=0;w<2;++w) {
        SignedWheel p={0};circuit_capture_prepare(&p);
        for(unsigned n=0;n<100;++n) {
            int pwm=circuit_wheel_step(&p,w,45,0,.01f,1000+(uint64_t)n*10000,true,1);
            assert(pwm>300 && pwm<=420 && !p.peak_active && !p.peak_end_us);
        }
        /* Inner target remains ordinary low-speed effort, with no turn floor. */
        p=(SignedWheel){0};int inner=circuit_wheel_step(&p,w,3,0,.01f,1000,false,1);
        assert(inner<100);
    }
}
static void cumulative_feedback(void) {
    CircuitFeedback s={0};
    /* One tick per20ms would fail V2's per10ms <2 criterion despite moving. */
    for(unsigned n=0;n<300;++n) assert(!circuit_stalled(&s,1000+(uint64_t)n*10000,n/2,45,45));
    uint64_t last=s.since;int64_t ticks=s.anchor;
    assert(!circuit_stalled(&s,last+C_STALL_US-1,ticks,45,45));
    assert(circuit_stalled(&s,last+C_STALL_US,ticks,45,45));
    assert(!circuit_stalled(&s,last+C_STALL_US+1,ticks,3,3) && !s.watching);
    s=(CircuitFeedback){0};assert(!circuit_stalled(&s,1000,0,45,45));
    for(unsigned n=1;n<75;++n) assert(!circuit_stalled(&s,1000+(uint64_t)n*10000,n%2,45,45));
    assert(circuit_stalled(&s,751000,1,45,45)); /* sub-threshold jitter */
    s=(CircuitFeedback){0};assert(!circuit_stalled(&s,1000,0,45,45));
    assert(!circuit_stalled(&s,601000,4,45,45));
    assert(!circuit_stalled(&s,1201000,8,45,45)); /* separated forward bursts count */
}
static void curve_load_effort(void) {
    /* Latest V3 failure: ordinary curve, left requested45 but almost stopped;
       right sees full black and is intentionally requested only1mm/s. */
    CircuitLoad s={0};CircuitInput i=initial();i.speed[0]=1.3f;
    AnalogDemand d={.level={38,1000},.target={45,1}};
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    i.now+=C_LOAD_QUALIFY_US-1;
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    ++i.now;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==0);
    i.speed[0]=45;d.target[1]=20;
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==0); /* hold through brief recovery */
    i.path=C_LOAD_MM;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    d.target[1]=1;i.speed[0]=0;i.now+=C_LOAD_COOLDOWN_US-1;
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1 && !s.timing);
    ++i.now;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1 && s.timing);
    i.now+=C_LOAD_QUALIFY_US;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==0);
    i.now+=C_LOAD_US;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    for(unsigned gate=0;gate<7;++gate) {
        s=(CircuitLoad){0};i=initial();d=(AnalogDemand){.level={38,1000},.target={45,1}};
        unsigned phase=C_FOLLOW;bool capture=false;
        if(gate==0) i.speed[0]=15;
        if(gate==1) d.level[1]=399;
        if(gate==2) d.target[1]=13;
        if(gate==3) d.target[0]=29;
        if(gate==4) phase=J_TURN;
        if(gate==5) capture=true;
        if(gate==6) d.stop=true;
        assert(circuit_load_outer(&s,&i,phase,&d,capture)==-1);
        i.now+=C_LOAD_QUALIFY_US;
        assert(circuit_load_outer(&s,&i,phase,&d,capture)==-1 && !s.active);
    }
    s=(CircuitLoad){0};i=initial();d=(AnalogDemand){.level={1000,0},.target={1,45}};
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    i.now+=C_LOAD_QUALIFY_US;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==1);
    d.level[0]=0;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1); /* never blind */
    for(unsigned profile=1;profile<=3;++profile) {
        SignedWheel p={0};circuit_capture_prepare(&p);
        int pwm=circuit_wheel_step(&p,0,45,1.3f,.01f,i.now,true,profile);
        assert(pwm>300 && pwm<=420 && !p.peak_active && !p.peak_end_us);
    }
    /* Swapping the required direction restarts qualification, never boosts both. */
    s=(CircuitLoad){0};i=initial();d=(AnalogDemand){.level={1000,0},.target={1,45}};
    (void)circuit_load_outer(&s,&i,C_FOLLOW,&d,false);i.now+=C_LOAD_QUALIFY_US-1;
    d=(AnalogDemand){.level={0,1000},.target={45,1}};
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    ++i.now;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==-1);
    i.now+=C_LOAD_QUALIFY_US;assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,false)==0);
    assert(circuit_load_outer(&s,&i,C_FOLLOW,&d,true)==-1); /* capture takes priority */
}
int main(void) {
    for(unsigned p=1;p<=3;++p) circuit_simulation(p);
    detection_and_bounds();wheel_caps();records();same_corrections_and_cruise();curve_after_branch();
    weak_signal_recovery();capture_effort();cumulative_feedback();curve_load_effort();
    puts("PASS:12 turns/profile; steering parity/cruise; observed weak-signal regression;2s/50mm bounds; scoped capture effort/no new peak; cumulative feedback/stall; V1-V4 records; qualified curve load effort/bounds/CRC/ring.");
    return 0;
}
