#include <assert.h>
#include <stdio.h>
#include "probe_logic.h"
#include "../circuit_with_barcode/straight_pwm.h"
static const LineCalibration cal={{176,175},{3483,3498},7};
int main(void) {
    for(unsigned profile=1;profile<=3;++profile)for(unsigned variant=0;variant<6;++variant) {
        Circuit c={0};BarcodeNav n={.pending='A',.cross_ready=true};SmoothFollow s={.paired=true};
        CircuitInput i={.now=1000,.adc={variant%2?3483:176,variant<3?175:3498}};
        AnalogDemand d=probe_drive_step(&c,&n,&cal,&i,profile,&s,.01f);
        assert(!d.stop && d.target[0]==45 && d.target[1]==45 && d.decision==25);
        assert(c.busy && c.phase==C_FOLLOW && !c.fast && !n.pending && !n.allow_cross);
        assert(!ramp_profile_active(profile,&c,&d));float speed[2]={45,45};
        assert(!straight_pwm_eligible(&c,&s,&d,speed,false));
        assert(!c.follower.core.white && !c.follower.search.active);
    }
    LineCalibration bad=cal;bad.mask=0;Circuit c={0};BarcodeNav n={0};SmoothFollow s={0};CircuitInput i={0};
    assert(probe_drive_step(&c,&n,&bad,&i,1,&s,.01f).stop);
    DualGuard g={0};assert(guard_start(&g,1000,true,false));g.deadline_us=probe_deadline(1000);
    g.last_control_us=g.deadline_us-1;guard_tick(&g,g.deadline_us-1,true,false);assert(g.active);
    guard_tick(&g,g.deadline_us,true,false);assert(!g.active && g.reason==WHY_LIMIT && g.duration_us==4000000);
    g=(DualGuard){0};assert(guard_start(&g,1000,true,false));g.deadline_us=probe_deadline(1000);
    guard_tick(&g,2000,true,true);assert(!g.active && g.reason==WHY_GP20);
    for(unsigned w=0;w<2;++w) {SignedWheel p={.sign=1,.pi={.ramp=45}};assert(circuit_wheel_step(&p,w,45,0,.01f,1000000,false,1)==300);}
    puts("PASS: fixed45/45 across optical states/profiles; no recovery/route/ramp/smoothing; invalid calibration rejected;4s independent deadline/manual stop; original motor breakaway retained.");
}
