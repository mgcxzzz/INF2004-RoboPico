#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "maneuver_power.h"
int main(void) {
    for(unsigned w=0;w<2;++w)for(int sign=-1;sign<=1;sign+=2) {
        SignedWheel p={0};ManeuverPower s={0};
        int pwm=maneuver_wheel_step(&s,&p,w,(int16_t)(sign*90),0,.01f,1000);
        assert(pwm==sign*750 && p.peak_active && p.peak_used);
        DualLevels levels=maneuver_levels(w,pwm,true);
        bool a_side=w==0?sign>0:sign<0;
        assert((a_side?levels.a:levels.b)==750 && !(a_side?levels.b:levels.a));
        if(sign<0) {levels=maneuver_levels(w,pwm,false);assert(!levels.a && !levels.b);}
        uint64_t deadline=p.peak_end_us;
        assert(maneuver_output_cap(sign*900,deadline,deadline-1)==sign*750);
        assert(maneuver_output_cap(sign*900,deadline,deadline)==sign*600);
        pwm=maneuver_wheel_step(&s,&p,w,(int16_t)(sign*90),0,.01f,deadline);
        assert(abs(pwm)<=600 && !p.peak_active && p.peak_used);
        for(unsigned k=0;k<100;++k) {
            pwm=maneuver_wheel_step(&s,&p,w,(int16_t)(sign*90),0,.01f,deadline+10000+k*10000);
            assert(abs(pwm)<=600 && p.peak_end_us==deadline);
        }
        assert(!maneuver_wheel_step(&s,&p,w,(int16_t)(sign*30),sign*80,.01f,deadline+2000000));
        maneuver_leave(&s,&p,0);assert(!s.active && !p.peak_active && !p.pi.integral);
        assert(!maneuver_wheel_step(&s,&p,w,0,0,.01f,deadline+2100000));
        assert(!s.active && !p.sign);
        pwm=maneuver_wheel_step(&s,&p,w,(int16_t)(sign*90),sign*55,.01f,deadline+2200000);
        assert(abs(pwm)<=600 && !p.peak_active); /* observed movement ends kick early */
    }
    SignedWheel a={0},b={0};ManeuverPower idle={0};
    for(unsigned k=0;k<200;++k) {
        maneuver_leave(&idle,&a,45);
        int x=circuit_wheel_step(&a,0,45,k%60,.01f,1000+k*10000,false,1);
        int y=circuit_wheel_step(&b,0,45,k%60,.01f,1000+k*10000,false,1);
        assert(x==y && !memcmp(&a,&b,sizeof(a)));
    }
    puts("PASS: physical60%/bounded75% effort;180ms ISR expiry;no repeated stalled kick;early moving cutoff;overspeed coast;zero OFF;signed pin mapping;optional forward-only mapping;normal-controller parity.");
}
