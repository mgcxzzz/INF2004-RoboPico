#include <assert.h>
#include <stdio.h>
#include "visible_wheel.h"
static void authority_and_slew(void) {
    for(unsigned w=0;w<2;++w)for(unsigned profile=1;profile<=3;++profile) {
        VisibleWheel s={0};SignedWheel p={0};int old=0,peak=0;
        for(unsigned k=0;k<400;++k) {
            int pwm=visible_wheel_step(&s,&p,w,45,0,.01f,k*10000,false,profile,old);
            assert(pwm>=0 && pwm<=(int)circuit_forward_cap(profile));
            assert(pwm-old<=7 && old-pwm<=13);old=pwm;if(pwm>peak)peak=pwm;
            assert(!p.kick.active && !p.peak_active);
        }
        assert(peak>=280); /* sustained deficit still reaches useful effort */
        int pwm=visible_wheel_step(&s,&p,w,45,0,.01f,4010000,true,profile,old);
        assert(pwm-old<=7);
        for(unsigned k=0;k<500;++k)pwm=visible_wheel_step(&s,&p,w,45,0,.01f,4020000+k*10000,true,profile,pwm);
        assert(pwm>300 && pwm<=420);
        assert(!visible_wheel_step(&s,&p,w,0,40,.01f,9990000,false,profile,pwm));
        assert(!s.active && !p.sign && !p.pi.integral);
    }
}
static void sticky_wheel_model(void) {
    /* Simple independent speed plant: static friction > rolling friction;
     * unequal wheels and a temporary extra load. This checks tracking and
     * repeated restart behaviour, not a prediction of chassis motion. */
    for(unsigned w=0;w<2;++w) {
        VisibleWheel s={0};SignedWheel p={0};float speed=0;int output=0,stops=0;
        for(unsigned k=0;k<2000;++k) {
            output=visible_wheel_step(&s,&p,w,45,speed,.01f,k*10000,false,1,output);
            float friction=(w?110:95)+(k>=700 && k<1100?35:0);
            float need=speed<1?friction+35:friction;
            float goal=output>need?(output-friction)*.8f:0;
            float before=speed;speed+=(goal-speed)*.01f/.08f;
            if(k>300 && before>=1 && speed<1)++stops;
            if(k==650 || k==1350 || k==1999)assert(fabsf(speed-45)<8);
        }
        assert(stops==0);printf("sticky wheel%u: final speed=%.1f PWM=%d, restarts=%d\n",w,(double)speed,output,stops);
    }
}
static void phase_handover(void) {
    Circuit c={.phase=C_FOLLOW};AnalogDemand d={.level={700,700},.decision=SYM_STRAIGHT,.target={45,45}};
    assert(visible_wheel_eligible(&c,&d,false));
    assert(!visible_wheel_eligible(&c,&d,true));
    c.follower.core.white=true;assert(!visible_wheel_eligible(&c,&d,false));c.follower.core.white=false;
    for(unsigned phase=J_CENTRE;phase<=J_ACQUIRE;++phase) {c.phase=phase;assert(!visible_wheel_eligible(&c,&d,false));}
    VisibleWheel s={0};SignedWheel p={.sign=1,.pi={.ramp=45}};
    int u=visible_wheel_step(&s,&p,0,45,45,.01f,0,false,1,180);
    assert(abs(u-180)<=13); /* seed actual previous output at handover */
    visible_wheel_leave(&s,&p);assert(!s.active && !p.pi.integral);
    assert(!visible_wheel_step(&s,&p,0,-30,0,.01f,0,false,1,0));
}
int main(void) {authority_and_slew();sticky_wheel_model();phase_handover();puts("PASS: continuous positive drive; sustained load authority; zero-off; sticky-wheel tracking; phase isolation.");}
