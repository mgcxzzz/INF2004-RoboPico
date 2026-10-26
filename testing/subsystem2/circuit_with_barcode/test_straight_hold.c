#include <assert.h>
#include <stdio.h>
#include "straight_pwm.h"
static AnalogDemand apply(SmoothFollow *s,Circuit *c,unsigned l,unsigned r) {
    AnalogDemand d={.level={(uint16_t)l,(uint16_t)r},.decision=SYM_STRAIGHT};
    symmetric_targets((int)l-(int)r,d.target);
    if(d.target[0]!=d.target[1])d.decision=d.target[0]<d.target[1]?SYM_LEFT:SYM_RIGHT;
    smooth_follow_apply(s,c,.01f,&d);return d;
}
static void plateau_and_release(void) {
    for(unsigned mirror=0;mirror<2;++mirror) {
        SmoothFollow s={0};Circuit c={.phase=C_FOLLOW};AnalogDemand d={0};
        for(unsigned k=0;k<9;++k)d=apply(&s,&c,mirror?1000:800,mirror?800:1000);
        assert(!s.paired); /* short black flashes do not earn straight hold */
        for(unsigned k=0;k<40;++k)d=apply(&s,&c,mirror?1000:800,mirror?800:1000);
        assert(s.paired && d.target[0]==45 && d.target[1]==45 && d.decision==B_STRAIGHT_HOLD);
        /* Different worn-black strengths within the hold band do not make
         * alternating differential commands once the plateau is qualified. */
        for(unsigned k=0;k<100;++k) {
            unsigned l=k%2?760:1000,r=k%2?1000:760;
            d=apply(&s,&c,l,r);assert(s.paired && d.target[0]==45 && d.target[1]==45);
        }
        d=apply(&s,&c,mirror?900:599,mirror?599:900);assert(!s.paired);
        for(unsigned k=0;k<40;++k)(void)apply(&s,&c,800,1000);
        d=apply(&s,&c,mirror?1000:650,mirror?650:1000);assert(!s.paired);
        /* The old urgent bend bypass remains immediate and mirrored. */
        d=apply(&s,&c,mirror?1000:0,mirror?0:1000);
        int16_t expected[2];symmetric_targets(mirror?1000:-1000,expected);
        assert(!s.active && !s.paired && d.target[0]==expected[0] && d.target[1]==expected[1]);
        c.follower.core.white=true;(void)apply(&s,&c,800,1000);assert(!s.paired);
    }
}
static void pwm_gates(void) {
    Circuit c={.phase=C_FOLLOW};SmoothFollow s={.paired=true};
    AnalogDemand d={.level={800,1000},.target={45,45}};float speed[2]={40,40};
    assert(straight_pwm_eligible(&c,&s,&d,speed,false));
    assert(straight_pwm_limit(160,190,.01f,true)==168);
    assert(straight_pwm_limit(190,140,.01f,true)==174);
    assert(straight_pwm_limit(160,0,.01f,true)==0);
    assert(straight_pwm_limit(0,300,.01f,true)==300);
    speed[0]=24.9f;assert(!straight_pwm_eligible(&c,&s,&d,speed,false));speed[0]=40;
    assert(!straight_pwm_eligible(&c,&s,&d,speed,true));
    d.target[0]=10;assert(!straight_pwm_eligible(&c,&s,&d,speed,false));d.target[0]=45;
    for(unsigned phase=J_CENTRE;phase<=J_ACQUIRE;++phase) {c.phase=phase;assert(!straight_pwm_eligible(&c,&s,&d,speed,false));}
    for(unsigned k=0;k<4;++k)assert(straight_pwm_limit(90,300,.01f,false)==300);
}
static void fast_profile_and_stop(void) {
    Circuit c={.phase=C_FOLLOW,.fast=true};SmoothFollow s={0};AnalogDemand d={0};
    for(unsigned k=0;k<60;++k) {d=(AnalogDemand){.level={900,900},.target={135,135},.decision=C_FAST};smooth_follow_apply(&s,&c,.01f,&d);}
    assert(s.paired && d.target[0]==135 && d.target[1]==135);
    d.stop=true;d.target[0]=d.target[1]=0;smooth_follow_apply(&s,&c,.01f,&d);
    assert(!s.paired && !d.target[0] && !d.target[1]);
}
int main(void) {plateau_and_release();pwm_gates();fast_profile_and_stop();puts("PASS: qualified strong-black plateau; hysteresis/edge release; mirrored urgent bends; PWM smooth only while rolling straight; all special/slow/stop paths bypass; fast targets retained.");}
