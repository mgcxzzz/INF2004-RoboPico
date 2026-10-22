#include <assert.h>
#include <stdio.h>
#include "boosted_wheel.h"

int main(void) {
    assert(effort_scale20(0)==0 && effort_scale20(100)==120);
    assert(effort_scale20(250)==300 && effort_scale20(350)==420 && effort_scale20(400)==480);
    for(unsigned w=0;w<2;++w) {
        SignedWheel ref={0},boost={0};
        for(unsigned n=0;n<1000;++n) {
            bool turn=n>=200 && n<700;
            int16_t desired=n<200?45:n<400?-35:n<600?45:n<700?0:n<800?30:45;
            float speed=(n%100<40)?0:desired*1.1f;
            uint64_t now=1000000+n*10000;
            int32_t old=junction_wheel_step(&ref,w,desired,speed,0.01f,now,turn);
            int32_t u=boosted_junction_step(&boost,w,desired,speed,0.01f,now,turn);
            assert(u==effort_scale20(old)); /* straight/turn/align/exit/breakaway */
            assert(boost.pi.ramp==ref.pi.ramp && boost.pi.integral==ref.pi.integral);
            assert(boost.peak_end_us==ref.peak_end_us);
            if(!desired) assert(u==0);
            uint64_t until=turn?boost.peak_end_us:0;
            int32_t committed=boosted_junction_peak_cap(u,until,now);
            DualLevels v=boosted_junction_levels(w,committed,turn);
            assert((int32_t)(v.a+v.b)==(committed<0?-committed:committed));
            assert(!(v.a && v.b));
            /* Repeated timer checks cannot re-scale/increase a held command. */
            assert(boosted_junction_peak_cap(committed,until,now)==committed);
        }
        for(int pwm=-1000;pwm<=1000;++pwm) {
            int32_t scaled=effort_scale20(pwm);
            assert(scaled>=-999 && scaled<=999 && scaled==-effort_scale20(-pwm));
            DualLevels old=junction_levels(w,pwm,false);
            DualLevels forward=boosted_junction_levels(w,scaled,false);
            DualLevels turn=boosted_junction_levels(w,scaled,true);
            assert(forward.a==effort_scale20(old.a) && forward.b==effort_scale20(old.b));
            assert(forward.a<=300 && forward.b<=300 && turn.a<=480 && turn.b<=480);
            assert(!(turn.a && turn.b));
        }
    }
    for(int sign=-1;sign<=1;sign+=2) {
        assert(boosted_junction_peak_cap(sign*480,1150000,1149999)==sign*480);
        assert(boosted_junction_peak_cap(sign*480,1150000,1150000)==sign*420);
        assert(boosted_junction_peak_cap(sign*480,1150000,2000000)==sign*420);
    }
    puts("PASS: exact1.20x V9 output, polarity/zero, phase transitions, caps300/420/480, deadline/ISR no double scaling");
    return 0;
}
