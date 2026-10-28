#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "cruise_handover.h"
static void captured_exit(float ref_l,float ref_r,float i_l,float i_r,
    int16_t target_l,int16_t target_r,float speed_l,float speed_r,float dt,int expected_l,int expected_r) {
    SignedWheel old[2]={{.pi={.ramp=ref_l,.integral=i_l},.sign=1},
                        {.pi={.ramp=ref_r,.integral=i_r},.sign=1}},next[2];
    memcpy(next,old,sizeof(old));int16_t targets[2]={target_l,target_r};float speeds[2]={speed_l,speed_r};
    assert(cruise_handover_eligible(1,C_FOLLOW,25,targets,speeds,next,false,false));
    int result[2],original[2];
    for(unsigned w=0;w<2;++w) {
        circuit_leave_fast(&old[w]);cruise_handover_apply(&next[w],true);
        original[w]=circuit_wheel_step(&old[w],w,targets[w],speeds[w],dt,3000000,false,1);
        result[w]=circuit_wheel_step(&next[w],w,targets[w],speeds[w],dt,3000000,false,1);
        assert(next[w].pi.ramp==old[w].pi.ramp && next[w].pi.integral>15);
        assert(result[w]>original[w]+15 && result[w]<=300 && !next[w].kick.active);
    }
    assert(abs(original[0]-expected_l)<=2 && abs(original[1]-expected_r)<=2);
    printf("CAPTURED EXIT oldPWM=%d/%d H1=%d/%d I=%.2f/%.2f (recorded speed held; not a physical prediction)\n",
        original[0],original[1],result[0],result[1],(double)next[0].pi.integral,(double)next[1].pi.integral);
}
static void eligibility_and_legacy(void) {
    unsigned compared=0;
    for(unsigned profile=1;profile<=3;++profile)for(unsigned phase=0;phase<=10;phase+=2)
    for(unsigned decision=11;decision<=25;++decision)for(unsigned special=0;special<2;++special)
    for(unsigned stop=0;stop<2;++stop) {
        SignedWheel a[2]={{.pi={.ramp=60,.integral=53},.sign=1},{.pi={.ramp=59,.integral=59},.sign=1}},b[2];
        int16_t target[2]={49,50};float speed[2]={51,56.5f};memcpy(b,a,sizeof(a));
        bool keep=cruise_handover_eligible(profile,phase,decision,target,speed,a,special,stop);
        if(keep){assert(!special && !stop && phase==C_FOLLOW && profile!=2 && decision>=24);continue;}
        for(unsigned w=0;w<2;++w){circuit_leave_fast(&a[w]);cruise_handover_apply(&b[w],false);}
        assert(!memcmp(a,b,sizeof(a)));++compared;
    }
    SignedWheel p[2]={{.pi={.ramp=135,.integral=140},.sign=1},{.pi={.ramp=60,.integral=60},.sign=1}};
    int16_t target[2]={50,50};float speed[2]={135,50};
    assert(!cruise_handover_eligible(3,C_FOLLOW,25,target,speed,p,false,false));
    speed[0]=60;assert(cruise_handover_eligible(3,C_FOLLOW,25,target,speed,p,false,false));
    speed[0]=60.1f;assert(!cruise_handover_eligible(3,C_FOLLOW,25,target,speed,p,false,false));
    speed[0]=NAN;assert(!cruise_handover_eligible(1,C_FOLLOW,25,target,speed,p,false,false));
    speed[0]=45;target[0]=19;assert(!cruise_handover_eligible(1,C_FOLLOW,25,target,speed,p,false,false));
    target[0]=45;p[0].was_turning=true;assert(!cruise_handover_eligible(1,C_FOLLOW,25,target,speed,p,false,false));
    p[0].was_turning=false;p[0].sign=-1;assert(!cruise_handover_eligible(1,C_FOLLOW,25,target,speed,p,false,false));
    /* Existing speed-controller negative authority, inner coast and zero keep
     * working even if the state-preserving helper is called explicitly. */
    for(unsigned w=0;w<2;++w) {
        SignedWheel q={.pi={.ramp=60,.integral=100},.sign=1};cruise_handover_apply(&q,true);
        assert(circuit_wheel_step(&q,w,0,45,.01f,1000,false,1)==0 && q.sign==0);
        q=(SignedWheel){.pi={.ramp=60,.integral=100},.sign=1};cruise_handover_apply(&q,true);
        assert(circuit_wheel_step(&q,w,10,40,.01f,1000,false,1)==0 && q.pi.integral==0);
        q=(SignedWheel){.pi={.ramp=60,.integral=100},.sign=1};SignedWheel before=q;
        cruise_handover_apply(&q,true);circuit_leave_fast(&before);
        int a=circuit_wheel_step(&q,w,45,90,.01f,1000,false,1);
        int b=circuit_wheel_step(&before,w,45,90,.01f,1000,false,1);
        assert(a==b && q.pi.integral<=0);
    }
    printf("PASS cruise handover scope: %u exact legacy bypass states; large overspeed/invalid speed/low target/turn state rejection; unchanged zero/coast/overspeed authority.\n",compared);
}
static void slowdown_models(void) {
    unsigned cases=0;
    for(unsigned w=0;w<2;++w)for(unsigned load=100;load<=160;load+=30)for(unsigned gain=0;gain<2;++gain) {
        SignedWheel p={0};float v=0,measured=0,k=gain?1.1f:.7f;
        for(unsigned tick=0;tick<1600;++tick) {
            int16_t wanted=tick<800?55:50;
            if(tick==800) {
                int16_t targets[2]={50,50};float speeds[2]={measured,measured};SignedWheel pair[2]={p,p};
                assert(cruise_handover_eligible(1,C_FOLLOW,25,targets,speeds,pair,false,false));
                cruise_handover_apply(&p,true);
            }
            int u=circuit_wheel_step(&p,w,wanted,measured,.01f,1000+tick*10000,false,1);
            float resistance=(float)load,equilibrium=fmaxf(0,(u-resistance)*k);
            if(v<2 && u<resistance+15)equilibrium=0;
            v+=(equilibrium-v)*.12f;measured+=(v-measured)*.25f;
            assert(u>=0 && u<=300);
            if(tick>800){assert(v>40 && v<62 && !p.kick.active);}
        }
        assert(fabsf(measured-50)<2);++cases;
    }
    printf("PASS %u simple load models:55->50mm/s settles without restart pulses after the selected handover. Physical validation remains required.\n",cases);
}
int main(void) {
    captured_exit(54,59,53.5f,59,49,50,51,56.5f,.010116f,103,110);
    captured_exit(53,56,38.5f,21,46,50,40.5f,32.5f,.010195f,108,140);
    eligibility_and_legacy();slowdown_models();
}
