#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drive_smooth.h"
static void bounds_and_bypass(void) {
    for(unsigned w=0;w<2;++w) {
        DriveSmooth s={0};SignedWheel p={0};int32_t previous=0;
        for(unsigned k=0;k<500;++k) {
            int32_t out=drive_smooth_step(&s,&p,w,45,k%60<20?0:30,.01f,1000+k*10000,false,1,true,previous);
            assert(out>=0 && out<=300 && out-previous<=10 && previous-out<=7);previous=out;
        }
        DriveSmooth fast={0};SignedWheel fast_pi={0};int32_t fast_previous=0;
        for(unsigned k=0;k<600;++k) {
            int32_t out=drive_smooth_step(&fast,&fast_pi,w,135,0,.01f,1000+k*10000,false,3,true,fast_previous);
            assert(out>=0 && out<=420 && out-fast_previous<=10);fast_previous=out;
        }
        assert(fast_previous==420);
        assert(drive_smooth_step(&s,&p,w,0,30,.01f,6000000,false,1,true,previous)==0 && !s.active && !s.extra);
        /* Deliberate inner-wheel coast is immediate, even with retained load. */
        s=(DriveSmooth){.active=true,.duty=250,.extra=90,.previous_target=45};
        assert(drive_smooth_step(&s,&p,w,10,40,.01f,6010000,false,1,true,250)==0 && !s.extra);
        for(unsigned profile=1;profile<=3;++profile)for(unsigned turning=0;turning<=1;++turning) {
            SignedWheel a={0},b={0};DriveSmooth idle={0};
            for(unsigned k=0;k<100;++k) {
                int target=k%30<10?45:k%30<20?0:30;
                int x=circuit_wheel_step(&a,w,target,k%45,.01f,1000+k*10000,turning,profile);
                int y=drive_smooth_step(&idle,&b,w,target,k%45,.01f,1000+k*10000,turning,profile,false,0);
                assert(x==y && !memcmp(&a,&b,sizeof(a)) && !idle.active);
            }
        }
    }
}
/* Deliberately simple stiction/load model: validates settling and removal of
 * relay oscillation across varying loads, not a prediction of the real robot. */
static void model(bool smooth,unsigned w,float load,float *variation,float *mean,float *settled_error) {
    SignedWheel p={0};DriveSmooth s={0};float velocity=0,filtered=0,previous=0,total=0,error=0;unsigned n=0;*variation=0;
    for(unsigned k=0;k<1600;++k) {
        float target=k<800?45:55;
        int out=smooth?drive_smooth_step(&s,&p,w,(int16_t)target,filtered,.01f,1000+k*10000,false,1,true,(int32_t)previous):
            circuit_wheel_step(&p,w,(int16_t)target,filtered,.01f,1000+k*10000,false,1);
        float dead=load+(k>=600 && k<1000?12:0);
        float equilibrium=out<dead?0:(out-dead)*1.25f;
        if(velocity<2 && out<dead+15)equilibrium=0;
        velocity+=(equilibrium-velocity)*.12f;filtered+=(velocity-filtered)*.25f;
        if(k>100) *variation+=fabsf(out-previous);
        if(k>=1300) {total+=filtered;error+=fabsf(target-filtered);++n;}
        previous=(float)out;
    }
    *mean=total/n;*settled_error=error/n;
}
int main(void) {
    bounds_and_bypass();
    for(unsigned w=0;w<2;++w)for(unsigned load=80;load<=180;load+=20) {
        float old_tv,new_tv,old_mean,new_mean,oe,ne;model(false,w,(float)load,&old_tv,&old_mean,&oe);model(true,w,(float)load,&new_tv,&new_mean,&ne);
        printf("model wheel%u load%u PWM_variation old=%.0f new=%.0f speed old=%.1f new=%.1f error new=%.1f\n",w,load,(double)old_tv,(double)new_tv,(double)old_mean,(double)new_mean,(double)ne);
        fflush(stdout);assert(ne<8 && new_mean>45 && new_mean<63);
    }
    puts("PASS ordinary forward PWM slew, sustained load effort, cap300, immediate zero/coast, legacy bypass and varying-stiction model. Physical smoothness remains unverified.");
}
