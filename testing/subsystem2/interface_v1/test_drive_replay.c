#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "drive_smooth.h"
#include "fixtures/jerk_capture.h"
int main(void) {
    SignedWheel old[2]={{0}},new[2]={{0}};DriveSmooth smooth[2]={{0}};
    int previous_old[2]={0},previous_new[2]={0};unsigned old_jumps=0,new_jumps=0,steady_old=0,steady_new=0;
    for(unsigned n=1;n<sizeof(jerk_capture)/sizeof(jerk_capture[0]);++n) {
        unsigned elapsed=jerk_capture[n].us-jerk_capture[n-1].us;
        float dt=elapsed/1000000.0f;
        for(unsigned w=0;w<2;++w) {
            int16_t target=jerk_capture[n].target[w];float speed=jerk_capture[n].speed[w];
            int a=circuit_wheel_step(&old[w],w,target,speed,dt,jerk_capture[n].us,false,1);
            int b=drive_smooth_step(&smooth[w],&new[w],w,target,speed,dt,jerk_capture[n].us,false,1,true,previous_new[w]);
            assert(b>=0 && b<=300);
            if(a-previous_old[w]>80)++old_jumps;
            if(b-previous_new[w]>80)++new_jumps;
            if(target>=20 && abs(target-jerk_capture[n-1].target[w])<=2) {
                if(a-previous_old[w]>80)++steady_old;
                if(b-previous_new[w]>80)++steady_new;
            }
            previous_old[w]=a;previous_new[w]=b;
        }
    }
    printf("V1.1 captured-input replay: positive PWM jumps>80 old=%u new=%u; near-steady targets old=%u new=%u.\n",old_jumps,new_jumps,steady_old,steady_new);
    assert(old_jumps>=20 && new_jumps<old_jumps/4 && steady_new==0);
    puts("PASS same recorded targets/speeds remove repeated kick edges; open-loop counterfactual replay does not predict new physical wheel speeds.");
}
