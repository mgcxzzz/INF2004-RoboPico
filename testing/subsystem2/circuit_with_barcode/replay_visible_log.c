/* Recorded-input comparison only: does not simulate the changed robot path. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "straight_pwm.h"
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    FILE *file=fopen(argv[1],"r");if(!file)return 2;
    char line[2048];Circuit c={.phase=C_FOLLOW};SmoothFollow s={0};
    double old_tv=0,new_tv=0;unsigned used=0,pulses=0,reversals=0;
    int old[2]={0},last[2]={0},pwm[2]={0},previous_sign=0;float last_ms=0;
    unsigned held=0,pwm_eligible=0;
    while(fgets(line,sizeof(line),file)) {
        float v[40];unsigned n=0;char *part=strtok(line,",");
        while(part && n<40) {v[n++]=strtof(part,NULL);part=strtok(NULL,",");}
        if(n!=28 || v[1]!=1 || v[2]!=C_FOLLOW || v[21]!=0) {s=(SmoothFollow){0};last_ms=0;continue;}
        AnalogDemand d={.level={(uint16_t)v[13],(uint16_t)v[14]},.decision=SYM_STRAIGHT};
        int error=(int)d.level[0]-d.level[1];symmetric_targets(error,d.target);
        unsigned strongest=d.level[0]>d.level[1]?d.level[0]:d.level[1];
        if(strongest>=200 && strongest<400 && abs(error)>=150)circuit_faint_targets(error,d.level,d.target);
        d.decision=d.target[0]<d.target[1]?SYM_LEFT:d.target[0]>d.target[1]?SYM_RIGHT:SYM_STRAIGHT;
        float dt=last_ms?(v[0]-last_ms)/1000:.01f;bool adjacent=last_ms && dt<.05f;
        smooth_follow_apply(&s,&c,dt,&d);
        if(s.paired)++held;
        float speed[2]={v[7],v[8]};
        if(straight_pwm_eligible(&c,&s,&d,speed,false))++pwm_eligible;
        if(adjacent)for(unsigned w=0;w<2;++w) {
            old_tv+=abs((int)v[15+w]-old[w]);new_tv+=abs(d.target[w]-last[w]);
            if((int)v[9+w]==300 && pwm[w]<250)++pulses;
        }
        int sign=error>80?1:error < -80?-1:0;
        if(adjacent && sign && previous_sign && sign!=previous_sign)++reversals;
        if(sign)previous_sign=sign;
        for(unsigned w=0;w<2;++w) {old[w]=(int)v[15+w];last[w]=d.target[w];pwm[w]=(int)v[9+w];}
        last_ms=v[0];++used;
    }
    fclose(file);
    printf("retained visible rows=%u, recorded max-kick entries=%u, optical side reversals=%u\n",used,pulses,reversals);
    printf("target total variation: recorded=%.0f; updated same-input replay=%.0f (%.1f%% reduction)\n",old_tv,new_tv,100*(old_tv-new_tv)/(old_tv?old_tv:1));
    printf("qualified paired-black rows=%u, rolling straight PWM-eligible rows=%u\n",held,pwm_eligible);
    return used?0:1;
}
