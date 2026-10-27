#include <assert.h>
#include <stdio.h>
#include "forward_effort.h"
#include "v12_2/scan_smooth.h"
#include "v12_2/baseline/testing/subsystem2/circuit_left_test/circuit_load.h"
static void transitions(void) {
    for(unsigned w=0;w<2;++w)for(unsigned profile=1;profile<=3;profile+=2) {
        ForwardEffort s={0};SignedWheel p={0};int last=0;unsigned starts=0;bool prior=false;
        for(unsigned k=0;k<500;++k) {
            int out=forward_effort_step(&s,&p,w,45,0,.01f,k%40<20,profile,last);
            assert(out>=0 && out<=420 && out-last<=19);
            if(s.pulse && !prior)++starts;prior=s.pulse;last=out;
        }
        assert(starts==1); /* no recurring full-duty relay on a blocked wheel */
        s.duty=410;s.previous_target=45;p.pi.ramp=45;p.pi.integral=140;
        assert(forward_effort_step(&s,&p,w,39,84.7f,.01f,false,profile,410)==0);
        s.duty=300;s.previous_target=45;p.pi.ramp=45;
        assert(forward_effort_step(&s,&p,w,10,40,.01f,false,profile,300)==0);
        assert(forward_effort_step(&s,&p,w,0,0,.01f,true,profile,0)==0 && !s.active && !p.sign);
        /* Reproduce the logged assist0->4 edge with the SAME target/speed.
         * Ceiling changes do not install the legacy310raw pivot feedforward. */
        s=(ForwardEffort){.active=true,.start_used=true,.duty=147,.previous_target=45};
        p=(SignedWheel){.sign=1,.pi={.ramp=45,.integral=20}};
        int out=forward_effort_step(&s,&p,w,45,5.5f,.01f,true,1,147);
        assert(out<=165 && !p.was_turning && !p.peak_active);
        /* Changing assist membership is bumpless and does not clear PI. */
        ForwardEffort a=s,b=s;SignedWheel pa=p,pb=p;
        int x=forward_effort_step(&a,&pa,w,45,30,.01f,false,1,out);
        int y=forward_effort_step(&b,&pb,w,45,30,.01f,true,1,out);
        assert(x==y && pa.pi.integral==pb.pi.integral);
    }
}
static void speed_models(void) {
    for(unsigned w=0;w<2;++w)for(unsigned surface=0;surface<3;++surface) {
        ForwardEffort s={0};SignedWheel p={0};float v=0,filtered=0,error=0;int last=0;
        for(unsigned k=0;k<1600;++k) {
            float load=110+surface*20+(k>=500 && k<900?20:0);
            int out=forward_effort_step(&s,&p,w,45,filtered,.01f,false,1,last);
            float equilibrium=fmaxf(0,(out-load)*.9f);
            if(v<2 && out<load+20)equilibrium=0;
            v+=(equilibrium-v)*.12f;filtered+=(v-filtered)*.25f;
            assert(out>=0 && out<=300 && out-last<=19);last=out;
            if(k>=1400)error+=fabsf(45-filtered);
        }
        assert(error/200<3);
    }
}
/* Motion changes simulated sensor readings and can qualify the actual legacy
 * load selector. This is deliberately a model, not physical smoothness proof. */
static void steering_feedback(void) {
    const LineCalibration cal={{164,168},{3514,3389},7};
    for(unsigned trial=0;trial<6;++trial) {
        Circuit c={0};BarcodeNav n={0};BarcodeAnalog a={0};SmoothFollow old={0};ScanSmooth scan={0};
        CircuitLoad load={0};ForwardEffort effort[2]={{0}};SignedWheel pi[2]={{0}};
        CircuitInput i={.now=1000,.adc={3514,3389}};
        circuit_begin(&c,&i);barcode_nav_begin(&n,&i);barcode_analog_begin(&a,&n,180,i.now,i.wheel);
        double lateral=trial%2?2:-2,heading=(trial%2?1:-1)*.02;
        float v[2]={0},f[2]={0};int last[2]={0};unsigned loaded_ticks=0,restarts=0;
        for(unsigned k=0;k<1800;++k) {
            i.now+=10000;
            for(unsigned w=0;w<2;++w) {
                double spot=lateral+127*sin(heading)+(w?9:-9);
                float level=clampf((float)((11-fabs(spot))/4),0,1);
                i.adc[w]=(uint16_t)(cal.white[w]+level*(cal.black[w]-cal.white[w]));i.speed[w]=f[w];
            }
            AnalogDemand d=scan_smooth_step(&c,&n,&cal,&i,1,&old,&a,&scan,.01f);
            if(d.stop || c.busy) {++restarts;break;}
            int outer=circuit_load_outer(&load,&i,c.phase,&d,false);
            if(outer>=0)++loaded_ticks;
            for(unsigned w=0;w<2;++w) {
                int out=forward_effort_step(&effort[w],&pi[w],w,d.target[w],f[w],.01f,outer==(int)w,1,last[w]);
                assert(out>=0 && out<=(outer==(int)w?420:300) && out-last[w]<=19);
                float resistance=110+trial/2*15+w*10;
                float equilibrium=fmaxf(0,(out-resistance)*.9f);
                if(v[w]<2 && out<resistance+20)equilibrium=0;
                v[w]+=(equilibrium-v[w])*.12f;f[w]+=(v[w]-f[w])*.25f;
                i.wheel[w]+=v[w]*.01;last[w]=out;
            }
            heading+=(v[0]-v[1])/110*.01;lateral+=(v[0]+v[1])*.005*sin(heading);
            i.path+=(fabsf(v[0])+fabsf(v[1]))*.005;
            assert(isfinite(lateral) && isfinite(heading));
        }
        printf("MODEL steering trial%u path=%.1f lateral=%.1f heading=%.3f loaded_ticks=%u stop=%u\n",
            trial,i.path,lateral,heading,loaded_ticks,restarts);
        assert(!restarts && loaded_ticks>0 && fabs(lateral)<5 && i.path>150);
    }
}
int main(void) {
    transitions();speed_models();steering_feedback();
    puts("PASS forward effort: assist handovers, immediate overspeed/coast/off, bounded one-shot aid, six load models and six motion/optics/assist feedback models.");
}
