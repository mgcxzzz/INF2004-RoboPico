#include <assert.h>
#include <stdio.h>
#include "drive_diag.h"
static DiagValues observed;
static unsigned raw_calls,soft_calls,diag_urgent_count,diag_filtered_count,diag_bypass_count;
#define DRIVE_DIAG_RAW(d) do {observed.v[D_OPT_L]=(d).target[0];observed.v[D_OPT_R]=(d).target[1];++raw_calls;}while(0)
#define DRIVE_DIAG_SOFT(d,kind,scan,active) do { \
 observed.v[D_SOFT_L]=(d).target[0];observed.v[D_SOFT_R]=(d).target[1];++soft_calls; \
 if((kind)==1)++diag_filtered_count;else if((kind)==2)++diag_urgent_count;else ++diag_bypass_count; \
 (void)(scan);(void)(active); }while(0)
#include "v12_2/scan_smooth.h"
#undef BARCODE_SCAN_SMOOTH_H
#define ScanSmooth ReferenceScanSmooth
#define scan_smooth_step reference_scan_smooth_step
#include "v12_2/scan_smooth_reference.h"
#undef ScanSmooth
#undef scan_smooth_step
int main(void) {
 const LineCalibration cal={{163,160},{3457,3700},7};
 unsigned ticks=0;
 for(unsigned profile=1;profile<=3;++profile) {
  Circuit a,b;BarcodeNav na,nb;BarcodeAnalog aa,ab;SmoothFollow la={0},lb={0};ScanSmooth sa={0};ReferenceScanSmooth sb={0};
  CircuitInput i={.now=1000,.adc={3457,3700},.speed={45,45}};
  circuit_begin(&a,&i);circuit_begin(&b,&i);barcode_nav_begin(&na,&i);barcode_nav_begin(&nb,&i);
  barcode_analog_begin(&aa,&na,180,i.now,i.wheel);barcode_analog_begin(&ab,&nb,180,i.now,i.wheel);
  for(unsigned k=0;k<4000;++k) {
   i.now+=10000;i.wheel[0]+=.44;i.wheel[1]+=.45;i.path+=.445;
   const unsigned l[]={1000,800,500,1000,0,0,250,700,500,900};
   const unsigned r[]={950,650,300,0,1000,0,900,700,950,250};unsigned part=(k/31)%10;
   i.adc[0]=(uint16_t)(163+3294*l[part]/1000);i.adc[1]=(uint16_t)(160+3540*r[part]/1000);
   for(unsigned t=0;t<5;++t) {
    unsigned adc=k%170<40?220:180;
    barcode_analog_sample(&aa,&na,adc,i.now-8000+2000*t,i.wheel);barcode_analog_sample(&ab,&nb,adc,i.now-8000+2000*t,i.wheel);
   }
   AnalogDemand x=scan_smooth_step(&a,&na,&cal,&i,profile,&la,&aa,&sa,.01f);
   AnalogDemand y=reference_scan_smooth_step(&b,&nb,&cal,&i,profile,&lb,&ab,&sb,.01f);
   assert(x.target[0]==y.target[0] && x.target[1]==y.target[1] && x.stop==y.stop && x.decision==y.decision);
   assert(x.level[0]==y.level[0] && x.level[1]==y.level[1] && a.phase==b.phase);
   assert(observed.v[D_SOFT_L]==x.target[0] && observed.v[D_SOFT_R]==x.target[1]);
   assert(sa.mean==sb.mean && sa.half==sb.half && sa.scanning==sb.scanning && sa.active==sb.active);
   ++ticks;
   if(x.stop){circuit_begin(&a,&i);circuit_begin(&b,&i);sa=(ScanSmooth){0};sb=(ReferenceScanSmooth){0};}
  }
 }
 assert(raw_calls==ticks && soft_calls==ticks && diag_filtered_count && diag_urgent_count && diag_bypass_count);
 printf("PASS diagnostic taps preserve V1.1 optical targets/state bit-for-bit across%u ticks, all profiles, diag_filtered_count%u diag_urgent_count%u diag_bypass_count%u; no control overrides.\n",ticks,diag_filtered_count,diag_urgent_count,diag_bypass_count);
}
