#include <stdio.h>
#include "b5_config.h"
#include "scans2.h"
static const char* gap(int L,int R,int need){ int pl=L>=need, pr=R>=need; if(pl&&pr){int d=L-R; if(d<=50&&d>=-50) return "RIGHT(tie)"; return R>L?"RIGHT":"LEFT";} if(pl)return "LEFT"; if(pr)return "RIGHT"; return "NONE";}
int main(void){
 for(int sym=0;sym<2;sym++){ printf(sym?"\n== symmetric window +/-44 (left samples beyond -44 dropped)\n":"== as recorded (-55..+44)\n");
  for(unsigned i=0;i<sizeof k/sizeof k[0];i++){ b5_scan_t s; b5_scan_clear(&s);
   for(int j=0;j<k[i].c;j++){ if(sym && k[i].a[j] < -440) continue; if(k[i].a[j]>440) continue;
     b5_sample_t m={0}; m.angle_dd=k[i].a[j]; m.range_mm=k[i].r[j]; char f=k[i].f[j];
     m.b_valid=f!='x'; m.b_weak=f=='w'; m.b_partial=f=='p'; m.b_block_only=f=='b'; m.b_stable=f=='s'; b5_scan_add_sample(&s,&m);}
   b5_params_t p={B5_NEAR_MM,0,(int16_t)(sym?-440:-550),440,B5_CAR_WIDTH_MM,B5_CAR_LENGTH_MM,B5_MARGIN_MM,B5_OBST_DEPTH_MM,1000,1};
   b5_profile_t pr; b5_build_navigation_profile(&s,&p,&pr);
   printf("%-13s clr %3u/%3u gap-choice %-10s edges %4d..%4d\n",k[i].l,(unsigned)pr.clear_left_mm,(unsigned)pr.clear_right_mm,gap(pr.clear_left_mm,pr.clear_right_mm,B5_CAR_WIDTH_MM+2*B5_MARGIN_MM),(int)pr.left_edge_dd,(int)pr.right_edge_dd);}}
 return 0;}
