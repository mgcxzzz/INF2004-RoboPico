/* Offline diagnostic sweep: original Code39 reader, unchanged width checks.
 * No expected-letter filtering, no inferred edges, no car connection. */
#include <stdio.h>
#include <stdlib.h>
#include "barcode_scan.h"
typedef struct {unsigned us,adc;double mm;} Row;
static Row rows[4096];
int main(int argc,char**argv){
 if(argc!=2)return 2;FILE*f=fopen(argv[1],"r");if(!f)return 2;
 char line[256];(void)fgets(line,sizeof(line),f);unsigned n=0,index;
 while(n<4096&&fgets(line,sizeof(line),f)){if(sscanf(line,"%u,%u,%lf,%u",&index,&rows[n].us,&rows[n].mm,&rows[n].adc)!=4)return 3;n++;}fclose(f);if(!n)return 4;
 unsigned match=0;
 const unsigned hysts[]={0,1,2,3,8,16};
 for(unsigned hi=0;hi<6;hi++)for(unsigned t=160;t<=300;t++){
  unsigned h=hysts[hi];if(t<h)continue;
  bool stable=rows[0].adc>=t,candidate=stable;unsigned cand_us=rows[0].us;double cand_mm=rows[0].mm;
  BarcodeReader reader;barcode_reader_reset(&reader,stable,cand_us,cand_mm);
  for(unsigned i=1;i<n;i++){
   Row*r=&rows[i];bool black=stable?r->adc>t-h:r->adc>=t+h;
   if(black!=candidate){candidate=black;cand_us=r->us;cand_mm=r->mm;}
   if(candidate!=stable&&r->us-cand_us>=2000){stable=candidate;BarcodeRead out=barcode_reader_edge(&reader,stable,cand_us,cand_mm);
    if(out.events&(1u<<B_DECODE)){printf("MATCH threshold=%u hyst=%u letter=%c from=%.2f to=%.2f reverse=%d\n",t,h,out.code,out.start,out.end,out.reverse);match++;}}
   (void)barcode_reader_expire(&reader,r->us,r->mm);
  }
 }
 printf("samples=%u threshold_hysteresis_matches=%u\n",n,match);return 0;
}
