#include <stdio.h>
#include <stdlib.h>
#include "barcode_analog.h"
int main(int argc,char **argv){
 if(argc!=2)return 2;FILE*f=fopen(argv[1],"r");if(!f)return 2;
 char line[256];(void)fgets(line,sizeof(line),f);BarcodeNav nav;BarcodeAnalog a;CircuitInput i={0};
 unsigned index,adc,us,count=0,starts[AO_BANKS]={0},shorts[AO_BANKS]={0},edges[AO_BANKS]={0};double mm=0,first=-1,last=-1;
 while(fgets(line,sizeof(line),f)){
  if(sscanf(line,"%u,%u,%lf,%u",&index,&us,&mm,&adc)!=4)return 3;
  i.now=us;i.wheel[0]=i.wheel[1]=mm;
  if(!count){barcode_nav_begin(&nav,&i);barcode_analog_begin(&a,&nav,adc,us,i.wheel);first=mm;}
  bool before[AO_BANKS];double prev[AO_BANKS];
  for(unsigned b=0;b<AO_BANKS;b++){before[b]=a.bank[b].reader.frame;prev[b]=a.bank[b].reader.edge_mm;}
  barcode_analog_sample(&a,&nav,adc,us,i.wheel);
  for(unsigned b=0;b<AO_BANKS;b++){
   if(!before[b]&&a.bank[b].reader.frame){starts[b]++;if(b==5)printf("start_candidate threshold=%u from_mm=%.2f at_mm=%.2f time_ms=%.1f\n",barcode_analog_threshold(b),a.bank[b].reader.start_mm,mm,us/1000.0);}
   if(a.bank[b].changed){edges[b]++;if(mm-prev[b]<.35)shorts[b]++;}
  }
  count++;last=mm;
 }
 fclose(f);unsigned total_starts=0;for(unsigned b=0;b<AO_BANKS;b++)total_starts+=starts[b];
 printf("samples=%u span_mm=%.2f..%.2f candidates=%u selections=%u last_letter=%c frame_starts_all_banks=%u\n",count,first,last,a.candidates,a.selections,nav.last_code?nav.last_code:'-',total_starts);
 for(unsigned b=0;b<AO_BANKS;b++)if(b==11||b==35||b==39||b==43||starts[b])printf("threshold=%u starts=%u edges=%u sub0.35mm_edges=%u pending_elements=%u\n",barcode_analog_threshold(b),starts[b],edges[b],shorts[b],a.bank[b].reader.count);
 return 0;
}
