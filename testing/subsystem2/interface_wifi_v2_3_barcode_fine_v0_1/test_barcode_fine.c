#include <assert.h>
#include <stdio.h>
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h"
#include "successful_calibration.h"
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h"
static void replay(const char*path,char expected){
 FILE*f=fopen(path,"r");assert(f);char line[256];assert(fgets(line,sizeof(line),f));
 BarcodeNav n;BarcodeAnalog a;CircuitInput i={0};unsigned rows=0,index,us,adc;double mm;
 while(fgets(line,sizeof(line),f)){
  assert(sscanf(line,"%u,%u,%lf,%u",&index,&us,&mm,&adc)==4);i.now=us;i.wheel[0]=i.wheel[1]=mm;
  if(!rows){barcode_nav_begin(&n,&i);barcode_analog_begin(&a,&n,adc,us,i.wheel);}
  barcode_analog_sample(&a,&n,adc,us,i.wheel);rows++;
 }
 fclose(f);assert(rows==2048);
 if(expected){assert(n.last_code==expected&&n.decoded_total==1&&a.selections==1&&a.candidates>=2);assert(a.selected==63);}
 else assert(n.decoded_total==0&&a.selections==0);
 printf("capture %s: decoded=%u letter=%c selected=%u candidates=%u\n",path,n.decoded_total,n.last_code?n.last_code:'-',a.selected,a.candidates);
}
static void consensus(void){
 BarcodeAnalog a={0};BarcodeNav n={0};a.minimum=170;a.maximum=2000;
 a.bank[63]=(AnalogBank){.vote='Z',.vote_start=20,.vote_end=100,.vote_us=1000};
 assert(!barcode_analog_select(&a,&n,1000,100));
 a.bank[62]=a.bank[63];assert(!barcode_analog_select(&a,&n,1000,100));
 a.bank[62]=(AnalogBank){0};a.bank[64]=a.bank[63];assert(barcode_analog_select(&a,&n,1000,100));
 assert(a.selected==63&&n.last_code=='Z'&&n.decoded_total==1);
 assert((0x80u|(a.selected<<1)|1u)==255u);assert(!barcode_analog_select(&a,&n,1000,100));
 a=(BarcodeAnalog){0};n=(BarcodeNav){0};
 for(unsigned k=0;k<2;k++){a.bank[k]=(AnalogBank){.vote='Y',.vote_us=1000,.vote_start=20,.vote_end=100};a.bank[63+k]=(AnalogBank){.vote='Z',.vote_us=1000,.vote_start=20,.vote_end=100};}
 assert(!barcode_analog_select(&a,&n,1000,100)&&a.conflict&&!n.pending);
}
static void noise(void){
 BarcodeAnalog a;BarcodeNav n;CircuitInput i={.now=1000};barcode_nav_begin(&n,&i);barcode_analog_begin(&a,&n,180,i.now,i.wheel);
 unsigned x=123456789;
 for(unsigned j=0;j<20000;j++){x=x*1664525u+1013904223u;i.now+=2000;i.wheel[0]=i.wheel[1]=j*.08;barcode_analog_sample(&a,&n,195+((x>>16)%25),i.now,i.wheel);}
 assert(!n.decoded_total&&!a.selections);
}
static void storage(void){
 BarcodeSaved s={.magic=B_MAGIC,.version=30,.profile=1,.recorded_profile=1,.calibration=successful_calibration(),.side={0,1,3}};
 s.crc=log_crc((const uint8_t*)&s+offsetof(BarcodeSaved,count),barcode_record_crc_size(s.version));assert(barcode_record_valid(&s));
 LineCalibration c=s.calibration;c.white[0]=170;assert(!successful_calibration_boot(&c,30,true)&&c.white[0]==170);
 assert(successful_calibration_boot(&c,30,false)&&c.white[0]==163);assert(barcode_analog_log_threshold(30,63)==204);
}
int main(void){replay("fixtures/barcode_Z_captured.csv",'Z');replay("fixtures/barcode_Y_captured.csv",0);consensus();noise();storage();puts("PASS fine pair: real Z, honest Y failure, consensus/conflict/noise checks, record30");return 0;}
