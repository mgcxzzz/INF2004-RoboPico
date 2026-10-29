#include <assert.h>
#include <stdio.h>
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h"
#include "successful_calibration.h"
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h"
static void replay(const char*path,char expected,unsigned candidates){
 FILE*f=fopen(path,"r");assert(f);char line[256];assert(fgets(line,sizeof(line),f));
 BarcodeNav n;BarcodeAnalog a;CircuitInput i={0};unsigned rows=0,index,us,adc;double mm;
 while(fgets(line,sizeof(line),f)){
  assert(sscanf(line,"%u,%u,%lf,%u",&index,&us,&mm,&adc)==4);i.now=us;i.wheel[0]=i.wheel[1]=mm;
  if(!rows){barcode_nav_begin(&n,&i);barcode_analog_begin(&a,&n,adc,us,i.wheel);}
  barcode_analog_sample(&a,&n,adc,us,i.wheel);rows++;
 }
 fclose(f);assert(rows==2048);
 if(expected){assert(n.last_code==expected&&n.decoded_total==1&&a.selections==1&&a.candidates>=candidates);}
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
 BarcodeSaved s={.magic=B_MAGIC,.version=31,.profile=1,.recorded_profile=1,.calibration=successful_calibration(),.side={0,1,3}};
 s.crc=log_crc((const uint8_t*)&s+offsetof(BarcodeSaved,count),barcode_record_crc_size(s.version));assert(barcode_record_valid(&s));
 LineCalibration c=s.calibration;c.white[0]=170;assert(!successful_calibration_boot(&c,31,true)&&c.white[0]==170);
 assert(successful_calibration_boot(&c,31,false)&&c.white[0]==163);assert(barcode_analog_log_threshold(31,63)==204);
}

/* Obtain fallback votes through the actual Code39 reader, all A-Z both ways.
 * No test injects a letter into the production ADC reader. */
static void quiet_fallback(void){
 for(unsigned rev=0;rev<2;rev++)for(char c='A';c<='Z';c++){
    double widths[29];unsigned k=0;uint16_t code=0;
    for(unsigned p=0;p<512;p++)if(barcode_character(p)==c)code=(uint16_t)p;
    uint16_t patterns[3]={0x094,code,0x094};
    for(unsigned p=0;p<3;p++){
        for(unsigned b=0;b<9;b++)widths[k++]=patterns[p]&(1u<<(8-b))?9:3;
        if(p<2)widths[k++]=3;
    }
    if(rev)for(unsigned j=0;j<14;j++){double t=widths[j];widths[j]=widths[28-j];widths[28-j]=t;}
    BarcodeReader r;barcode_reader_reset(&r,false,1000,0);double mm=40;uint64_t now=401000;
    (void)barcode_reader_edge(&r,true,now,mm);BarcodeRead out={0};
    for(unsigned j=0;j<29;j++){mm+=widths[j];now+=(uint64_t)(widths[j]*10000);out=barcode_reader_edge(&r,j%2!=0,now,mm);}
    assert(out.events&(1u<<B_DECODE));assert(out.code==c&&out.reverse==(bool)rev);
    assert(r.leading_quiet_mm==40);AnalogBank v={.reader=r,.vote=c,.reverse=out.reverse,.vote_us=now,.vote_start=out.start,.vote_end=mm};
    memcpy(v.pattern,out.pattern,sizeof(v.pattern));assert(!v.stable&&!v.candidate);
    BarcodeAnalog a={.minimum=170,.maximum=2000};BarcodeNav n={0};a.bank[7]=v;
    assert(!barcode_analog_select(&a,&n,now,mm)); /* no trailing white yet */
    assert(!barcode_analog_select(&a,&n,now+1000000,mm)); /* waiting still is insufficient */
    assert(!barcode_analog_select(&a,&n,now+200000,mm+29.9));
    assert(barcode_analog_select(&a,&n,now+400000,mm+30));
    assert(n.last_code==c && n.decoded_total==1 && a.selected==7);
    assert(!barcode_analog_select(&a,&n,now+500000,mm+35)); /* duplicate */
    for(unsigned negative=0;negative<7;negative++){
        a=(BarcodeAnalog){.minimum=170,.maximum=2000};n=(BarcodeNav){0};a.bank[7]=v;
        if(negative==0)a.bank[7].reader.leading_quiet_mm=29.9;
        if(negative==1)a.bank[7].stable=true;
        if(negative==2)a.bank[7].candidate=true;
        if(negative==3)a.bank[7].reader.edge_mm+=1; /* intervening edge */
        if(negative==4)a.bank[7].reader.frame=true;
        if(negative==5)a.bank[7].vote='1';
        if(negative==6){a.bank[20]=v;a.bank[20].vote=c=='Z'?'A':'Z';a.bank[20].reader.leading_quiet_mm=0;}
        assert(!barcode_analog_select(&a,&n,now+400000,mm+30));assert(!n.pending&&!n.decoded_total);
    }
    a=(BarcodeAnalog){0};n=(BarcodeNav){0};a.bank[7]=v;
    assert(!barcode_analog_select(&a,&n,now+3000001,mm+30));
 }
 puts("PASS all26 letters both directions: actual full-reader fallback, two quiet zones, movement gate, conflict, duplicate, stale and malformed-state vetoes");
}


static void damaged_capture(void){
 for(unsigned kind=0;kind<5;kind++){
    FILE*f=fopen("fixtures/barcode_Z_latest.csv","r");assert(f);char line[256];assert(fgets(line,sizeof(line),f));
    BarcodeNav n;BarcodeAnalog a;CircuitInput i={0};unsigned index,us,adc,rows=0;double mm;
    while(fgets(line,sizeof(line),f)){
        assert(sscanf(line,"%u,%u,%lf,%u",&index,&us,&mm,&adc)==4);
        if(kind==0 && mm<20)continue; /* insufficient leading quiet */
        if(kind==1 && mm>190)break; /* insufficient trailing quiet */
        if(kind==2 && mm>=35 && mm<=82)adc=170; /* erase start sentinel */
        if(kind==3 && mm>=130 && mm<=174)adc=170; /* erase stop sentinel */
        if(kind==4 && adc>500)adc=170; /* missing wide bars */
        i.now=us;i.wheel[0]=i.wheel[1]=mm;
        if(!rows){barcode_nav_begin(&n,&i);barcode_analog_begin(&a,&n,adc,us,i.wheel);}
        barcode_analog_sample(&a,&n,adc,us,i.wheel);rows++;
    }
    fclose(f);assert(rows>0);assert(!n.decoded_total&&!a.selections&&!n.pending);
 }
 puts("PASS real-capture mutations: cropped quiet zones, erased start/stop sentinels and missing wide bars all rejected");
}

int main(void){replay("fixtures/barcode_Z_captured.csv",'Z',2);replay("fixtures/barcode_Z_latest.csv",'Z',1);replay("fixtures/barcode_Y_captured.csv",0,0);consensus();noise();storage();quiet_fallback();damaged_capture();puts("PASS fine v0.3: both real Z scans, honest Y failure, consensus and quiet fallback guards, record31");return 0;}
