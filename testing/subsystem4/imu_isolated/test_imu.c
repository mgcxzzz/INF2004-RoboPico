#include <assert.h>
#include <stdio.h>
#include "imu_driver.h"
typedef struct { uint8_t a[256],m[256];unsigned writes;int fail_reg,corrupt_reg; } Fake;
static bool read_fake(void *ctx,uint8_t addr,uint8_t reg,uint8_t *b,size_t n) {
    Fake *f=ctx;if(reg==f->fail_reg) return false;
    uint8_t *bank=addr==IMU_A?f->a:addr==IMU_M?f->m:NULL;
    if(!bank) return false;
    for(size_t j=0;j<n;++j) b[j]=bank[(uint8_t)((reg&0x7f)+j)];
    if(reg==f->corrupt_reg) b[0]^=1;
    return true;
}
static bool write_fake(void *ctx,uint8_t addr,uint8_t reg,uint8_t v) {
    Fake *f=ctx;++f->writes;
    assert((addr==IMU_A && reg>=0x20 && reg<=0x25) || (addr==IMU_M && reg<=2));
    if(reg==f->fail_reg) return false;
    (addr==IMU_A?f->a:f->m)[reg]=v;return true;
}
static void near(float x,float y) { assert(fabsf(x-y)<0.001f); }
static Fake recognised(void) {
    Fake f={.fail_reg=-1,.corrupt_reg=-1};f.a[0x0f]=0x33;memcpy(f.m+0x0a,"H43",3);return f;
}
static void test_configuration(void) {
    Fake f=recognised();ImuBus b={&f,read_fake,write_fake};ImuDevice d=imu_configure(b);
    assert(d.acc_on && d.mag_on && !d.errors && f.writes==10);
    assert(f.a[0x20]==0x47 && f.a[0x23]==0x88 && f.m[0]==0x14 && f.m[1]==0x20 && f.m[2]==0);
    f=recognised();f.a[0x0f]=0x49;memset(f.m+0x0a,0,3);d=imu_configure(b);
    assert(!d.acc_on && !d.mag_on && f.writes==0); /* unknown IDs are never configured */
    f=recognised();f.fail_reg=0x0a;d=imu_configure(b);
    assert(d.acc_on && !d.mag_on && d.errors==1); /* partial device failure stays visible */
    f=recognised();f.corrupt_reg=0x23;d=imu_configure(b);
    assert(!d.acc_on && d.mag_on && d.errors==1); /* failed readback */
    f=recognised();f.fail_reg=0x20;d=imu_configure(b);
    assert(!d.acc_on && d.mag_on && d.errors==1);
}
static void test_data(void) {
    int16_t raw[3];
    uint8_t a[6]={0x80,0x3e,0x80,0xc1,0,0};Vec3 g=imu_accel(a,raw);
    assert(raw[0]==16000 && raw[1]==-16000 && raw[2]==0);near(g.x,1);near(g.y,-1);near(g.z,0);
    uint8_t m[6]={0x04,0x4c,0xfc,0x2c,0xfb,0xb4};Vec3 u=imu_mag(m,raw);
    assert(raw[0]==1100 && raw[1]==-1100 && raw[2]==-980);near(u.x,100);near(u.y,-100);near(u.z,-100);
    assert(!imu_mag_overflow(raw));raw[1]=-4096;assert(imu_mag_overflow(raw));
    assert(imu_s16(0x8000)==-32768 && imu_s16(0xffff)==-1 && imu_s16(0x7fff)==32767);
    assert(!imu_acc_clip((int16_t[3]){0,0,16000}));assert(imu_acc_clip((int16_t[3]){-32768,0,0}));
    Fake f=recognised();ImuBus b={&f,read_fake,write_fake};
    ImuReadout r=imu_read_acc(b);assert(r.ok && !r.fresh);
    f.a[0x27]=0x88;memcpy(f.a+0x28,a,6);r=imu_read_acc(b);
    assert(r.ok && r.fresh && r.status==0x88 && memcmp(r.bytes,a,6)==0);
    f.fail_reg=0xa8;r=imu_read_acc(b);assert(!r.ok && !r.fresh);f.fail_reg=-1;
    f.m[9]=1;memcpy(f.m+3,m,6);r=imu_read_mag(b);
    assert(r.ok && r.fresh && !r.locked && memcmp(r.bytes,m,6)==0);
    f.m[9]=2;r=imu_read_mag(b);assert(r.ok && r.locked && !r.fresh);
    f.fail_reg=3;r=imu_read_mag(b);assert(!r.ok && !r.fresh);
}
static void test_reference(void) {
    ImuStats s={0};for(unsigned i=0;i<100;++i) imu_stats_add(&s,(Vec3){i&1?0.01f:-0.01f,0,1});
    assert(imu_reference_ok(&s,false) && !imu_reference_ok(&s,true));near(s.mean.x,0);near(s.mean.z,1);
    Vec3 sd=imu_stats_std(&s);assert(sd.x>0.009f && sd.x<0.011f);
    near(imu_tilt((Vec3){0,0,1},(Vec3){0,0,1}),0);
    near(imu_tilt((Vec3){1,0,0},(Vec3){0,0,1}),90);
    near(imu_tilt((Vec3){0,0,-1},(Vec3){0,0,1}),180);
    near(imu_roll((Vec3){0,1,0}),90);near(imu_pitch((Vec3){1,0,0}),-90);
    assert(isnan(imu_tilt((Vec3){0},(Vec3){0,0,1})));
    s=(ImuStats){0};for(unsigned i=0;i<100;++i) imu_stats_add(&s,(Vec3){0});
    assert(!imu_reference_ok(&s,false));
    s=(ImuStats){0};for(unsigned i=0;i<100;++i) imu_stats_add(&s,(Vec3){i&1?0.1f:-0.1f,0,1});
    assert(!imu_reference_ok(&s,false));
    s=(ImuStats){0};for(unsigned i=0;i<99;++i) imu_stats_add(&s,(Vec3){0,0,1});
    assert(!imu_reference_ok(&s,false));
}
int main(void) {
    test_configuration();test_data();test_reference();
    puts("PASS: IDs/config readback/partial failures; endian/XZY/sign/scales; fresh/lock/failure semantics; tilt and stationary-reference rejection");
    return 0;
}
