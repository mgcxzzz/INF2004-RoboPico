#ifndef IMU_ISOLATED_LOGIC_H
#define IMU_ISOLATED_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Never send NaN/Inf to Pico's small floating-point formatter. Missing, stale,
   clipped and uncalibrated values are explicitly textual NA. */
static inline void imu_format(char out[24],float v,bool valid,unsigned precision) {
    if(valid && isfinite(v)) (void)snprintf(out,24,"%.*f",(int)precision,(double)v);
    else strcpy(out,"NA");
}

typedef struct { float x,y,z; } Vec3;
typedef struct { uint32_t n;Vec3 mean,m2;float norm_min,norm_max; } ImuStats;
static inline int16_t imu_s16(uint16_t u) {
    return (int16_t)((u&0x8000u)?(int32_t)u-65536:(int32_t)u);
}
static inline int16_t imu_le(const uint8_t *p) { return imu_s16((uint16_t)(p[0]|((uint16_t)p[1]<<8))); }
static inline int16_t imu_be(const uint8_t *p) { return imu_s16((uint16_t)(((uint16_t)p[0]<<8)|p[1])); }
/* DLHC HR acceleration: signed 12-bit samples left justified in little-endian
   16-bit pairs; +/-2g sensitivity is 1mg per 12-bit count. */
static inline Vec3 imu_accel(const uint8_t b[6],int16_t raw[3]) {
    for(unsigned j=0;j<3;++j) raw[j]=imu_le(b+2*j);
    return (Vec3){raw[0]/16000.0f,raw[1]/16000.0f,raw[2]/16000.0f};
}
/* DLHC magnetic burst is BIG endian X,Z,Y, unlike acceleration. */
static inline Vec3 imu_mag(const uint8_t b[6],int16_t raw[3]) {
    raw[0]=imu_be(b);raw[1]=imu_be(b+4);raw[2]=imu_be(b+2);
    return (Vec3){raw[0]/11.0f,raw[1]/11.0f,raw[2]/9.8f}; /* uT at +/-1.3gauss */
}
static inline bool imu_mag_overflow(const int16_t raw[3]) {
    return raw[0]==-4096 || raw[1]==-4096 || raw[2]==-4096;
}
static inline bool imu_acc_clip(const int16_t raw[3]) {
    return raw[0]>=32640 || raw[0]<=-32640 || raw[1]>=32640 || raw[1]<=-32640 || raw[2]>=32640 || raw[2]<=-32640;
}
static inline float imu_norm(Vec3 a) { return sqrtf(a.x*a.x+a.y*a.y+a.z*a.z); }
static inline float imu_roll(Vec3 a) { return atan2f(a.y,a.z)*57.2957795131f; }
static inline float imu_pitch(Vec3 a) { return atan2f(-a.x,hypotf(a.y,a.z))*57.2957795131f; }
static inline float imu_tilt(Vec3 a,Vec3 reference) {
    float n=imu_norm(a)*imu_norm(reference);
    if(n<0.01f) return NAN;
    float dot=(a.x*reference.x+a.y*reference.y+a.z*reference.z)/n;
    return acosf(fminf(1,fmaxf(-1,dot)))*57.2957795131f;
}
static inline void imu_stats_add(ImuStats *s,Vec3 v) {
    float norm=imu_norm(v);++s->n;
    if(s->n==1) s->norm_min=s->norm_max=norm;
    s->norm_min=fminf(s->norm_min,norm);s->norm_max=fmaxf(s->norm_max,norm);
    float dx=v.x-s->mean.x,dy=v.y-s->mean.y,dz=v.z-s->mean.z;
    s->mean.x+=dx/s->n;s->mean.y+=dy/s->n;s->mean.z+=dz/s->n;
    s->m2.x+=dx*(v.x-s->mean.x);s->m2.y+=dy*(v.y-s->mean.y);s->m2.z+=dz*(v.z-s->mean.z);
}
static inline Vec3 imu_stats_std(const ImuStats *s) {
    if(s->n<2) return (Vec3){NAN,NAN,NAN};
    return (Vec3){sqrtf(fmaxf(0,s->m2.x/(s->n-1))),sqrtf(fmaxf(0,s->m2.y/(s->n-1))),sqrtf(fmaxf(0,s->m2.z/(s->n-1)))};
}
/* A stationary reference only, not six-face scale/bias or compass calibration. */
static inline bool imu_reference_ok(const ImuStats *s,bool bad) {
    Vec3 sd=imu_stats_std(s);float n=imu_norm(s->mean);
    return !bad && s->n>=100 && n>=0.85f && n<=1.15f && sd.x<=0.02f && sd.y<=0.02f && sd.z<=0.02f;
}
#endif
