#ifndef IMU_ISOLATED_DRIVER_H
#define IMU_ISOLATED_DRIVER_H
#include <stddef.h>
#include "imu_logic.h"
enum { IMU_A=0x19, IMU_M=0x1e };
typedef bool (*ImuRead)(void *,uint8_t,uint8_t,uint8_t *,size_t);
typedef bool (*ImuWrite)(void *,uint8_t,uint8_t,uint8_t);
typedef struct { void *ctx;ImuRead read;ImuWrite write; } ImuBus;
typedef struct {
    bool acc_id_ok,mag_id_ok,acc_on,mag_on;
    uint8_t acc_id,mag_id[3];
    uint32_t errors;
} ImuDevice;
static inline bool imu_write_check(ImuBus b,uint8_t addr,uint8_t reg,uint8_t v) {
    uint8_t got=0;
    return b.write(b.ctx,addr,reg,v) && b.read(b.ctx,addr,reg,&got,1) && got==v;
}
static inline ImuDevice imu_configure(ImuBus b) {
    ImuDevice d={0};
    if(b.read(b.ctx,IMU_A,0x0f,&d.acc_id,1)) d.acc_id_ok=d.acc_id==0x33;
    else ++d.errors;
    /* H43 is the documented magnetic identity. 0x33 is also used by other
       compatible ST accelerometers: report compatibility, not unique identity. */
    if(b.read(b.ctx,IMU_M,0x0a,d.mag_id,3)) d.mag_id_ok=memcmp(d.mag_id,"H43",3)==0;
    else ++d.errors;
    if(d.acc_id_ok) {
        /* Power down first; disable FIFO, high-pass and interrupt routes;
           enable block update and HR +/-2g, then 50Hz all three axes. */
        d.acc_on=imu_write_check(b,IMU_A,0x20,0x07) &&
                 imu_write_check(b,IMU_A,0x21,0x00) &&
                 imu_write_check(b,IMU_A,0x22,0x00) &&
                 imu_write_check(b,IMU_A,0x24,0x00) &&
                 imu_write_check(b,IMU_A,0x25,0x00) &&
                 imu_write_check(b,IMU_A,0x23,0x88) &&
                 imu_write_check(b,IMU_A,0x20,0x47);
        if(!d.acc_on) ++d.errors;
    }
    if(d.mag_id_ok) {
        d.mag_on=imu_write_check(b,IMU_M,0x00,0x14) && /* 30Hz, temperature off */
                 imu_write_check(b,IMU_M,0x01,0x20) && /* +/-1.3gauss */
                 imu_write_check(b,IMU_M,0x02,0x00);   /* continuous */
        if(!d.mag_on) ++d.errors;
    }
    return d;
}
typedef struct { bool ok,fresh,locked;uint8_t status,bytes[6]; } ImuReadout;
static inline ImuReadout imu_read_acc(ImuBus b) {
    ImuReadout r={0};
    r.ok=b.read(b.ctx,IMU_A,0x27,&r.status,1);
    if(r.ok && (r.status&0x08)) {
        r.ok=b.read(b.ctx,IMU_A,0x28|0x80,r.bytes,6); /* accel auto-increment */
        r.fresh=r.ok;
    }
    return r;
}
static inline ImuReadout imu_read_mag(ImuBus b) {
    ImuReadout r={0};
    r.ok=b.read(b.ctx,IMU_M,0x09,&r.status,1);r.locked=(r.status&2)!=0;
    /* Drain a partially-read locked set after an interrupted transaction.
       A lock-only drain is NOT counted as a new sample. */
    if(r.ok && (r.status&3)) {
        r.ok=b.read(b.ctx,IMU_M,0x03,r.bytes,6); /* magnetic pointer increments itself */
        r.fresh=r.ok && (r.status&1);
    }
    return r;
}
#endif
