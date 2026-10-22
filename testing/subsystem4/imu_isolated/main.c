/* Buddy 4 isolated USB IMU diagnostics. No RTOS, motion, ADC, servo or flash writes. */
#include <stdio.h>
#include <inttypes.h>
#include <limits.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "imu_driver.h"

enum { SDA_PIN=2,SCL_PIN=3,I2C_TIMEOUT_US=2000,POLL_US=5000,PRINT_US=200000 };
static ImuBus bus;
static ImuDevice device;
typedef struct {
    Vec3 value;int16_t raw[3];bool seen,io_ok,clip;
    uint8_t status;uint64_t last_us;uint32_t count,errors,max_gap_us;
} Sensor;
static Sensor acc,mag;
static uint32_t acc_overruns,mag_locks,mag_overflows,acc_clips,low_bits;
static uint32_t poll_max_us,late_polls,seq;
static uint64_t last_poll,rate_start;
static uint32_t rate_acc,rate_mag;
static ImuStats run_stats,cal_stats;
static bool streaming=true,reference_valid,cal_active,cal_bad;
static Vec3 reference;
static uint64_t cal_start;

static bool read_reg(void *ctx,uint8_t addr,uint8_t reg,uint8_t *data,size_t n) {
    (void)ctx;
    return i2c_write_timeout_us(i2c1,addr,&reg,1,true,I2C_TIMEOUT_US)==1 &&
           i2c_read_timeout_us(i2c1,addr,data,n,false,I2C_TIMEOUT_US)==(int)n;
}
static bool write_reg(void *ctx,uint8_t addr,uint8_t reg,uint8_t value) {
    (void)ctx;uint8_t b[2]={reg,value};
    return i2c_write_timeout_us(i2c1,addr,b,2,false,I2C_TIMEOUT_US)==2;
}
static int32_t age_ms(const Sensor *s,uint64_t now) {
    if(!s->seen) return -1;
    uint64_t age=(now-s->last_us)/1000;
    return age>INT32_MAX?INT32_MAX:(int32_t)age;
}
static bool acc_valid(uint64_t now) {
    return device.acc_on && acc.seen && acc.io_ok && !acc.clip && now-acc.last_us<=200000;
}
static bool mag_valid(uint64_t now) {
    return device.mag_on && mag.seen && mag.io_ok && !mag.clip && now-mag.last_us<=500000;
}
static void identity(void) {
    printf("DEVICE: A_addr=0x19 A_id=0x%02x expected=0x33 acc_cfg=%u | M_addr=0x1e M_id=%02x,%02x,%02x expected=48,34,33 mag_cfg=%u init_errors=%" PRIu32 "\n",
        device.acc_id,device.acc_on,device.mag_id[0],device.mag_id[1],device.mag_id[2],device.mag_on,device.errors);
    if(!device.acc_on || !device.mag_on)
        puts("CHECK: missing/unsupported ID or failed config readback; h/i remain usable. Send DEVICE + SCAN output. Do not infer a dead IMU from a missing address alone.");
}
static void help(void) {
    puts("\nBUDDY4 IMU ISOLATED v1 | GP8..11 held LOW | motors=OFF always");
    puts("GY-511 wiring: 3.3V/GND, SDA=GP2 SCL=GP3, I2C1=100kHz; USB console only.");
    puts("Supported: LSM303DLHC-compatible A_id33 + magnetic H43; parts tested independently.");
    puts("Acceleration: +/-2g HR,50Hz; magnetometer:+/-1.3gauss,30Hz; poll5ms; print5Hz.");
    puts("h=help/IDs; i=read-probe bus scan; r=reidentify/configure/reset counters/reference.");
    puts("c=keep still for100 fresh accel samples (~2s) to set RAM tilt reference; z=clear reference.");
    puts("p=summary since r/boot; s=pause streaming; g=resume streaming. No buttons needed.");
    puts("acc_g includes gravity; norm_g should be near1 stationary. roll/pitch use SENSOR axes.");
    puts("tilt_ref is angle from captured gravity direction, not yaw. Moving readings include acceleration.");
    puts("No gyro, heading calibration, hump-height estimator, motor control or offline flash logging.");
    puts("MAG units=uT factory nominal sensitivity, uncalibrated; chassis/magnets may distort/saturate.");
    puts("A_valid/M_valid include age/I2C/clip checks. No sample -> nan,age=-1. Pausing output still samples.");
    puts("Fresh sample counters/rates differ from5Hz printing. Diagnostic commands can interrupt sampling.");
    identity();
}
static void scan(void) {
    if(cal_active) { cal_active=false;puts("CAL cancelled by scan."); }
    puts("SCAN: I2C1 GP2/GP3 100kHz, 7-bit addresses (read probes, no config writes)");
    unsigned found=0;
    printf("BUS: SDA=%u SCL=%u before scan\n",gpio_get(SDA_PIN),gpio_get(SCL_PIN));
    for(uint8_t addr=8;addr<0x78;++addr) {
        uint8_t b;
        if(i2c_read_timeout_us(i2c1,addr,&b,1,false,I2C_TIMEOUT_US)==1) {
            printf("SCAN: ACK 0x%02x\n",addr);++found;
        }
    }
    printf("SCAN: found=%u expected A=0x19 M=0x1e; SDA=%u SCL=%u\n",found,gpio_get(SDA_PIN),gpio_get(SCL_PIN));
    /* A read probe can leave magnetic data locked. Drain all six axes. */
    if(device.mag_on) (void)imu_read_mag(bus);
}
static void configure(void) {
    device=imu_configure(bus);
    acc=(Sensor){0};mag=(Sensor){0};
    run_stats=(ImuStats){0};cal_stats=(ImuStats){0};cal_active=reference_valid=false;
    acc_overruns=mag_locks=mag_overflows=acc_clips=low_bits=0;
    poll_max_us=late_polls=seq=rate_acc=rate_mag=0;
    last_poll=rate_start=time_us_64();
    identity();
}
static void mark_sample(Sensor *s,uint64_t now) {
    if(s->seen) {
        uint64_t gap=now-s->last_us;
        if(gap>s->max_gap_us) s->max_gap_us=gap>UINT32_MAX?UINT32_MAX:(uint32_t)gap;
    }
    s->seen=true;s->last_us=now;++s->count;
}
static void finish_calibration(bool timeout) {
    cal_active=false;Vec3 sd=imu_stats_std(&cal_stats);
    if(!timeout && imu_reference_ok(&cal_stats,cal_bad)) {
        reference=cal_stats.mean;reference_valid=true;
        printf("CAL: OK samples=%" PRIu32 " mean_g=%.4f,%.4f,%.4f std_g=%.4f,%.4f,%.4f reference=RAM_ONLY\n",
            cal_stats.n,(double)reference.x,(double)reference.y,(double)reference.z,(double)sd.x,(double)sd.y,(double)sd.z);
    } else printf("CAL: REJECTED samples=%" PRIu32 " timeout=%u bad_io_clip_or_gap=%u norm=%.4f std_g=%.4f,%.4f,%.4f old_reference_retained=%u\n",
        cal_stats.n,timeout,cal_bad,(double)imu_norm(cal_stats.mean),(double)sd.x,(double)sd.y,(double)sd.z,reference_valid);
}
static void sample(uint64_t now) {
    uint64_t elapsed=now-last_poll;if(elapsed<POLL_US) return;
    last_poll=now;
    if(elapsed>poll_max_us) poll_max_us=elapsed>UINT32_MAX?UINT32_MAX:(uint32_t)elapsed;
    if(elapsed>10000) ++late_polls;
    if(device.acc_on) {
        ImuReadout r=imu_read_acc(bus);acc.status=r.status;acc.io_ok=r.ok;
        if(!r.ok) { ++acc.errors;if(cal_active) cal_bad=true; }
        else if(r.fresh) {
            uint64_t read_at=time_us_64();
            if(cal_active && acc.seen && read_at-acc.last_us>60000) cal_bad=true;
            acc.value=imu_accel(r.bytes,acc.raw);acc.clip=imu_acc_clip(acc.raw);
            mark_sample(&acc,read_at);
            if(r.status&0x80) ++acc_overruns;
            if(((uint16_t)acc.raw[0]|(uint16_t)acc.raw[1]|(uint16_t)acc.raw[2])&15u) ++low_bits;
            if(acc.clip) ++acc_clips;
            imu_stats_add(&run_stats,acc.value);
            if(cal_active) {
                if(acc.clip || (r.status&0x80)) cal_bad=true;
                imu_stats_add(&cal_stats,acc.value);
                if(cal_stats.n>=100) finish_calibration(false);
            }
        }
    }
    if(device.mag_on) {
        ImuReadout r=imu_read_mag(bus);mag.status=r.status;mag.io_ok=r.ok;
        if(!r.ok) ++mag.errors;
        if(r.locked) ++mag_locks;
        if(r.fresh) {
            mag.value=imu_mag(r.bytes,mag.raw);mag.clip=imu_mag_overflow(mag.raw);
            if(mag.clip) ++mag_overflows;
            mark_sample(&mag,time_us_64());
        }
    }
    if(cal_active && time_us_64()-cal_start>=3000000) finish_calibration(true);
}
static void summary(void) {
    Vec3 sd=imu_stats_std(&run_stats);
    printf("RAW: A_xyz16=%d,%d,%d (HR left-justified) M_xyz=%d,%d,%d statusA=0x%02x statusM=0x%02x\n",
        acc.raw[0],acc.raw[1],acc.raw[2],mag.raw[0],mag.raw[1],mag.raw[2],acc.status,mag.status);
    printf("SUMMARY: A_n=%" PRIu32 " M_n=%" PRIu32 " A_errors=%" PRIu32 " M_errors=%" PRIu32
        " A_overruns=%" PRIu32 " A_clips=%" PRIu32 " HR_lowbits_nonzero=%" PRIu32
        " M_locks=%" PRIu32 " M_overflows=%" PRIu32 " A_max_gap_ms=%.2f M_max_gap_ms=%.2f poll_max_ms=%.2f late_polls=%" PRIu32 " motors=OFF\n",
        acc.count,mag.count,acc.errors,mag.errors,acc_overruns,acc_clips,low_bits,mag_locks,mag_overflows,
        (double)(acc.max_gap_us/1000.0f),(double)(mag.max_gap_us/1000.0f),(double)(poll_max_us/1000.0f),late_polls);
    if(run_stats.n) printf("ACC_STATS: n=%" PRIu32 " mean_g=%.4f,%.4f,%.4f std_g=%.4f,%.4f,%.4f norm_min=%.4f norm_max=%.4f (includes all movement since r/boot)\n",
        run_stats.n,(double)run_stats.mean.x,(double)run_stats.mean.y,(double)run_stats.mean.z,
        (double)sd.x,(double)sd.y,(double)sd.z,(double)run_stats.norm_min,(double)run_stats.norm_max);
}
static void print_sample(uint64_t now) {
    bool av=acc_valid(now),mv=mag_valid(now);
    Vec3 a=acc.seen?acc.value:(Vec3){NAN,NAN,NAN};
    Vec3 m=mag.seen?mag.value:(Vec3){NAN,NAN,NAN};
    if(mag.clip) m=(Vec3){NAN,NAN,NAN};
    printf("IMU: seq=%" PRIu32 " ms=%" PRIu64 " A_valid=%u ageA=%" PRId32 " acc_g=%.3f,%.3f,%.3f norm=%.3f roll=%.1f pitch=%.1f tilt_ref=%.1f cal=%u busy=%u M_valid=%u ageM=%" PRId32 " mag_uT=%.1f,%.1f,%.1f A_n=%" PRIu32 " M_n=%" PRIu32 " errors=%" PRIu32 ",%" PRIu32 " motors=OFF\n",
        seq++,now/1000,av,age_ms(&acc,now),(double)a.x,(double)a.y,(double)a.z,(double)imu_norm(a),
        (double)(av?imu_roll(a):NAN),(double)(av?imu_pitch(a):NAN),
        (double)(av && reference_valid?imu_tilt(a,reference):NAN),reference_valid,cal_active,
        mv,age_ms(&mag,now),(double)m.x,(double)m.y,(double)m.z,acc.count,mag.count,acc.errors,mag.errors);
}
int main(void) {
    /* First application action: disable both motor bridges and keep them LOW.
       There is no motor PWM, motion-start command, encoder or line-sensor code. */
    for(uint p=8;p<=11;++p) { gpio_init(p);gpio_put(p,0);gpio_set_dir(p,GPIO_OUT); }
    i2c_init(i2c1,100000);gpio_set_function(SDA_PIN,GPIO_FUNC_I2C);gpio_set_function(SCL_PIN,GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);gpio_pull_up(SCL_PIN);
    stdio_init_all();bus=(ImuBus){NULL,read_reg,write_reg};
    sleep_ms(100);scan();configure();
    bool connected=false;uint64_t last_print=0;
    while(true) {
        sample(time_us_64());
        bool current=stdio_usb_connected();
        if(current && !connected) {
            help();rate_start=time_us_64();rate_acc=acc.count;rate_mag=mag.count;
        }
        connected=current;
        int ch=current?getchar_timeout_us(0):PICO_ERROR_TIMEOUT;
        if(ch=='h') help();
        else if(ch=='i') scan();
        else if(ch=='r') configure();
        else if(ch=='p') summary();
        else if(ch=='s') { streaming=false;puts("STREAM paused; sampling continues. g resumes."); }
        else if(ch=='g') { streaming=true;puts("STREAM resumed."); }
        else if(ch=='z') { reference_valid=cal_active=false;puts("CAL reference cleared (RAM only)."); }
        else if(ch=='c') {
            if(!acc_valid(time_us_64())) puts("CAL rejected: need fresh valid acceleration first; send h/i output.");
            else { cal_active=true;cal_bad=false;cal_stats=(ImuStats){0};cal_start=time_us_64();puts("CAL: keep robot STILL for100 fresh samples (~2s), timeout3s. This sets a gravity reference only."); }
        }
        uint64_t now=time_us_64();
        if(current && streaming && now-last_print>=PRINT_US) { last_print=now;print_sample(now); }
        if(now-rate_start>=1000000) {
            if(current && streaming) {
                float seconds=(now-rate_start)/1000000.0f;
                printf("RATE: interval_ms=%" PRIu64 " A_Hz=%.1f M_Hz=%.1f overruns=%" PRIu32 " mag_overflows=%" PRIu32 "\n",
                    (now-rate_start)/1000,(double)((acc.count-rate_acc)/seconds),(double)((mag.count-rate_mag)/seconds),acc_overruns,mag_overflows);
            }
            rate_start=now;rate_acc=acc.count;rate_mag=mag.count;
        }
        sleep_ms(1);
    }
}
