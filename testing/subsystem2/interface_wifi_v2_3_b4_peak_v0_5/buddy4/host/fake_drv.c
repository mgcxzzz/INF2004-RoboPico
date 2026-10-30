/* host fake for the LSM303 driver: synthetic accelerometer FIFO */
#include <math.h>
#include "../subsystem4/drv_lsm303.h"
int fake_lat_mg = 0, fake_up_mg = 1000, fake_fwd_mg = 0;
bool fake_ok = true;
/* small deterministic noise (+-3 mg): a real sensor never repeats exact means,
 * and imu_proc faults a stuck output (IMU_FAULT_INVALID). */
static unsigned lcg = 12345u;
static int16_t jit(void) { lcg = lcg * 1103515245u + 12345u; return (int16_t)((int)((lcg >> 16) % 7u) - 3); }
bool b4_drv_init(b4_chip_t *c, bool *m) { *c = B4_CHIP_DLHC; *m = false; return fake_ok; }
bool b4_drv_read_fifo(b4_raw_t *s, uint8_t max, uint8_t *n, bool *ovr) {
    uint8_t k = max < 8 ? max : 8; *ovr = false;
    for (uint8_t i = 0; i < k; i++) { int16_t v[3] = {0,0,0};
        v[B4_AXIS_FWD] = (int16_t)(fake_fwd_mg * B4_SIGN_FWD + jit()); v[B4_AXIS_UP] = (int16_t)(fake_up_mg * B4_SIGN_UP + jit());
        v[3 - B4_AXIS_FWD - B4_AXIS_UP] = (int16_t)(fake_lat_mg + jit());
        s[i].x = v[0]; s[i].y = v[1]; s[i].z = v[2]; }
    *n = k; return fake_ok; }
bool b4_drv_read_mag(b4_raw_t *m) { (void)m; return false; }
bool b4_i2c_read_reg(uint8_t a, uint8_t r, uint8_t *b, uint32_t l) { (void)a;(void)r;(void)b;(void)l; return true; }
bool b4_i2c_write_reg(uint8_t a, uint8_t r, uint8_t v) { (void)a;(void)r;(void)v; return true; }
