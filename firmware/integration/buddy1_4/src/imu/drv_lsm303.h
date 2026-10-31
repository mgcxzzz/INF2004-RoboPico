/**
 * @file    drv_lsm303.h
 * @brief   Buddy 4: GY-511 driver. LSM303 accelerometer (and, as a stretch
 *          goal, magnetometer) on I2C1, GP2 = SDA, GP3 = SCL.
 *
 * The bus is driven directly through the RP2040 I2C1 registers, polled
 * with a microsecond timeout on every wait (B4_I2C_TIMEOUT_US), so an
 * unplugged or stuck sensor gives an error in about 2 ms and never hangs
 * the task. IMU Terrain is the only user of I2C1 (Week 6 Section 3), so
 * there is no bus lock. Call these from the IMU task only.
 *
 * Both GY-511 variants are recognised: LSM303DLHC (the usual one) and
 * LSM303AGR. Their accelerometers are configured the same way.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdbool.h>
#include <stdint.h>

#include "src/imu/b4_config.h"

typedef enum {
    B4_CHIP_NONE = 0,
    B4_CHIP_DLHC, /* LSM303DLHC */
    B4_CHIP_AGR   /* LSM303AGR  */
} b4_chip_t;

/* One accelerometer sample in the SENSOR's own axes, mg. */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} b4_raw_t;

/* Set up I2C1 and the sensor: 400 Hz, +-4 g, high resolution, FIFO in
 * stream mode. Returns false if the accelerometer does not answer or has
 * the wrong ID. *p_chip says which one was found; *p_b_mag whether the
 * magnetometer answered too (not needed for any requirement). */
bool b4_drv_init(b4_chip_t *p_chip, bool *p_b_mag);

/* Empty the FIFO: up to max samples (oldest first) into p_s. *p_n is the
 * number read (0 = nothing new), *p_b_overrun true if the FIFO had
 * overflowed (samples were lost). Returns false on an I2C error. */
bool b4_drv_read_fifo(b4_raw_t *p_s, uint8_t max, uint8_t *p_n, bool *p_b_overrun);

/* Raw magnetometer reading (X, Y, Z, sensor axes, LSB). */
bool b4_drv_read_mag(b4_raw_t *p_m);

/* Low level, also used by the self-test. */
bool b4_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint32_t len);
bool b4_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

#endif /* DRV_LSM303_H */
