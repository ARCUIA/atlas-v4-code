#ifndef IMU_H
#define IMU_H
/* LSM6DSV80X wrapper (SPI2, CS_IMU). */
#include <stdint.h>
#include "devices.h"

/* Latest raw readings. imu_poll() only overwrites a channel when that channel
 * has a new sample, so the struct always holds the most recent value of each. */
typedef struct {
    int16_t  xl_raw[3];    /* low-g accelerometer  */
    int16_t  hg_raw[3];    /* high-g accelerometer */
    int16_t  gy_raw[3];    /* gyroscope            */
    uint32_t n_xl, n_hg, n_gy;   /* samples seen since init, to check data rates */
} imu_raw_t;

typedef struct { uint16_t xl_fs_g, hg_fs_g, gy_fs_dps, xl_odr_hz, hg_odr_hz, gy_odr_hz; } imu_cfg_t;
void imu_get_cfg(imu_cfg_t *c);        /* what imu_init() configured, for the log header */

dev_err_t imu_init(void);
uint8_t   imu_who_am_i(void);               /* raw WHO_AM_I byte last read */
dev_err_t imu_poll(imu_raw_t *s);           /* DEV_OK if anything new, DEV_NO_DATA if not */

/* Raw counts -> physical units. They match the full scales set in imu.c. */
float imu_xl_mg(int16_t raw);
float imu_hg_mg(int16_t raw);
float imu_gy_mdps(int16_t raw);

#endif
