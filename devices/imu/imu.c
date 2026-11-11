#include "imu.h"
#include "spi.h"            /* hspi2 (CubeMX) */
#include "spi_dev.h"
#include "lsm6dsv80x_reg.h" /* ST platform-independent driver (third_party/) */

/* ---- Configuration: change here (and the matching conversion below) ---- */
#define IMU_XL_ODR     LSM6DSV80X_ODR_AT_480Hz
#define IMU_XL_FS      LSM6DSV80X_16g          /* low-g accelerometer range  */
#define IMU_HG_ODR     LSM6DSV80X_HG_XL_ODR_AT_480Hz
#define IMU_HG_FS      LSM6DSV80X_80g          /* high-g accelerometer range */
#define IMU_GY_ODR     LSM6DSV80X_ODR_AT_480Hz
#define IMU_GY_FS      LSM6DSV80X_2000dps      /* use 4000dps if the rocket spins faster */
#define IMU_BOOT_MS    10u
/* Same settings as plain numbers, recorded in every log. Keep in step with the enums above. */
#define IMU_XL_FS_G      16
#define IMU_HG_FS_G      80
#define IMU_GY_FS_DPS    2000
#define IMU_XL_ODR_HZ    480
#define IMU_HG_ODR_HZ    480
#define IMU_GY_ODR_HZ    480

/* LSM6DSV80X SPI: read = address | 0x80, write = address as-is. */
static spi_dev_t imu_bus = {
    .hspi = &hspi2, .cs_port = CS_IMU_GPIO_Port, .cs_pin = CS_IMU_Pin,
    .read_flags = 0x80, .write_flags = 0x00,
};

static void delay_ms(uint32_t ms) { HAL_Delay(ms); }

static stmdev_ctx_t ctx = {
    .write_reg = spi_dev_write,
    .read_reg  = spi_dev_read,
    .mdelay    = delay_ms,
    .handle    = &imu_bus,
};

static uint8_t last_id;

#define CHK(call)  do { if ((call) != 0) return DEV_ERR_BUS; } while (0)

dev_err_t imu_init(void)
{
    last_id = 0;
    HAL_Delay(IMU_BOOT_MS);                       /* sensor boot time */

    CHK(lsm6dsv80x_device_id_get(&ctx, &last_id));
    if (last_id != LSM6DSV80X_ID)
        return DEV_ERR_ID;

    CHK(lsm6dsv80x_sw_por(&ctx));                 /* known starting state */
    CHK(lsm6dsv80x_block_data_update_set(&ctx, PROPERTY_ENABLE));   /* coherent multi-byte reads */

    CHK(lsm6dsv80x_xl_setup(&ctx, IMU_XL_ODR, LSM6DSV80X_XL_HIGH_PERFORMANCE_MD));
    CHK(lsm6dsv80x_hg_xl_data_rate_set(&ctx, IMU_HG_ODR, 1));       /* 1 = output in the data registers */
    CHK(lsm6dsv80x_gy_setup(&ctx, IMU_GY_ODR, LSM6DSV80X_GY_HIGH_PERFORMANCE_MD));

    CHK(lsm6dsv80x_xl_full_scale_set(&ctx, IMU_XL_FS));
    CHK(lsm6dsv80x_hg_xl_full_scale_set(&ctx, IMU_HG_FS));
    CHK(lsm6dsv80x_gy_full_scale_set(&ctx, IMU_GY_FS));

    /* TODO later: digital filters, FIFO, data-ready interrupt on INT1/INT2. */
    return DEV_OK;
}

void imu_get_cfg(imu_cfg_t *c)
{
    c->xl_fs_g = IMU_XL_FS_G; c->hg_fs_g = IMU_HG_FS_G; c->gy_fs_dps = IMU_GY_FS_DPS;
    c->xl_odr_hz = IMU_XL_ODR_HZ; c->hg_odr_hz = IMU_HG_ODR_HZ; c->gy_odr_hz = IMU_GY_ODR_HZ;
}

uint8_t imu_who_am_i(void) { return last_id; }

dev_err_t imu_poll(imu_raw_t *s)
{
    lsm6dsv80x_data_ready_t drdy;
    int got = 0;

    if (lsm6dsv80x_flag_data_ready_get(&ctx, &drdy) != 0)
        return DEV_ERR_BUS;

    if (drdy.drdy_xl) {
        if (lsm6dsv80x_acceleration_raw_get(&ctx, s->xl_raw) != 0) return DEV_ERR_BUS;
        s->n_xl++; got = 1;
    }
    if (drdy.drdy_hgxl) {
        if (lsm6dsv80x_hg_acceleration_raw_get(&ctx, s->hg_raw) != 0) return DEV_ERR_BUS;
        s->n_hg++; got = 1;
    }
    if (drdy.drdy_gy) {
        if (lsm6dsv80x_angular_rate_raw_get(&ctx, s->gy_raw) != 0) return DEV_ERR_BUS;
        s->n_gy++; got = 1;
    }
    return got ? DEV_OK : DEV_NO_DATA;
}

/* Conversions: keep these in step with IMU_*_FS above. */
float imu_xl_mg(int16_t raw)   { return lsm6dsv80x_from_fs16_to_mg(raw); }
float imu_hg_mg(int16_t raw)   { return lsm6dsv80x_from_fs80_to_mg(raw); }
float imu_gy_mdps(int16_t raw) { return lsm6dsv80x_from_fs2000_to_mdps(raw); }
