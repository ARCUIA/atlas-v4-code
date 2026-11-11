#include "baro.h"
#include "spi.h"
#include "spi_dev.h"
#include "lps22df_reg.h"

/* ---- Configuration ---- */
#define BARO_ODR   LPS22DF_25Hz         /* check the datasheet for valid ODR/averaging pairs */
#define BARO_AVG   LPS22DF_16_AVG
#define BARO_LPF   LPS22DF_LPF_ODR_DIV_4
#define BARO_BOOT_MS 10u
#define BARO_ODR_HZ_X100 2500           /* same rate as BARO_ODR, as a plain number */

/* LPS22DF SPI: read = address | 0x80, write = address as-is. */
static spi_dev_t baro_bus = {
    .hspi = &hspi2, .cs_port = CS_BARO_GPIO_Port, .cs_pin = CS_BARO_Pin,
    .read_flags = 0x80, .write_flags = 0x00,
};

static void delay_ms(uint32_t ms) { HAL_Delay(ms); }

static stmdev_ctx_t ctx = {
    .write_reg = spi_dev_write,
    .read_reg  = spi_dev_read,
    .mdelay    = delay_ms,
    .handle    = &baro_bus,
};

static uint8_t last_id;

#define CHK(call)  do { if ((call) != 0) return DEV_ERR_BUS; } while (0)

dev_err_t baro_init(void)
{
    lps22df_id_t id = { 0 };
    lps22df_bus_mode_t bus = { 0 };
    lps22df_md_t md = { 0 };

    last_id = 0;
    HAL_Delay(BARO_BOOT_MS);

    CHK(lps22df_id_get(&ctx, &id));
    last_id = id.whoami;
    if (last_id != LPS22DF_ID)
        return DEV_ERR_ID;

    /* Same sequence as ST's example: boot, reset, then driver-ready defaults. */
    CHK(lps22df_init_set(&ctx, LPS22DF_BOOT));
    CHK(lps22df_init_set(&ctx, LPS22DF_RESET));
    CHK(lps22df_init_set(&ctx, LPS22DF_DRV_RDY));      /* sets BDU and register auto-increment */

    bus.filter    = LPS22DF_FILTER_AUTO;
    bus.interface = LPS22DF_SEL_BY_HW;                 /* SPI chosen by the CS pin */
    CHK(lps22df_bus_mode_set(&ctx, &bus));

    md.odr = BARO_ODR;
    md.avg = BARO_AVG;
    md.lpf = BARO_LPF;
    CHK(lps22df_mode_set(&ctx, &md));

    /* TODO later: data-ready interrupt on BARO_INT. */
    return DEV_OK;
}

uint8_t baro_who_am_i(void) { return last_id; }
uint16_t baro_odr_x100(void) { return BARO_ODR_HZ_X100; }

dev_err_t baro_poll(baro_sample_t *s)
{
    lps22df_all_sources_t src;
    lps22df_data_t d;

    if (lps22df_all_sources_get(&ctx, &src) != 0)
        return DEV_ERR_BUS;
    if (!(src.drdy_pres || src.drdy_temp))
        return DEV_NO_DATA;
    if (lps22df_data_get(&ctx, &d) != 0)
        return DEV_ERR_BUS;

    s->pressure_raw = d.pressure.raw;
    s->temp_raw     = d.heat.raw;
    s->pressure_hpa = d.pressure.hpa;
    s->temp_c       = d.heat.deg_c;
    s->n_samples++;
    return DEV_OK;
}
