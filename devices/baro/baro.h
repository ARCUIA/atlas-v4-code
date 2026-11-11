#ifndef BARO_H
#define BARO_H
/* LPS22DF barometer wrapper (SPI2, CS_BARO). */
#include <stdint.h>
#include "devices.h"

typedef struct {
    int32_t  pressure_raw;   /* raw counts, as read from the chip */
    int16_t  temp_raw;
    float    pressure_hpa;
    float    temp_c;
    uint32_t n_samples;      /* samples seen since init, to check the data rate */
} baro_sample_t;

dev_err_t baro_init(void);
uint8_t   baro_who_am_i(void);
uint16_t  baro_odr_x100(void);           /* configured data rate in Hz x 100, for the log header */
dev_err_t baro_poll(baro_sample_t *s);   /* DEV_OK if new data, DEV_NO_DATA if not */

#endif
