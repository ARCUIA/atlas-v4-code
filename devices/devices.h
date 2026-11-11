#ifndef DEVICES_H
#define DEVICES_H
/* Shared definitions for the sensor wrappers in devices/. */
#include <stdint.h>

typedef enum {
    DEV_OK       =  0,
    DEV_NO_DATA  =  1,   /* not an error: nothing new since the last poll */
    DEV_ERR_BUS  = -1,   /* an SPI transfer failed                        */
    DEV_ERR_ID   = -2,   /* WHO_AM_I did not match the expected chip      */
    DEV_ERR_TIMEOUT = -3,/* device did not answer in time                 */
    DEV_ERR_PROTO   = -4,/* unexpected response or data mismatch          */
    DEV_ERR_CFG     = -5 /* CubeMX/pin configuration is not what the driver needs */
} dev_err_t;

const char *dev_err_str(dev_err_t e);

/* Drive every SPI2 chip select high (deselected). Call before any SPI traffic. */
void devices_cs_idle(void);

#endif
