#ifndef SPI_DEV_H
#define SPI_DEV_H
/*
 * "Read/write registers over SPI with a GPIO chip select."
 * ST's drivers call function pointers to talk to the chip; these are those
 * functions. Each sensor passes its own spi_dev_t as the driver's `handle`.
 * Blocking, polled transfers. Do not call from an interrupt.
 */
#include <stdint.h>
#include "main.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint8_t            read_flags;   /* OR'd into the register address on reads  */
    uint8_t            write_flags;  /* OR'd into the register address on writes */
} spi_dev_t;

/* Signatures match ST's stmdev_read_ptr / stmdev_write_ptr. Return 0 = OK. */
int32_t spi_dev_read (void *handle, uint8_t reg, uint8_t *buf, uint16_t len);
int32_t spi_dev_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len);

#endif
