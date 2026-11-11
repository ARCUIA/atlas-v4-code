#include "spi_dev.h"

#define SPI_TIMEOUT_MS 10u

int32_t spi_dev_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    const spi_dev_t *d = (const spi_dev_t *)handle;
    uint8_t cmd = (uint8_t)(reg | d->read_flags);
    int32_t rc = 0;

    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);          /* select   */
    if (HAL_SPI_Transmit(d->hspi, &cmd, 1, SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_Receive(d->hspi, buf, len, SPI_TIMEOUT_MS) != HAL_OK)
        rc = -1;
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);            /* deselect */
    return rc;
}

int32_t spi_dev_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    const spi_dev_t *d = (const spi_dev_t *)handle;
    uint8_t cmd = (uint8_t)(reg | d->write_flags);
    int32_t rc = 0;

    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(d->hspi, &cmd, 1, SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_Transmit(d->hspi, (uint8_t *)buf, len, SPI_TIMEOUT_MS) != HAL_OK)
        rc = -1;
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
    return rc;
}
