#ifndef SD_SPI_H
#define SD_SPI_H
/*
 * SD / SD NAND (ZDSD16G) driver in SPI mode on SPI3.
 * Pins: SCK PC10, MISO PC11, MOSI PC12, chip select CS_FLASH (PA15).
 *
 * Needs in CubeMX: SPI3 "Hardware NSS Signal" = Disable (software chip select)
 * and CS_FLASH as a GPIO output with initial level High.
 * Blocking, polled transfers. Not for use in interrupts.
 */
#include <stdint.h>
#include "devices.h"

typedef struct {
    uint8_t  v2;          /* 1 = SD spec v2.0 or newer card                           */
    uint8_t  ccs;         /* 1 = block addressing (SDHC), 0 = byte addressing (SDSC)  */
    uint32_t sectors;     /* capacity in 512-byte sectors, from the CSD               */
    uint8_t  cid[16];
    uint8_t  csd[16];
    uint8_t  last_r1;     /* last R1 response seen during init, for debugging         */
} sd_info_t;

dev_err_t sd_init(sd_info_t *info);
dev_err_t sd_read_block (uint32_t sector, uint8_t *buf);          /* buf: 512 bytes */
dev_err_t sd_write_block(uint32_t sector, const uint8_t *buf);    /* buf: 512 bytes, waits until programmed */

/* Non-blocking write: returns once the card has accepted the data; the card then programs it in
 * the background. Call sd_ready() until it returns 1 before the next SD operation. */
dev_err_t sd_write_block_nb(uint32_t sector, const uint8_t *buf);
int       sd_ready(void);

/* Writes a known pattern to `sector`, reads it back, compares. DESTROYS that sector. */
dev_err_t sd_selftest(uint32_t sector);

#endif
