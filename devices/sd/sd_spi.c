#include "sd_spi.h"
#include "spi.h"            /* hspi3 (CubeMX) */
#include "main.h"
#include <string.h>

#define SD_BLOCK        512u
#define INIT_TIMEOUT_MS 1000u    /* ACMD41 can take a while after power-up */
#define TOKEN_TIMEOUT_MS 200u
#define BUSY_TIMEOUT_MS  500u    /* NAND programming can be slow */

static uint8_t ccs;                       /* 1 = block addressing */
static uint8_t g_busy;                    /* 1 = write accepted, card still programming, CS held low */
static uint8_t ff_block[SD_BLOCK];        /* 0xFF bytes clocked out while reading */
static uint8_t wr_buf[SD_BLOCK], rd_buf[SD_BLOCK];   /* used by the self-test */

/* ---------------- low level ---------------- */
static uint8_t spi_byte(uint8_t tx)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi3, &tx, &rx, 1, 10);
    return rx;
}

static void cs_low(void)  { HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_RESET); }
static void deselect(void)
{
    HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_SET);
    spi_byte(0xFF);                        /* extra clocks let the card release MISO */
}

static void set_prescaler(uint32_t ps)
{
    hspi3.Init.BaudRatePrescaler = ps;
    HAL_SPI_Init(&hspi3);
}

/* The card holds MISO low while busy and releases it (0xFF) when ready. */
static int wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (spi_byte(0xFF) != 0xFF)
        if (HAL_GetTick() - t0 > timeout_ms) return -1;
    return 0;
}

/* Finish any write that is still in progress (blocking). */
static void settle(void)
{
    if (g_busy) {
        (void)wait_ready(BUSY_TIMEOUT_MS);
        deselect();
        g_busy = 0;
    }
}

/* Sends a command frame, returns the R1 response (0xFF = no answer). CS must be low. */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r1 = 0xFF;
    (void)wait_ready(BUSY_TIMEOUT_MS);
    spi_byte((uint8_t)(0x40u | cmd));
    spi_byte((uint8_t)(arg >> 24));
    spi_byte((uint8_t)(arg >> 16));
    spi_byte((uint8_t)(arg >> 8));
    spi_byte((uint8_t)arg);
    spi_byte(crc);                         /* only CMD0 and CMD8 need a real CRC in SPI mode */
    for (int i = 0; i < 10; i++) {
        r1 = spi_byte(0xFF);
        if (!(r1 & 0x80u)) break;
    }
    return r1;
}

/* Waits for the data-start token 0xFE. Returns the token, or 0xFF on timeout. */
static uint8_t wait_token(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t t;
    do {
        t = spi_byte(0xFF);
        if (t != 0xFF) return t;
    } while (HAL_GetTick() - t0 <= timeout_ms);
    return 0xFF;
}

/* CMD9 (CSD) / CMD10 (CID): 16-byte data block. */
static dev_err_t read_register(uint8_t cmd, uint8_t *out16)
{
    dev_err_t rc = DEV_ERR_PROTO;
    cs_low();
    if (sd_cmd(cmd, 0, 0xFF) == 0x00 && wait_token(TOKEN_TIMEOUT_MS) == 0xFE) {
        for (int i = 0; i < 16; i++) out16[i] = spi_byte(0xFF);
        spi_byte(0xFF); spi_byte(0xFF);    /* CRC, ignored */
        rc = DEV_OK;
    }
    deselect();
    return rc;
}

static uint32_t sectors_from_csd(const uint8_t *csd)
{
    if ((csd[0] >> 6) == 1) {              /* CSD v2.0: SDHC/SDXC */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
        return (c_size + 1u) * 1024u;
    } else {                               /* CSD v1.0: standard capacity */
        uint32_t read_bl_len = csd[5] & 0x0F;
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        uint32_t mult   = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
        uint64_t bytes  = ((uint64_t)(c_size + 1u)) << (mult + 2u) << read_bl_len;
        return (uint32_t)(bytes / SD_BLOCK);
    }
}

/* ---------------- public API ---------------- */
dev_err_t sd_init(sd_info_t *info)
{
    uint8_t r1, r7[4];
    uint32_t t0;
    memset(info, 0, sizeof *info);
    memset(ff_block, 0xFF, sizeof ff_block);
    g_busy = 0;

    /* Guard against the usual CubeMX mistake: chip select must be a plain GPIO output. */
    if (hspi3.Init.NSS != SPI_NSS_SOFT) return DEV_ERR_CFG;
    if (((CS_FLASH_GPIO_Port->MODER >> (2u * (uint32_t)__builtin_ctz(CS_FLASH_Pin))) & 3u) != 1u)
        return DEV_ERR_CFG;

    set_prescaler(SPI_BAUDRATEPRESCALER_128);          /* 42 MHz / 128 = 328 kHz, under the 400 kHz init limit */
    HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
    for (int i = 0; i < 10; i++) spi_byte(0xFF);       /* 80 clocks with CS high: puts the card in native mode */

    /* CMD0: reset into SPI mode. CS low while sending it selects SPI. Expect R1 = 0x01 (idle). */
    for (int tries = 0; tries < 5; tries++) {
        cs_low(); r1 = sd_cmd(0, 0, 0x95); deselect();
        info->last_r1 = r1;
        if (r1 == 0x01) break;
        HAL_Delay(10);
    }
    if (r1 != 0x01) return (r1 == 0xFF) ? DEV_ERR_TIMEOUT : DEV_ERR_PROTO;

    /* CMD8: interface condition. A v2 card echoes the voltage range and check pattern. */
    cs_low(); r1 = sd_cmd(8, 0x000001AAu, 0x87);
    for (int i = 0; i < 4; i++) r7[i] = spi_byte(0xFF);
    deselect();
    info->last_r1 = r1;
    if (r1 == 0x01) {
        if (r7[2] != 0x01 || r7[3] != 0xAA) return DEV_ERR_PROTO;
        info->v2 = 1;
    } else if (!(r1 & 0x04)) {
        return DEV_ERR_PROTO;              /* neither v2 nor "illegal command" (v1 card) */
    }

    /* ACMD41 (CMD55 + CMD41) until the card leaves idle. HCS bit tells a v2 card we support SDHC. */
    t0 = HAL_GetTick();
    do {
        cs_low(); r1 = sd_cmd(55, 0, 0xFF); deselect();
        cs_low(); r1 = sd_cmd(41, info->v2 ? 0x40000000u : 0u, 0xFF); deselect();
        info->last_r1 = r1;
        if (r1 == 0x00) break;
    } while (HAL_GetTick() - t0 < INIT_TIMEOUT_MS);
    if (r1 != 0x00) return DEV_ERR_TIMEOUT;

    /* CMD58: read OCR. Bit 30 (CCS) says whether addresses are blocks (SDHC) or bytes (SDSC). */
    cs_low(); r1 = sd_cmd(58, 0, 0xFF);
    for (int i = 0; i < 4; i++) r7[i] = spi_byte(0xFF);
    deselect();
    if (r1 != 0x00) return DEV_ERR_PROTO;
    ccs = (r7[0] & 0x40u) ? 1u : 0u;
    info->ccs = ccs;

    if (!ccs) {                            /* standard capacity: force 512-byte blocks */
        cs_low(); r1 = sd_cmd(16, SD_BLOCK, 0xFF); deselect();
        if (r1 != 0x00) return DEV_ERR_PROTO;
    }

    set_prescaler(SPI_BAUDRATEPRESCALER_4);            /* 42 MHz / 4 = 10.5 MHz for normal operation */

    if (read_register(10, info->cid) != DEV_OK) return DEV_ERR_PROTO;
    if (read_register(9,  info->csd) != DEV_OK) return DEV_ERR_PROTO;
    info->sectors = sectors_from_csd(info->csd);
    return DEV_OK;
}

dev_err_t sd_read_block(uint32_t sector, uint8_t *buf)
{
    dev_err_t rc = DEV_ERR_PROTO;
    settle();
    cs_low();
    if (sd_cmd(17, ccs ? sector : sector * SD_BLOCK, 0xFF) == 0x00) {
        if (wait_token(TOKEN_TIMEOUT_MS) == 0xFE) {
            if (HAL_SPI_TransmitReceive(&hspi3, ff_block, buf, SD_BLOCK, 100) == HAL_OK) {
                spi_byte(0xFF); spi_byte(0xFF);        /* CRC, ignored */
                rc = DEV_OK;
            } else rc = DEV_ERR_BUS;
        } else rc = DEV_ERR_TIMEOUT;
    }
    deselect();
    return rc;
}

/* Starts a block write and returns as soon as the card has ACCEPTED the data. The card then
 * programs its flash internally (it holds MISO low meanwhile). Chip select stays low during that
 * time, which is fine because SPI3 only talks to this card. Poll sd_ready() before the next call. */
dev_err_t sd_write_block_nb(uint32_t sector, const uint8_t *buf)
{
    dev_err_t rc = DEV_ERR_PROTO;
    settle();
    cs_low();
    if (sd_cmd(24, ccs ? sector : sector * SD_BLOCK, 0xFF) == 0x00) {
        spi_byte(0xFF);                                /* one idle byte before the data token */
        spi_byte(0xFE);                                /* start-of-block token */
        if (HAL_SPI_Transmit(&hspi3, (uint8_t *)buf, SD_BLOCK, 100) == HAL_OK) {
            spi_byte(0xFF); spi_byte(0xFF);            /* dummy CRC */
            if ((spi_byte(0xFF) & 0x1Fu) == 0x05u) {   /* "data accepted" */
                g_busy = 1;                            /* keep CS low while the card programs */
                return DEV_OK;
            }
        } else rc = DEV_ERR_BUS;
    }
    deselect();
    return rc;
}

/* 1 if the card can take another command now. Cheap: one SPI byte while a write is pending. */
int sd_ready(void)
{
    if (!g_busy) return 1;
    if (spi_byte(0xFF) == 0xFF) {
        deselect();
        g_busy = 0;
        return 1;
    }
    return 0;
}

dev_err_t sd_write_block(uint32_t sector, const uint8_t *buf)
{
    dev_err_t rc = sd_write_block_nb(sector, buf);
    if (rc != DEV_OK) return rc;
    uint32_t t0 = HAL_GetTick();
    while (!sd_ready())
        if (HAL_GetTick() - t0 > BUSY_TIMEOUT_MS) return DEV_ERR_TIMEOUT;
    return DEV_OK;
}

dev_err_t sd_selftest(uint32_t sector)
{
    dev_err_t rc;
    for (uint32_t i = 0; i < SD_BLOCK; i++) wr_buf[i] = (uint8_t)(i * 7u + 3u + sector);
    memset(rd_buf, 0, sizeof rd_buf);

    rc = sd_write_block(sector, wr_buf);
    if (rc != DEV_OK) return rc;
    rc = sd_read_block(sector, rd_buf);
    if (rc != DEV_OK) return rc;
    return (memcmp(wr_buf, rd_buf, SD_BLOCK) == 0) ? DEV_OK : DEV_ERR_PROTO;
}
