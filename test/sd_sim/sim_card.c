/* Host-side simulation of an SD card in SPI mode, to exercise devices/sd/sd_spi.c */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "spi.h"
#include "sd_spi.h"
#include "sim_card.h"

GPIO_TypeDef sim_gpioa;
SPI_HandleTypeDef hspi3;

static struct {
    int present, sdhc, v2, cs_low;
    uint8_t cmdbuf[6]; int cmdlen;
    uint8_t q[1200]; int qh, qt;
    int busy, idle, acmd, acmd41_calls;
    enum { W_NONE, W_TOKEN, W_DATA } w; int wcount; uint32_t waddr; uint8_t wbuf[514];
    uint8_t csd[16], cid[16];
    uint8_t storage[1100][512];
    int cmd0_seen_with_cs_low;
    int busy_len;
} S;
static uint32_t tick;

static void push(uint8_t b) { S.q[S.qt++] = b; }
static void exec_cmd(void)
{
    uint8_t cmd = S.cmdbuf[0] & 0x3F;
    uint32_t arg = ((uint32_t)S.cmdbuf[1] << 24) | ((uint32_t)S.cmdbuf[2] << 16) | ((uint32_t)S.cmdbuf[3] << 8) | S.cmdbuf[4];
    uint8_t crc = S.cmdbuf[5];
    S.qh = S.qt = 0;
    push(0xFF);                                    /* NCR gap before R1 */
    int was_acmd = S.acmd; S.acmd = 0;
    switch (cmd) {
    case 0:  assert(crc == 0x95); S.idle = 1; S.cmd0_seen_with_cs_low = 1; push(0x01); break;
    case 8:  assert(crc == 0x87); assert(arg == 0x1AA);
             if (S.v2) { push(0x01); push(0); push(0); push(0x01); push(0xAA); } else push(0x05);
             break;
    case 55: S.acmd = 1; push(S.idle ? 0x01 : 0x00); break;
    case 41: assert(was_acmd);
             if (++S.acmd41_calls < 3) push(0x01); else { S.idle = 0; push(0x00); }
             break;
    case 58: push(0x00); push(0x80 | (S.sdhc ? 0x40 : 0)); push(0xFF); push(0x80); push(0x00); break;
    case 16: push(arg == 512 ? 0x00 : 0x40); break;
    case 9:  case 10:
             push(0x00); push(0xFF); push(0xFE);
             for (int i = 0; i < 16; i++) push(cmd == 9 ? S.csd[i] : S.cid[i]);
             push(0xAA); push(0xAA); break;
    case 17: {
             uint32_t blk = S.sdhc ? arg : arg / 512;
             if (!S.sdhc) assert(arg % 512 == 0);
             push(0x00); push(0xFF); push(0xFF); push(0xFE);
             for (int i = 0; i < 512; i++) push(S.storage[blk][i]);
             push(0xAA); push(0xAA); break; }
    case 24: S.waddr = S.sdhc ? arg : arg / 512; if (!S.sdhc) assert(arg % 512 == 0);
             push(0x00); S.w = W_TOKEN; break;
    default: push(0x04);
    }
}

static uint8_t sim_xfer(uint8_t in)
{
    uint8_t out = 0xFF;
    if (!S.present || !S.cs_low) return 0xFF;
    if (S.qh < S.qt) out = S.q[S.qh++];                  /* queued response bytes go out first ... */
    else if (S.busy > 0) { S.busy--; out = 0x00; }       /* ... then the card holds MISO low while busy */

    if (S.w == W_TOKEN) { if (in == 0xFE) { S.w = W_DATA; S.wcount = 0; } }
    else if (S.w == W_DATA) {
        S.wbuf[S.wcount++] = in;
        if (S.wcount == 514) {                      /* 512 data + 2 CRC */
            memcpy(S.storage[S.waddr], S.wbuf, 512);
            S.qh = S.qt = 0; push(0x05); S.busy = S.busy_len; S.w = W_NONE;   /* data accepted, then busy */
        }
    }
    else if (S.cmdlen > 0 || (in & 0xC0) == 0x40) {
        S.cmdbuf[S.cmdlen++] = in;
        if (S.cmdlen == 6) { S.cmdlen = 0; exec_cmd(); }
    }
    return out;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState st) { (void)p; if (pin == CS_FLASH_Pin) S.cs_low = (st == GPIO_PIN_RESET); }
uint32_t HAL_GetTick(void) { return tick++; }
void HAL_Delay(uint32_t ms) { tick += ms; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t t) { (void)h; (void)t; for (uint16_t i = 0; i < n; i++) rx[i] = sim_xfer(tx[i]); return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *tx, uint16_t n, uint32_t t) { (void)h; (void)t; for (uint16_t i = 0; i < n; i++) (void)sim_xfer(tx[i]); return HAL_OK; }


void sim_reset_card(int sdhc, int v2, uint32_t c_size)
{
    memset(&S, 0, sizeof S);
    S.present = 1; S.sdhc = sdhc; S.v2 = v2; S.busy_len = 3;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    sim_gpioa.MODER = 1u << 30;                     /* PA15 = general purpose output */
    if (sdhc) { uint32_t c = c_size; S.csd[0] = 0x40; S.csd[7] = (c >> 16) & 0x3F; S.csd[8] = (c >> 8) & 0xFF; S.csd[9] = c & 0xFF; }
    else      { uint32_t c = c_size, m = 7; S.csd[0] = 0x00; S.csd[5] = 10;   /* 12-bit C_SIZE, 1024-byte READ_BL_LEN like real ~2 GB cards */
                S.csd[6] = (c >> 10) & 3; S.csd[7] = (c >> 2) & 0xFF;
                S.csd[8] = (c & 3) << 6; S.csd[9] = (m >> 1) & 3; S.csd[10] = (m & 1) << 7; }
    S.cid[0] = 0x9A;
    tick = 0;
}
void sim_set_busy_len(uint32_t n) { S.busy_len = (int)n; }
void sim_set_present(int p) { S.present = p; }
int  sim_cmd0_seen(void) { return S.cmd0_seen_with_cs_low; }
uint8_t *sim_sector(uint32_t sec) { assert(sec < 1100); return S.storage[sec]; }
