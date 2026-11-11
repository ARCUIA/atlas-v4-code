#include "logblk.h"
#include <string.h>

#define OFF_GEN 4
#define OFF_SESSION 8
#define OFF_SEQ 12
#define OFF_T0 16
#define OFF_NREC 20
#define OFF_USED 22
#define OFF_CRC 24

uint16_t crc16_ccitt(uint16_t crc, const uint8_t *p, size_t n)
{
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

void logblk_begin(uint8_t *blk, uint32_t generation, uint32_t session, uint32_t seq)
{
    memset(blk, 0, LOGBLK_SIZE);
    blk[0] = 'A'; blk[1] = 'T'; blk[2] = 'L'; blk[3] = 'B';
    lb_put_u32(blk + OFF_GEN, generation);
    lb_put_u32(blk + OFF_SESSION, session);
    lb_put_u32(blk + OFF_SEQ, seq);
}

int logblk_append(uint8_t *blk, uint8_t type, const uint8_t *data, uint8_t len, uint32_t t_us)
{
    uint16_t used = lb_get_u16(blk + OFF_USED);
    uint16_t nrec = lb_get_u16(blk + OFF_NREC);
    if ((uint32_t)used + 2u + len > LOGBLK_PAYLOAD)
        return -1;
    if (nrec == 0)
        lb_put_u32(blk + OFF_T0, t_us);
    uint8_t *dst = blk + LOGBLK_HDR + used;
    dst[0] = type;
    dst[1] = len;
    memcpy(dst + 2, data, len);
    lb_put_u16(blk + OFF_USED, (uint16_t)(used + 2u + len));
    lb_put_u16(blk + OFF_NREC, (uint16_t)(nrec + 1u));
    return 0;
}

static uint16_t block_crc(const uint8_t *blk)
{
    uint16_t used = lb_get_u16(blk + OFF_USED);
    uint16_t crc = crc16_ccitt(0xFFFFu, blk, OFF_CRC);
    return crc16_ccitt(crc, blk + LOGBLK_HDR, used);
}

void logblk_finalize(uint8_t *blk)
{
    lb_put_u16(blk + OFF_CRC, block_crc(blk));
}

int logblk_check(const uint8_t *blk, logblk_info_t *info)
{
    if (blk[0] != 'A' || blk[1] != 'T' || blk[2] != 'L' || blk[3] != 'B') return -1;
    uint16_t used = lb_get_u16(blk + OFF_USED);
    if (used > LOGBLK_PAYLOAD) return -2;
    if (lb_get_u16(blk + OFF_CRC) != block_crc(blk)) return -3;
    if (info) {
        info->generation = lb_get_u32(blk + OFF_GEN);
        info->session    = lb_get_u32(blk + OFF_SESSION);
        info->seq        = lb_get_u32(blk + OFF_SEQ);
        info->t0_us      = lb_get_u32(blk + OFF_T0);
        info->nrec       = lb_get_u16(blk + OFF_NREC);
        info->used       = used;
    }
    return 0;
}
