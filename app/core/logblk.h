#ifndef LOGBLK_H
#define LOGBLK_H
/*
 * Log block format. HAL-free, shared in spirit with tools/atlas_parse.py.
 *
 * Every 512-byte SD sector holds one self-contained block, so a lost or corrupt block never
 * affects its neighbours:
 *
 *   offset size  field
 *    0     4     magic "ATLB"
 *    4     4     generation   (bumped by an erase; blocks of an older generation are ignored)
 *    8     4     session      (one per power-up)
 *   12     4     seq          (block number within the session, starts at 0)
 *   16     4     t0_us        (timestamp of the first record, low 32 bits)
 *   20     2     nrec         (records in this block)
 *   22     2     used         (payload bytes in use, max 480)
 *   24     2     crc16        (CRC-16/CCITT over bytes 0..23 and payload[0..used))
 *   26     6     reserved (zero)
 *   32   480     payload: records [type u8][len u8][data len bytes]
 *
 * All multi-byte fields are little-endian.
 */
#include <stdint.h>
#include <stddef.h>

#define LOGBLK_SIZE     512u
#define LOGBLK_HDR      32u
#define LOGBLK_PAYLOAD  (LOGBLK_SIZE - LOGBLK_HDR)

enum {
    REC_SESSION = 1,   /* t_us u32, fmt u8, rsv u8, then 8 x u16: cpu_mhz, xl_fs_g, hg_fs_g, gy_fs_dps,
                          xl_odr_hz, hg_odr_hz, gy_odr_hz, baro_odr_x100                            (22 B) */
    REC_IMU     = 2,   /* t_us u32, flags u8 (bit0 xl new, bit1 hg new, bit2 gy new), xl[3] i16,
                          hg[3] i16, gy[3] i16                                                      (23 B) */
    REC_BARO    = 3,   /* t_us u32, pressure_raw i32, temp_raw i16                                  (10 B) */
    REC_STATUS  = 4,   /* t_us u32, blocks_written u32, blocks_dropped u32, queue u16, sd_errors u16 (16 B) */
    REC_GPS     = 5,   /* t_us u32, then the 92-byte UBX-NAV-PVT payload exactly as the receiver sent it   (96 B) */
    REC_GPSCFG  = 6    /* t_us u32, baud_found u32, baud_final u32, meas_period_ms u16, code i8, dynmodel u8,
                          gps_only u8                                                                (17 B) */
};

typedef struct {
    uint32_t generation, session, seq, t0_us;
    uint16_t nrec, used;
} logblk_info_t;

static inline void lb_put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void lb_put_u32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static inline uint16_t lb_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t lb_get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

uint16_t crc16_ccitt(uint16_t crc, const uint8_t *p, size_t n);

void logblk_begin(uint8_t *blk, uint32_t generation, uint32_t session, uint32_t seq);
/* Returns 0 on success, -1 if the record does not fit in this block. */
int  logblk_append(uint8_t *blk, uint8_t type, const uint8_t *data, uint8_t len, uint32_t t_us);
void logblk_finalize(uint8_t *blk);
/* Returns 0 if the block is intact (magic and CRC), negative otherwise. info may be NULL. */
int  logblk_check(const uint8_t *blk, logblk_info_t *info);

#endif
