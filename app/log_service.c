#include "log_service.h"
#include "logblk.h"
#include "app_time.h"
#include <string.h>

#define QUEUE_BLOCKS   48u          /* RAM queue, about 24 KB */
#define FLUSH_MS       1000u        /* write a partly filled block after this long */

static uint8_t  q[QUEUE_BLOCKS][LOGBLK_SIZE];
static uint8_t  scratch[LOGBLK_SIZE];      /* for card header and end-of-log search (kept off the stack) */
static uint32_t wr_cnt, rd_cnt;            /* finished blocks; the block being built lives in slot wr_cnt % N */
static uint8_t  open_blk, ready, paused, card_full;
static uint32_t opened_ms;
static uint32_t gen, session, seq, next_sector, total_sectors;
static log_stats_t st;

/* ---------------- card header (sector 0): "ATLC", generation, CRC ---------------- */
static int read_card_header(uint32_t *g)
{
    if (sd_read_block(0, scratch) != DEV_OK) return -1;
    if (memcmp(scratch, "ATLC", 4) != 0) return -1;
    if (lb_get_u16(scratch + 8) != crc16_ccitt(0xFFFFu, scratch, 8)) return -1;
    *g = lb_get_u32(scratch + 4);
    return 0;
}

static dev_err_t write_card_header(uint32_t g)
{
    memset(scratch, 0, sizeof scratch);
    memcpy(scratch, "ATLC", 4);
    lb_put_u32(scratch + 4, g);
    lb_put_u16(scratch + 8, crc16_ccitt(0xFFFFu, scratch, 8));
    return sd_write_block(0, scratch);
}

static int block_valid(uint32_t sector, logblk_info_t *info)
{
    if (sd_read_block(sector, scratch) != DEV_OK) return 0;
    if (logblk_check(scratch, info) != 0) return 0;
    return info->generation == gen;
}

/* ---------------- building and queueing blocks ---------------- */
static void close_block(void)
{
    if (!open_blk) return;
    logblk_finalize(q[wr_cnt % QUEUE_BLOCKS]);
    open_blk = 0;
    seq++;
    if ((wr_cnt - rd_cnt) < (QUEUE_BLOCKS - 1u))
        wr_cnt++;                         /* queued for writing */
    else
        st.blocks_dropped++;              /* queue full: drop this block, its slot is reused */
}

static void put_record(uint8_t type, const uint8_t *data, uint8_t len, uint32_t t_us)
{
    if (!ready || paused) return;
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t *b = q[wr_cnt % QUEUE_BLOCKS];
        if (!open_blk) {
            logblk_begin(b, gen, session, seq);
            open_blk = 1;
            opened_ms = time_ms();
        }
        if (logblk_append(b, type, data, len, t_us) == 0) return;
        close_block();                    /* record did not fit: close this block, retry in a fresh one */
    }
}

static void session_record(void)
{
    uint8_t d[22];
    imu_cfg_t c;
    uint32_t t = (uint32_t)time_us();
    imu_get_cfg(&c);
    lb_put_u32(d, t);
    d[4] = 1;                             /* format version */
    d[5] = 0;
    lb_put_u16(d + 6,  time_cpu_mhz());
    lb_put_u16(d + 8,  c.xl_fs_g);
    lb_put_u16(d + 10, c.hg_fs_g);
    lb_put_u16(d + 12, c.gy_fs_dps);
    lb_put_u16(d + 14, c.xl_odr_hz);
    lb_put_u16(d + 16, c.hg_odr_hz);
    lb_put_u16(d + 18, c.gy_odr_hz);
    lb_put_u16(d + 20, baro_odr_x100());
    put_record(REC_SESSION, d, sizeof d, t);
}

/* ---------------- public API ---------------- */
dev_err_t log_service_init(const sd_info_t *sd)
{
    uint32_t g;
    memset(&st, 0, sizeof st);
    wr_cnt = rd_cnt = 0; open_blk = 0; seq = 0; paused = 0; card_full = 0; ready = 0;
    total_sectors = sd->sectors;
    if (total_sectors < 4u) return DEV_ERR_CFG;

    if (read_card_header(&g) != 0) {                  /* first use of this card */
        g = 1;
        if (write_card_header(g) != DEV_OK) return DEV_ERR_BUS;
    }
    gen = g;

    /* Valid blocks of this generation form a prefix starting at sector 1, so a binary search
     * finds the first free sector in about 22 reads. */
    uint32_t lo = 1, hi = total_sectors;
    while (lo < hi) {
        logblk_info_t info;
        uint32_t mid = lo + (hi - lo) / 2u;
        if (block_valid(mid, &info)) lo = mid + 1u; else hi = mid;
    }
    next_sector = lo;

    session = 1;
    if (lo > 1u) {
        logblk_info_t info;
        if (block_valid(lo - 1u, &info)) session = info.session + 1u;
    }
    card_full = (next_sector >= total_sectors);
    ready = 1;
    session_record();
    return DEV_OK;
}

void log_service_poll(void)
{
    if (!ready) return;
    uint32_t now = time_ms();

    if (open_blk && (paused || (now - opened_ms) >= FLUSH_MS))
        close_block();                                /* flush a partial block */

    uint32_t pending = wr_cnt - rd_cnt;
    if (pending > st.max_queue) st.max_queue = pending;
    if (!pending) return;

    if (next_sector >= total_sectors) {               /* card full: nothing more can be stored */
        card_full = 1;
        st.blocks_dropped += pending;
        rd_cnt = wr_cnt;
        return;
    }
    if (!sd_ready()) return;                          /* card busy programming: try again next pass */
    if (sd_write_block_nb(next_sector, q[rd_cnt % QUEUE_BLOCKS]) == DEV_OK) {
        next_sector++; rd_cnt++; st.blocks_written++;
    } else {
        st.sd_errors++;
    }
}

void log_imu(const imu_raw_t *s, uint8_t flags)
{
    uint8_t d[23];
    uint32_t t = (uint32_t)time_us();
    lb_put_u32(d, t);
    d[4] = flags;
    for (int i = 0; i < 3; i++) {
        lb_put_u16(d + 5  + 2 * i, (uint16_t)s->xl_raw[i]);
        lb_put_u16(d + 11 + 2 * i, (uint16_t)s->hg_raw[i]);
        lb_put_u16(d + 17 + 2 * i, (uint16_t)s->gy_raw[i]);
    }
    put_record(REC_IMU, d, sizeof d, t);
}

void log_baro(const baro_sample_t *b)
{
    uint8_t d[10];
    uint32_t t = (uint32_t)time_us();
    lb_put_u32(d, t);
    lb_put_u32(d + 4, (uint32_t)b->pressure_raw);
    lb_put_u16(d + 8, (uint16_t)b->temp_raw);
    put_record(REC_BARO, d, sizeof d, t);
}

void log_gps(const uint8_t *pvt)
{
    uint8_t d[96];
    uint32_t t = (uint32_t)time_us();
    lb_put_u32(d, t);
    memcpy(d + 4, pvt, 92);
    put_record(REC_GPS, d, sizeof d, t);
}

void log_gps_cfg(uint32_t baud_found, uint32_t baud_final, uint16_t meas_ms, int8_t code, uint8_t dynmodel, uint8_t gps_only)
{
    uint8_t d[17];
    uint32_t t = (uint32_t)time_us();
    lb_put_u32(d, t);
    lb_put_u32(d + 4, baud_found);
    lb_put_u32(d + 8, baud_final);
    lb_put_u16(d + 12, meas_ms);
    d[14] = (uint8_t)code;
    d[15] = dynmodel;
    d[16] = gps_only;
    put_record(REC_GPSCFG, d, sizeof d, t);
}

void log_status(void)
{
    uint8_t d[16];
    uint32_t t = (uint32_t)time_us();
    lb_put_u32(d, t);
    lb_put_u32(d + 4, st.blocks_written);
    lb_put_u32(d + 8, st.blocks_dropped);
    lb_put_u16(d + 12, (uint16_t)(wr_cnt - rd_cnt));
    lb_put_u16(d + 14, (uint16_t)st.sd_errors);
    put_record(REC_STATUS, d, sizeof d, t);
}

void log_pause(void) { paused = 1; }

int log_idle(void)
{
    if (!ready) return 1;
    return paused && !open_blk && (wr_cnt == rd_cnt) && sd_ready();
}

dev_err_t log_erase(void)
{
    if (!ready || !log_idle()) return DEV_ERR_CFG;
    gen++;
    if (write_card_header(gen) != DEV_OK) return DEV_ERR_BUS;
    next_sector = 1; session = 1; seq = 0; card_full = 0;
    return DEV_OK;
}

const log_stats_t *log_stats(void)
{
    st.generation = gen; st.session = session; st.next_sector = next_sector;
    st.total_sectors = total_sectors; st.queue = wr_cnt - rd_cnt;
    st.paused = paused; st.card_full = card_full;
    return &st;
}
