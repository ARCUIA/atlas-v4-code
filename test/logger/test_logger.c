#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "spi.h"
#include "sd_spi.h"
#include "sim_card.h"
#include "logblk.h"
#include "log_service.h"

extern uint64_t g_us;

typedef struct { uint32_t blocks, bad, imu, baro, status, session_recs, gaps, seq_ok, gps, gpscfg; } scan_t;

/* Walk the stored blocks like the PC parser does. */
static scan_t scan(uint32_t first, uint32_t last_excl, uint32_t gen, uint32_t session)
{
    scan_t r; memset(&r, 0, sizeof r);
    int have_prev = 0; uint32_t prev_seq = 0;
    for (uint32_t s = first; s < last_excl; s++) {
        const uint8_t *b = sim_sector(s);
        logblk_info_t in;
        if (logblk_check(b, &in) != 0) { r.bad++; continue; }
        if (in.generation != gen || in.session != session) continue;
        r.blocks++;
        if (have_prev) { if (in.seq > prev_seq + 1) r.gaps += in.seq - prev_seq - 1; assert(in.seq > prev_seq); }
        prev_seq = in.seq; have_prev = 1;
        uint16_t pos = LOGBLK_HDR, end = (uint16_t)(LOGBLK_HDR + in.used), n = 0;
        while (pos + 2 <= end) {
            uint8_t type = b[pos], len = b[pos + 1];
            assert(pos + 2 + len <= end);
            if (type == REC_IMU) r.imu++; else if (type == REC_BARO) r.baro++;
            else if (type == REC_STATUS) r.status++; else if (type == REC_SESSION) r.session_recs++;
            else if (type == REC_GPS) { r.gps++; assert(len == 96); } else if (type == REC_GPSCFG) { r.gpscfg++; assert(len == 17); }
            pos = (uint16_t)(pos + 2 + len); n++;
        }
        assert(n == in.nrec);
    }
    return r;
}

static void drain(void)
{
    log_pause();
    for (int i = 0; i < 2000000 && !log_idle(); i++) { log_service_poll(); g_us += 50; }
    assert(log_idle());
}

/* Feed IMU/baro/status records the way main() does, polling the writer once per sample. */
static void feed(int n_imu, int baro_every, int status_every, int poll_every_record)
{
    imu_raw_t s; memset(&s, 0, sizeof s);
    baro_sample_t b; memset(&b, 0, sizeof b);
    for (int i = 0; i < n_imu; i++) {
        s.xl_raw[0] = (int16_t)i; s.hg_raw[2] = (int16_t)(-i); s.gy_raw[1] = (int16_t)(i * 3);
        g_us += 2083;                                          /* 480 Hz */
        log_imu(&s, 7);
        if (baro_every && i % baro_every == 0) { b.pressure_raw = 0x5A000000 + i * 256; b.temp_raw = 2300; log_baro(&b); }
        if (status_every && i % status_every == 0) log_status();
        if (poll_every_record) log_service_poll();
    }
}

int main(void)
{
    sd_info_t info;

    /* ---------- block format ---------- */
    {
        uint8_t blk[LOGBLK_SIZE]; uint8_t d[23] = {1,2,3};
        logblk_begin(blk, 5, 6, 7);
        int n = 0; while (logblk_append(blk, REC_IMU, d, sizeof d, 1000 + n) == 0) n++;
        assert(n == (int)(LOGBLK_PAYLOAD / (2 + sizeof d)));           /* 19 records fit */
        logblk_finalize(blk);
        logblk_info_t in; assert(logblk_check(blk, &in) == 0);
        assert(in.generation == 5 && in.session == 6 && in.seq == 7 && in.nrec == n && in.t0_us == 1000);
        blk[100] ^= 0x01; assert(logblk_check(blk, NULL) != 0);        /* any flipped bit is caught */
        uint8_t v[9] = {'1','2','3','4','5','6','7','8','9'};
        assert(crc16_ccitt(0xFFFF, v, 9) == 0x29B1);                   /* standard CRC-16/CCITT-FALSE check value */
        puts("log block format + CRC: OK");
    }

    /* ---------- fresh card: everything logged ends up on the card, intact ---------- */
    sim_reset_card(1, 1, 0);                                           /* 1024 sectors */
    assert(sd_init(&info) == DEV_OK && info.sectors == 1024);
    g_us = 1000000;
    assert(log_service_init(&info) == DEV_OK);
    assert(log_stats()->generation == 1 && log_stats()->session == 1 && log_stats()->next_sector == 1);
    log_gps_cfg(38400, 115200, 56, 0, 8, 1);
    feed(1000, 20, 480, 1);
    {   /* 50 GPS solutions, 56 ms apart, with a known position */
        uint8_t pvt[92]; memset(pvt, 0, sizeof pvt);
        pvt[20] = 3; pvt[21] = 1; pvt[23] = 9; pvt[4] = 0xEA; pvt[5] = 0x07; pvt[6] = 11; pvt[7] = 5;
        int32_t lat = 416600000, lon = -916500000; memcpy(pvt + 28, &lat, 4); memcpy(pvt + 24, &lon, 4);
        for (uint32_t i = 0; i < 50; i++) { uint32_t itow = i * 56; memcpy(pvt, &itow, 4); g_us += 56000; log_gps(pvt); log_service_poll(); }
    }
    drain();
    uint32_t end1 = log_stats()->next_sector;
    scan_t r = scan(1, end1, 1, 1);
    printf("fresh card: %u blocks, imu=%u baro=%u status=%u session=%u bad=%u gaps=%u dropped=%u\n",
           r.blocks, r.imu, r.baro, r.status, r.session_recs, r.bad, r.gaps, (unsigned)log_stats()->blocks_dropped);
    assert(r.imu == 1000 && r.baro == 50 && r.session_recs == 1 && r.status >= 2 && r.gps == 50 && r.gpscfg == 1);
    assert(r.bad == 0 && r.gaps == 0 && log_stats()->blocks_dropped == 0 && r.blocks == end1 - 1);

    /* export for the Python parser test */
    FILE *f = fopen("dump.bin", "wb");
    for (uint32_t s = 1; s < end1; s++) fwrite(sim_sector(s), 1, 512, f);
    fclose(f);
    f = fopen("expected.txt", "w"); fprintf(f, "imu %u\nbaro %u\nblocks %u\ngps %u\n", r.imu, r.baro, r.blocks, r.gps); fclose(f);

    /* ---------- reboot: the log continues after the old data, as a new session ---------- */
    assert(log_service_init(&info) == DEV_OK);
    assert(log_stats()->next_sector == end1 && log_stats()->session == 2 && log_stats()->generation == 1);
    feed(200, 20, 0, 1);
    drain();
    uint32_t end2 = log_stats()->next_sector;
    scan_t r2 = scan(end1, end2, 1, 2);
    assert(end2 > end1 && r2.imu == 200 && r2.bad == 0 && r2.gaps == 0 && r2.session_recs == 1);
    assert(scan(1, end1, 1, 1).imu == 1000);                           /* session 1 untouched */
    puts("reboot / session handling: OK");

    /* ---------- card stalls: the loop never waits, blocks queue, overflow is dropped and counted ---------- */
    assert(log_service_init(&info) == DEV_OK);                         /* session 3 */
    uint32_t start3 = log_stats()->next_sector;
    sim_set_busy_len(4000);                                            /* each write keeps the card busy a long time */
    feed(4000, 0, 0, 1);
    const log_stats_t *st = log_stats();
    printf("stall: queue=%u max=%u dropped=%u written=%u\n", (unsigned)st->queue, (unsigned)st->max_queue,
           (unsigned)st->blocks_dropped, (unsigned)st->blocks_written);
    assert(st->blocks_dropped > 0 && st->queue <= 47 && st->max_queue <= 47);
    sim_set_busy_len(3);                                               /* card recovers */
    uint32_t written_before = st->blocks_written;
    feed(300, 0, 0, 1);
    drain();
    assert(log_stats()->blocks_written > written_before);
    scan_t r3 = scan(start3, log_stats()->next_sector, 1, 3);
    assert(r3.bad == 0 && r3.gaps <= log_stats()->blocks_dropped && r3.blocks == log_stats()->next_sector - start3);
    puts("stall / overflow handling: OK");

    /* ---------- erase: a new generation hides all old blocks ---------- */
    assert(log_service_init(&info) == DEV_OK);
    log_pause();
    drain();
    assert(log_erase() == DEV_OK && log_stats()->generation == 2 && log_stats()->next_sector == 1);
    assert(log_service_init(&info) == DEV_OK);                         /* reboot after erase */
    assert(log_stats()->generation == 2 && log_stats()->next_sector == 1 && log_stats()->session == 1);
    feed(100, 0, 0, 1);
    drain();
    uint32_t end4 = log_stats()->next_sector;
    assert(end4 > 1 && scan(1, end4, 2, 1).imu == 100);
    assert(log_service_init(&info) == DEV_OK && log_stats()->next_sector == end4);   /* old-generation blocks beyond are ignored */
    puts("erase / generations: OK");

    /* ---------- card full: stops cleanly ---------- */
    sim_reset_card(1, 1, 0);
    assert(sd_init(&info) == DEV_OK);
    sd_info_t tiny = info; tiny.sectors = 8;                           /* pretend the card has only 8 sectors */
    assert(log_service_init(&tiny) == DEV_OK);
    feed(1000, 0, 0, 1);
    drain();
    assert(log_stats()->card_full && log_stats()->next_sector == 8 && log_stats()->blocks_dropped > 0);
    puts("card full: OK");

    puts("ALL LOGGER TESTS PASSED");
    return 0;
}
