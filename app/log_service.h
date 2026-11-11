#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H
/*
 * Flight log service: packs sensor records into 512-byte blocks, queues them in RAM, and writes
 * them to the SD card without ever waiting on it.
 *
 * Call log_imu()/log_baro()/log_status() whenever there is data, and log_service_poll() on every
 * pass of the main loop. If the card stalls, blocks wait in the RAM queue (about 2 s of data);
 * beyond that the newest blocks are dropped and counted, never the loop blocked.
 */
#include <stdint.h>
#include "devices.h"
#include "sd_spi.h"
#include "imu.h"
#include "baro.h"

typedef struct {
    uint32_t generation, session, next_sector, total_sectors;
    uint32_t blocks_written, blocks_dropped, sd_errors;
    uint32_t queue, max_queue;      /* blocks waiting in RAM now / at most so far */
    uint8_t  paused, card_full;
} log_stats_t;

dev_err_t log_service_init(const sd_info_t *sd);   /* finds the end of the existing log, starts a new session */
void log_service_poll(void);

void log_imu (const imu_raw_t *s, uint8_t flags);
void log_baro(const baro_sample_t *b);
void log_status(void);
void log_gps(const uint8_t *nav_pvt_payload92);      /* one UBX-NAV-PVT, stored exactly as received */
void log_gps_cfg(uint32_t baud_found, uint32_t baud_final, uint16_t meas_ms, int8_t code, uint8_t dynmodel, uint8_t gps_only);

void log_pause(void);          /* stop accepting records; the partial block is flushed */
int  log_idle(void);           /* paused, queue empty, card idle: safe to read the log */
dev_err_t log_erase(void);     /* start a new generation: old blocks are ignored. Only when idle. */
const log_stats_t *log_stats(void);

#endif
