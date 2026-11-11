/* Runs the real dump protocol (app/dump.c) over stdin/stdout, so tools/atlas_dump.py can be tested end to end. */
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "sd_spi.h"
#include "sim_card.h"
#include "log_service.h"
#include "dump.h"

extern uint64_t g_us;
static unsigned long tx_bytes;

uint32_t dump_now_ms(void) { return (uint32_t)(g_us / 1000u); }
int dump_tx_ready(void) { return 1; }
int dump_tx(const uint8_t *p, uint16_t n) { fwrite(p, 1, n, stdout); fflush(stdout); tx_bytes += n; return 0; }

int main(void)
{
    sd_info_t info; char cmd[128];
    sim_reset_card(1, 1, 0);
    if (sd_init(&info) != DEV_OK || log_service_init(&info) != DEV_OK) return 2;
    dump_init();
    imu_raw_t s; memset(&s, 0, sizeof s);
    for (int i = 0; i < 400; i++) { s.xl_raw[0] = (int16_t)i; g_us += 2083; log_imu(&s, 7); log_service_poll(); }

    while (fgets(cmd, sizeof cmd, stdin)) {
        dump_rx((const uint8_t *)cmd, (uint32_t)strlen(cmd));
        long guard = 0;
        for (int idle = 0; idle < 5 && ++guard < 5000000; ) {   /* run the main loop until the command is fully handled */
            unsigned long before = tx_bytes;
            log_service_poll(); dump_poll(); g_us += 100;
            if (tx_bytes == before && !dump_busy()) idle++; else idle = 0;
        }
        if (guard >= 5000000) return 3;
    }
    return 0;
}
