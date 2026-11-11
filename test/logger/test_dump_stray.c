/* Stray bytes on the USB link must not silence the status stream or pause logging. */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "spi.h"
#include "sd_spi.h"
#include "sim_card.h"
#include "log_service.h"
#include "dump.h"

extern uint64_t g_us;
static char txbuf[4096]; static size_t txn;
uint32_t dump_now_ms(void) { return (uint32_t)(g_us / 1000u); }
int dump_tx_ready(void) { return 1; }
int dump_tx(const uint8_t *p, uint16_t n) { if (txn + n < sizeof txbuf) memcpy(txbuf + txn, p, n); txn += n; return 0; }

static void send(const char *s) { dump_rx((const uint8_t *)s, (uint32_t)strlen(s)); }
static void pump_until_idle(void)
{
    for (int i = 0; i < 3000000; i++) {
        log_service_poll(); dump_poll(); g_us += 100;
        if (!dump_busy() && i > 10) return;
    }
    assert(0 && "never went idle");
}
static void pump(int n) { for (int i = 0; i < n; i++) { log_service_poll(); dump_poll(); g_us += 100; } }

int main(void)
{
    sd_info_t info; imu_raw_t s; memset(&s, 0, sizeof s);
    sim_reset_card(1, 1, 0);
    assert(sd_init(&info) == DEV_OK);
    g_us = 5000000; assert(log_service_init(&info) == DEV_OK);
    dump_init();
    for (int i = 0; i < 100; i++) { g_us += 2083; log_imu(&s, 7); log_service_poll(); }

    /* 1. things Linux or a terminal might send: all ignored, no reply, no pause, no silence */
    send("AT\r\n"); send("ATE0\r\n"); send("\r\n"); send("hello\n");
    pump(5000);
    assert(!dump_quiet() && !log_stats()->paused && txn == 0);
    puts("stray bytes / modem probes: ignored (no reply, logging not paused, stream not silenced): OK");

    /* 2. a real command silences the stream, pauses logging, and answers */
    send("i\n");
    assert(dump_quiet());                                   /* silenced immediately, before the main loop even handles it */
    pump_until_idle();
    assert(txn > 0 && strncmp(txbuf, "ATLAS used=", 11) == 0 && log_stats()->paused);
    assert(dump_quiet());                                   /* still inside the quiet window */
    puts("real command: stream silenced, logging paused, reply sent: OK");

    /* 3. after the quiet window the status stream may resume */
    g_us += 3500000;
    assert(!dump_quiet());
    puts("quiet window expires after about 3 s: OK");

    /* 4. sectors stream cleanly, and the stream stays quiet while it runs */
    txn = 0; send("r 1 2\n");
    assert(dump_quiet());
    pump_until_idle();
    assert(strncmp(txbuf, "ATLD 1 2\n", 9) == 0 && txn == 9 + 1024);
    assert(dump_quiet());
    puts("sector stream: header + 1024 bytes, no stray text: OK");

    puts("ALL STRAY-BYTE TESTS PASSED");
    return 0;
}
