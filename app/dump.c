#include "dump.h"
#include "log_service.h"
#include "sd_spi.h"
#include "ringbuf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QUIET_MS 3000u            /* status text stays off the link this long after the last command */

static uint8_t   rx_store[128];
static ringbuf_t rx_ring;
static volatile uint8_t inited, cmd_hint, at_line_start = 1;
static uint32_t  quiet_until;
static char      line[48];
static uint8_t   linelen, have_line;
static char      reply[96];
static uint8_t   blk[512];
static uint32_t  cur, remaining;
static uint8_t   streaming;

void dump_init(void)
{
    ringbuf_init(&rx_ring, rx_store, sizeof rx_store);
    inited = 1;
}

/* USB interrupt context. Only a line that STARTS with a command letter silences the status text,
 * so stray bytes (modem probes, a terminal's own characters) cannot. */
void dump_rx(const uint8_t *buf, uint32_t len)
{
    if (!inited) return;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        if (c == '\n' || c == '\r') {
            at_line_start = 1;
        } else {
            if (at_line_start && (c == 'i' || c == 'r' || c == 'x')) cmd_hint = 1;
            at_line_start = 0;
        }
    }
    ringbuf_write(&rx_ring, buf, len);
}

int dump_quiet(void)
{
    uint32_t now = dump_now_ms();
    if (cmd_hint) { cmd_hint = 0; quiet_until = now + QUIET_MS; }
    return (int32_t)(quiet_until - now) > 0;
}

int dump_active(void) { return streaming; }
int dump_busy(void)   { return have_line || streaming; }

static int is_command(const char *s)
{
    return s[0] == 'i' || s[0] == 'r' || (s[0] == 'x' && s[1] == '!');
}

static void send_reply(int n)
{
    if (n > 0 && n < (int)sizeof reply)
        (void)dump_tx((const uint8_t *)reply, (uint16_t)n);
}

static void handle_line(void)
{
    const log_stats_t *s = log_stats();
    if (line[0] == 'i') {
        send_reply(snprintf(reply, sizeof reply, "ATLAS used=%lu total=%lu gen=%lu session=%lu\n",
                            (unsigned long)(s->next_sector - 1u), (unsigned long)s->total_sectors,
                            (unsigned long)s->generation, (unsigned long)s->session));
    } else if (line[0] == 'r') {
        char *p;
        unsigned long a = strtoul(line + 1, &p, 10);
        unsigned long n = strtoul(p, &p, 10);
        if (a >= s->total_sectors) n = 0;
        else if (a + n > s->total_sectors) n = s->total_sectors - a;
        send_reply(snprintf(reply, sizeof reply, "ATLD %lu %lu\n", a, n));
        cur = (uint32_t)a; remaining = (uint32_t)n; streaming = (n > 0);
    } else if (line[0] == 'x' && line[1] == '!') {
        if (log_erase() == DEV_OK)
            send_reply(snprintf(reply, sizeof reply, "ATLAS erased gen=%lu\n", (unsigned long)log_stats()->generation));
        else
            send_reply(snprintf(reply, sizeof reply, "ATLAS erase-failed\n"));
    }
}

void dump_poll(void)
{
    if (!inited) return;

    if (streaming) {
        quiet_until = dump_now_ms() + QUIET_MS;
        if (!dump_tx_ready()) return;
        if (sd_read_block(cur, blk) != DEV_OK) { streaming = 0; return; }   /* host will time out */
        if (dump_tx(blk, 512) == 0) {
            cur++;
            if (--remaining == 0) streaming = 0;
        }
        return;
    }

    uint8_t c;
    while (!have_line && ringbuf_read(&rx_ring, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (linelen) { line[linelen] = 0; have_line = 1; }
        } else if (linelen < sizeof line - 1) {
            line[linelen++] = (char)c;
        }
    }
    if (!have_line) return;

    if (!is_command(line)) {                      /* not ours (e.g. a modem probe): ignore it completely */
        have_line = 0;
        linelen = 0;
        return;
    }

    quiet_until = dump_now_ms() + QUIET_MS;
    log_pause();                                  /* a real command: stop logging and flush what is queued */
    if (!log_idle()) return;                      /* wait for the queue to drain, retry next pass */
    if (!dump_tx_ready()) return;
    handle_line();
    have_line = 0;
    linelen = 0;
}
