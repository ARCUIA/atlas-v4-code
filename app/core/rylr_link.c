#include "rylr_link.h"
#include "lora_airtime.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PROBE_TIMEOUT_MS   400u
#define QUERY_TIMEOUT_MS   500u
#define SET_TIMEOUT_MS    1500u      /* a flash write can take a while */
#define MAX_TRIES            3u
#define INIT_LIMIT_MS     8000u
#define TX_MARGIN_MS      1500u

uint32_t rylr_bw_hz(uint8_t code) { return code == 7 ? 125000u : code == 8 ? 250000u : 500000u; }

static void build_items(rylr_link_t *l)
{
    const rylr_settings_t *c = &l->cfg;
    snprintf(l->query[0], sizeof l->query[0], "AT+ADDRESS?");   snprintf(l->key[0], sizeof l->key[0], "ADDRESS");
    if (c->own_addr == RYLR_KEEP) snprintf(l->want[0], sizeof l->want[0], "*");      /* any address is fine */
    else                          snprintf(l->want[0], sizeof l->want[0], "%u", (unsigned)c->own_addr);
    snprintf(l->set[0], sizeof l->set[0], "AT+ADDRESS=%u", (unsigned)c->own_addr);

    snprintf(l->query[1], sizeof l->query[1], "AT+NETWORKID?"); snprintf(l->key[1], sizeof l->key[1], "NETWORK");
    snprintf(l->want[1], sizeof l->want[1], "%u", (unsigned)c->network_id);
    snprintf(l->set[1], sizeof l->set[1], "AT+NETWORKID=%u", (unsigned)c->network_id);

    snprintf(l->query[2], sizeof l->query[2], "AT+BAND?");      snprintf(l->key[2], sizeof l->key[2], "BAND");
    snprintf(l->want[2], sizeof l->want[2], "%lu", (unsigned long)c->band_hz);
    snprintf(l->set[2], sizeof l->set[2], "AT+BAND=%lu", (unsigned long)c->band_hz);

    snprintf(l->query[3], sizeof l->query[3], "AT+PARAMETER?"); snprintf(l->key[3], sizeof l->key[3], "PARAMETER");
    snprintf(l->want[3], sizeof l->want[3], "%u,%u,%u,%u", c->sf, c->bw_code, c->cr_code, c->preamble);
    snprintf(l->set[3], sizeof l->set[3], "AT+PARAMETER=%u,%u,%u,%u", c->sf, c->bw_code, c->cr_code, c->preamble);

    snprintf(l->query[4], sizeof l->query[4], "AT+CRFOP?");     snprintf(l->key[4], sizeof l->key[4], "CRFOP");
    snprintf(l->want[4], sizeof l->want[4], "%u", c->tx_dbm);
    snprintf(l->set[4], sizeof l->set[4], "AT+CRFOP=%u", c->tx_dbm);
}

/* "09" equals "9": compare comma-separated numbers numerically. */
static int values_match(const char *got, const char *want)
{
    if (want[0] == '*') return 1;
    for (;;) {
        char *e1, *e2;
        unsigned long a = strtoul(got, &e1, 10), b = strtoul(want, &e2, 10);
        if (e1 == got || e2 == want || a != b) return 0;
        got = e1; want = e2;
        if (*got == ',' && *want == ',') { got++; want++; continue; }
        return (*got == '\0' || *got == ' ') && *want == '\0';
    }
}

static void send_text(rylr_link_t *l, const char *text, uint32_t now)
{
    int n = snprintf(l->cmd, sizeof l->cmd, "%s\r\n", text);
    if (n > 0 && l->io.tx_start((const uint8_t *)l->cmd, (uint16_t)n) == 0) { l->waiting = 1; l->t_cmd_ms = now; }
}

static void fail(rylr_link_t *l, int code) { l->state = RL_FAILED; l->last_error = code; l->waiting = 0; }

static void restart_config(rylr_link_t *l)
{
    l->state = RL_PROBE; l->item = 0; l->tries = 0; l->waiting = 0;
    l->rx_bytes_at_start = l->rx_bytes;
}

static void on_line(rylr_link_t *l, uint32_t now)
{
    rylr_parse_line(l->lines.buf, &l->msg);
    const rylr_msg_t *m = &l->msg;
    (void)now;

    if (m->kind == RYLR_MSG_RCV) {
        l->rcv_count++;
        if (l->on_rcv) l->on_rcv(l->on_rcv_ctx, m->addr, m->data, m->len, m->rssi, m->snr);
        return;
    }
    if (m->kind == RYLR_MSG_READY) {                    /* the module (re)started: it may have lost settings */
        if (l->state == RL_READY) { l->reconfigs++; if (l->tx_pending) { l->tx_pending = 0; l->tx_timeout++; } restart_config(l); }
        return;
    }

    switch (l->state) {
    case RL_PROBE:
        if (l->waiting && m->kind == RYLR_MSG_OK) { l->state = RL_QUERY; l->item = 0; l->tries = 0; l->waiting = 0; }
        break;
    case RL_QUERY:
        if (!l->waiting) break;
        if (m->kind == RYLR_MSG_VALUE && strncmp(m->key, l->key[l->item], strlen(l->key[l->item])) == 0) {
            l->waiting = 0; l->tries = 0;
            if (values_match(m->value, l->want[l->item])) {
                if (++l->item >= RYLR_ITEMS) l->state = RL_READY;
            } else {
                l->state = RL_SET;
            }
        } else if (m->kind == RYLR_MSG_ERR) {
            fail(l, RYLR_ERR_REJECTED - m->err);
        }
        break;
    case RL_SET:
        if (!l->waiting) break;
        if (m->kind == RYLR_MSG_OK)       { l->waiting = 0; l->tries = 0; l->state = RL_QUERY; }      /* set, then read it back to confirm */
        else if (m->kind == RYLR_MSG_ERR) { fail(l, RYLR_ERR_REJECTED - m->err); }
        break;
    case RL_READY:
        if (l->tx_pending) {
            if (m->kind == RYLR_MSG_OK)       { l->tx_pending = 0; l->tx_ok++; }
            else if (m->kind == RYLR_MSG_ERR) { l->tx_pending = 0; l->tx_err++; l->last_error = RYLR_ERR_REJECTED - m->err; }
        }
        break;
    default: break;
    }
}

void rylr_link_poll(rylr_link_t *l, uint32_t now)
{
    uint8_t b;
    while (l->io.read_byte(&b)) {
        l->rx_bytes++;
        if (rylr_lines_feed(&l->lines, b)) on_line(l, now);
    }

    if (l->state == RL_READY) {
        if (l->tx_pending && (uint32_t)(now - l->tx_deadline_ms) < 0x80000000u && now != l->tx_deadline_ms) {
            l->tx_pending = 0; l->tx_timeout++;                    /* no +OK or +ERR in time */
        }
        return;
    }
    if (l->state == RL_FAILED) return;

    uint32_t limit = l->state == RL_PROBE ? PROBE_TIMEOUT_MS : l->state == RL_QUERY ? QUERY_TIMEOUT_MS : SET_TIMEOUT_MS;
    if (l->waiting && (uint32_t)(now - l->t_cmd_ms) > limit) {
        l->waiting = 0;
        if (++l->tries >= MAX_TRIES) {
            fail(l, (l->rx_bytes == l->rx_bytes_at_start) ? RYLR_ERR_NO_RESPONSE : RYLR_ERR_BAD_REPLY);
            return;
        }
    }
    if (!l->waiting && l->io.tx_idle()) {
        if (l->state == RL_PROBE)      send_text(l, "AT", now);
        else if (l->state == RL_QUERY) send_text(l, l->query[l->item], now);
        else if (l->state == RL_SET)   { send_text(l, l->set[l->item], now); l->set_commands++; }
    }
}

int rylr_link_init(rylr_link_t *l, const rylr_io_t *io, const rylr_settings_t *cfg)
{
    memset(l, 0, sizeof *l);
    l->io = *io; l->cfg = *cfg;
    build_items(l);
    rylr_lines_init(&l->lines);

    uint32_t t0 = l->io.now_ms();
    uint8_t b;
    while (l->io.now_ms() - t0 < 30u) { while (l->io.read_byte(&b)) { } l->io.delay_ms(1); }   /* discard boot chatter */
    l->rx_bytes = 0;
    restart_config(l);
    t0 = l->io.now_ms();
    while (l->state != RL_READY && l->state != RL_FAILED && l->io.now_ms() - t0 < INIT_LIMIT_MS) {
        rylr_link_poll(l, l->io.now_ms());
        l->io.delay_ms(1);
    }
    if (l->state == RL_READY) return RYLR_OK;
    if (l->state != RL_FAILED) fail(l, RYLR_ERR_NO_RESPONSE);
    return l->last_error;
}

void rylr_link_set_rcv(rylr_link_t *l, rylr_rcv_fn fn, void *ctx) { l->on_rcv = fn; l->on_rcv_ctx = ctx; }

int rylr_link_busy(const rylr_link_t *l) { return l->tx_pending; }

/* Shared by the binary and the text sender: checks that a send is allowed now, starts it, and sets the deadline. */
static int start_send(rylr_link_t *l, size_t len, uint16_t module_payload_bytes, uint32_t now)
{
    l->last_airtime_us = lora_airtime_us(l->cfg.sf, rylr_bw_hz(l->cfg.bw_code), (uint8_t)(l->cfg.cr_code + 4),
                                         l->cfg.preamble, (uint16_t)(module_payload_bytes + 8), 1, 0);   /* the module adds 8 bytes of its own */
    if (l->io.tx_start((const uint8_t *)l->cmd, (uint16_t)len) != 0) return -1;
    l->tx_pending = 1;
    l->tx_started_ms = now ? now : 1u;
    l->tx_deadline_ms = l->tx_started_ms + l->last_airtime_us / 1000u + TX_MARGIN_MS;
    return 0;
}

static int may_send(const rylr_link_t *l, uint32_t now)
{
    if (l->state != RL_READY || l->tx_pending || !l->io.tx_idle()) return 0;
    /* a second AT+SEND before the first transmission has ended makes the module answer +ERR=17 */
    if (l->tx_started_ms != 0 && (uint32_t)(now - l->tx_started_ms) < l->last_airtime_us / 1000u + 50u) return 0;
    return 1;
}

int rylr_link_send(rylr_link_t *l, const uint8_t *bin, size_t n, uint32_t now)
{
    if (!may_send(l, now)) return -1;
    size_t len = rylr_build_send(l->cmd, sizeof l->cmd, l->cfg.dest_addr, bin, n);
    if (len == 0) return -1;
    return start_send(l, len, (uint16_t)(2 * n), now);
}

int rylr_link_send_text(rylr_link_t *l, const char *text, size_t n, uint32_t now)
{
    if (!may_send(l, now)) return -1;
    size_t len = rylr_build_send_text(l->cmd, sizeof l->cmd, l->cfg.dest_addr, text, n);
    if (len == 0) return -1;
    return start_send(l, len, (uint16_t)n, now);
}
