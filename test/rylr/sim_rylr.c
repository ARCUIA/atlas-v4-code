#include "sim_rylr.h"
#include "lora_airtime.h"
#include "rylr_link.h"      /* rylr_bw_hz */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t sim_now_ms;

static void emit_at(sim_mod_t *m, uint32_t at, const char *s)
{
    for (; *s && m->ot < (int)(sizeof m->out / sizeof m->out[0]); s++) { m->out[m->ot].b = (uint8_t)*s; m->out[m->ot].at = at; m->ot++; }
}
static void reply(sim_mod_t *m, uint32_t delay_ms, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
#include <stdarg.h>
static void reply(sim_mod_t *m, uint32_t delay_ms, const char *fmt, ...)
{
    char t[200]; va_list ap; va_start(ap, fmt); vsnprintf(t, sizeof t, fmt, ap); va_end(ap);
    if (m->dead || m->garbage) return;
    char u[220]; snprintf(u, sizeof u, "%s\r\n", t);
    emit_at(m, sim_now_ms + delay_ms, u);
}

void sim_mod_init(sim_mod_t *m)
{
    memset(m, 0, sizeof *m);
    m->address = 0; m->network = 18; m->band = 915000000u; m->sf = 9; m->bw = 7; m->cr = 1; m->pre = 12; m->crfop = 22;
}
void sim_mod_reboot(sim_mod_t *m)
{
    m->band = 915000000u; m->sf = 9; m->bw = 7; m->cr = 1; m->pre = 12; m->crfop = 22;
    m->tx_busy_until = 0; m->ln = 0; m->npend = 0;
    if (!m->dead) { reply(m, 5, "+RESET"); reply(m, 6, "+READY"); }
}
void sim_pair(sim_mod_t *a, sim_mod_t *b) { a->peer = b; b->peer = a; }

static int params_ok(const sim_mod_t *m, int sf, int bw, int cr, int pre, int *err)
{
    *err = 4;
    if (sf < 5 || sf > 11 || bw < 7 || bw > 9 || cr < 1 || cr > 4) return 0;
    if (bw == 7 && sf > 9)  return 0;               /* "SF7 to SF9 at 125 kHz, SF7 to SF10 at 250 kHz, and SF7 to SF11 at 500 kHz" */
    if (bw == 8 && sf > 10) return 0;
    if (m->network == 18) { if (pre < 4 || pre > 24) { *err = 18; return 0; } }
    else if (pre != 12)   { *err = 18; return 0; }
    return 1;
}

static void deliver_later(sim_mod_t *src, uint16_t dest, const char *data, int len, uint32_t air_ms)
{
    sim_mod_t *p = src->peer;
    if (!p || p->dead) return;
    if (p->network != src->network || p->band != src->band || p->sf != src->sf || p->bw != src->bw ||
        p->cr != src->cr || p->pre != src->pre) return;                       /* different RF settings: not heard */
    if (dest != 0 && dest != p->address) return;                              /* addressed to someone else */
    if (p->npend >= 16) return;
    sim_pending_t *e = &p->pend[p->npend++];
    e->at = sim_now_ms + air_ms;
    snprintf(e->text, sizeof e->text, "+RCV=%u,%d,%.*s,-90,10\r\n", src->address, len, len, data);
}

static void handle(sim_mod_t *m, char *s)
{
    unsigned long a, b, c, d;
    if (strcmp(s, "AT") == 0) { reply(m, 5, "+OK"); return; }
    if (strcmp(s, "AT+ADDRESS?") == 0)   { reply(m, 5, "+ADDRESS=%u", m->address); return; }
    if (strcmp(s, "AT+NETWORKID?") == 0) { reply(m, 5, "+NETWORK=%u", m->network); return; }
    if (strcmp(s, "AT+BAND?") == 0)      { reply(m, 5, "+BAND=%u", (unsigned)m->band); return; }
    if (strcmp(s, "AT+PARAMETER?") == 0) { reply(m, 5, "+PARAMETER=%u,%u,%u,%u", m->sf, m->bw, m->cr, m->pre); return; }
    if (strcmp(s, "AT+CRFOP?") == 0)     { reply(m, 5, "+CRFOP=%02u", m->crfop); return; }
    if (sscanf(s, "AT+ADDRESS=%lu", &a) == 1)   { if (a > 65535) { reply(m, 5, "+ERR=4"); return; } m->address = (uint16_t)a; m->flash_writes++; reply(m, 80, "+OK"); return; }
    if (sscanf(s, "AT+NETWORKID=%lu", &a) == 1) {
        if (!((a >= 3 && a <= 15) || a == 18)) { reply(m, 5, "+ERR=4"); return; }
        m->network = (uint16_t)a; m->flash_writes++; reply(m, 80, "+OK"); return;
    }
    if (sscanf(s, "AT+BAND=%lu", &a) == 1) { if (a < 150000000ul || a > 960000000ul) { reply(m, 5, "+ERR=4"); return; } m->band = (uint32_t)a; reply(m, 10, "+OK"); return; }
    if (sscanf(s, "AT+PARAMETER=%lu,%lu,%lu,%lu", &a, &b, &c, &d) == 4) {
        int err; if (!params_ok(m, (int)a, (int)b, (int)c, (int)d, &err)) { reply(m, 5, "+ERR=%d", err); return; }
        m->sf = (uint8_t)a; m->bw = (uint8_t)b; m->cr = (uint8_t)c; m->pre = (uint8_t)d; reply(m, 10, "+OK"); return;
    }
    if (sscanf(s, "AT+CRFOP=%lu", &a) == 1) { if (a > 22) { reply(m, 5, "+ERR=4"); return; } m->crfop = (uint8_t)a; reply(m, 10, "+OK"); return; }
    if (strncmp(s, "AT+SEND=", 8) == 0) {
        unsigned dest, len; int off = 0;
        if (sscanf(s + 8, "%u,%u,%n", &dest, &len, &off) < 2 || off == 0) { reply(m, 5, "+ERR=4"); return; }
        const char *data = s + 8 + off;
        if (strlen(data) != len)  { reply(m, 5, "+ERR=5");  m->errors_sent++; return; }     /* data does not match the stated length */
        if (len > 240)            { reply(m, 5, "+ERR=13"); m->errors_sent++; return; }
        if (sim_now_ms < m->tx_busy_until) { reply(m, 5, "+ERR=17"); m->errors_sent++; return; }   /* last TX not completed */
        uint32_t air_ms = lora_airtime_us(m->sf, rylr_bw_hz(m->bw), (uint8_t)(m->cr + 4), m->pre, (uint16_t)(len + 8), 1, 0) / 1000u;
        m->tx_busy_until = sim_now_ms + air_ms; m->sent++;
        deliver_later(m, (uint16_t)dest, data, (int)len, air_ms);
        reply(m, m->ok_after_air ? air_ms + 2 : 5, "+OK");
        return;
    }
    reply(m, 5, "+ERR=4");
}

void sim_host_write(sim_mod_t *m, const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        if (m->ln < (int)sizeof m->line - 1) m->line[m->ln++] = (char)p[i];
        if (p[i] == '\n') {
            m->line[m->ln] = '\0';
            int ok = m->ln >= 2 && m->line[m->ln - 2] == '\r';
            m->line[m->ln - (ok ? 2 : 1)] = '\0';
            if (!ok) reply(m, 5, "+ERR=1");                      /* the guide requires \r\n at the end of every command */
            else handle(m, m->line);
            m->ln = 0;
        }
    }
}

int sim_host_read(sim_mod_t *m, uint8_t *b)
{
    for (int i = 0; i < m->npend; ) {                            /* packets whose airtime is over reach the host now */
        if (m->pend[i].at <= sim_now_ms) {
            if (!m->dead && !m->garbage) emit_at(m, sim_now_ms, m->pend[i].text);
            m->delivered++;
            m->pend[i] = m->pend[--m->npend];
        } else i++;
    }
    if (m->garbage) { static uint32_t x = 7; x = x * 1664525u + 1013904223u; if ((x >> 28) < 4) { *b = (uint8_t)(x >> 8); return 1; } return 0; }
    if (m->oh < m->ot && m->out[m->oh].at <= sim_now_ms) { *b = m->out[m->oh++].b; if (m->oh == m->ot) m->oh = m->ot = 0; return 1; }
    return 0;
}
