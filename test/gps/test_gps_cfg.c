/* Runs the real configuration sequence (app/core/gps_cfg.c) against a simulated u-blox receiver.
 *
 * The simulator models real UART timing: bytes arrive at the baud rate, one at a time. Two receive
 * styles are tested:
 *   RING   : an interrupt stores each byte in a buffer (what the firmware must do)
 *   POLLED : a single-byte data register read in a loop with 1 ms pauses (what the first version did) */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "gps_cfg.h"
#include "ubx.h"

enum { MODE_RING = 1, MODE_POLLED = 0 };

typedef struct { uint8_t b; uint64_t at; } timed_byte_t;

typedef struct {
    int present; uint32_t baud;
    int nmea_on, pvt_rate, nak, drop_ack;
    uint32_t meas_ms, dynmodel;
    int gps_ena, gal_ena, bds_ena, glo_ena, qzss_ena, sbas_ena;
    timed_byte_t q[16384]; int qh, qt; uint64_t last_at;      /* module -> host, with arrival times */
    uint32_t next_pvt, next_nmea;
    ubx_parser_t parser;
    uint32_t rng;
    /* host UART receive side */
    uint8_t ring[16384]; int rh, rt;
    uint8_t dr; int dr_valid, ore;
    unsigned long overruns;
} sim_t;
static sim_t S;
static uint64_t t_us;
static uint32_t host_baud;
static int mode = MODE_RING;

static uint32_t now_ms_sim(void) { return (uint32_t)(t_us / 1000u); }
static uint64_t byte_time_us(uint32_t baud) { return (10ull * 1000000ull) / baud; }

static void push(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n && S.qt < (int)(sizeof S.q / sizeof S.q[0]); i++) {
        uint64_t at = (S.last_at > t_us ? S.last_at : t_us) + byte_time_us(S.baud);
        S.q[S.qt].b = p[i]; S.q[S.qt].at = at; S.qt++; S.last_at = at;
    }
}
static void push_frame(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{ uint8_t f[120]; size_t n = ubx_build(f, cls, id, pl, len); push(f, n); }
static void push_pvt(void)
{
    uint8_t pv[92]; memset(pv, 0, sizeof pv);
    uint32_t itow = now_ms_sim(); memcpy(pv, &itow, 4);
    pv[20] = 3; pv[21] = 1; pv[23] = 9;
    push_frame(1, 7, pv, 92);
}
static void sim_run(void)                                  /* module's periodic output up to "now" */
{
    if (!S.present) return;
    uint32_t ms = now_ms_sim();
    if (S.pvt_rate > 0) while (S.next_pvt <= ms) { push_pvt(); S.next_pvt += S.meas_ms; }
    if (S.nmea_on) while (S.next_nmea <= ms) {
        const char *t = "$GNGGA,000000.00,4137.0,N,09130.0,W,1,09,0.9,215.5,M,,M,,*5A\r\n";
        push((const uint8_t *)t, strlen(t)); S.next_nmea += 1000;
    }
}
/* Deliver every byte that has finished arriving, in order, into the host UART. */
static void uart_advance(void)
{
    while (S.qh < S.qt && S.q[S.qh].at <= t_us) {
        uint8_t b = S.q[S.qh++].b;
        if (mode == MODE_RING) { S.ring[S.rt++ % sizeof S.ring] = b; }
        else if (S.dr_valid) { S.ore = 1; S.overruns++; }   /* data register still full: the new byte is lost */
        else { S.dr = b; S.dr_valid = 1; }
    }
    if (S.qh == S.qt) S.qh = S.qt = 0;
}

static uint32_t key_val(const uint8_t *p, int *size)
{
    static const int sz[8] = { 0, 1, 1, 2, 4, 8, 0, 0 };
    uint32_t key = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    *size = sz[(key >> 28) & 7];
    return key;
}

static void handle_frame(void)
{
    ubx_parser_t *p = &S.parser;
    if (p->cls == 1 && p->id == 7 && p->len == 0) { push_pvt(); return; }                 /* poll */
    if (p->cls == 6 && p->id == 0x8A) {
        uint32_t new_baud = 0; int pos = 4;
        while (pos + 4 <= p->len) {
            int size; uint32_t key = key_val(p->payload + pos, &size); pos += 4;
            uint32_t val = 0; for (int i = 0; i < size; i++) val |= (uint32_t)p->payload[pos + i] << (8 * i); pos += size;
            switch (key) {
            case 0x40520001u: new_baud = val; break;
            case 0x10740002u: S.nmea_on = (int)val; break;
            case 0x30210001u: S.meas_ms = val; break;
            case 0x20910007u: S.pvt_rate = (int)val; if (val) S.next_pvt = now_ms_sim() + S.meas_ms; break;
            case 0x20110021u: S.dynmodel = val; break;
            case 0x1031001fu: S.gps_ena = (int)val; break;
            case 0x10310021u: S.gal_ena = (int)val; break;
            case 0x10310022u: S.bds_ena = (int)val; break;
            case 0x10310025u: S.glo_ena = (int)val; break;
            case 0x10310024u: S.qzss_ena = (int)val; break;
            case 0x10310020u: S.sbas_ena = (int)val; break;
            default: break;
            }
        }
        uint8_t ack[2] = { 6, 0x8A };
        if (!S.drop_ack) push_frame(5, S.nak ? 0x00 : 0x01, ack, 2);
        if (new_baud) { S.baud = new_baud; S.qh = S.qt = 0; S.last_at = 0; }   /* receiver switches rate; the ACK is lost on the way */
    }
}

/* ---- the gps_io_t the sequence runs against ---- */
static void io_set_baud(uint32_t b)
{
    host_baud = b;
    S.rh = S.rt = 0; S.dr_valid = 0; S.ore = 0;             /* re-initialising the UART clears its receive state */
}
static void io_write(const uint8_t *p, uint16_t n)
{
    t_us += (uint64_t)n * byte_time_us(host_baud);          /* blocking transmit takes real time */
    if (!S.present || host_baud != S.baud) return;          /* receiver hears garbage at the wrong rate */
    for (uint16_t i = 0; i < n; i++) if (ubx_parser_feed(&S.parser, p[i])) handle_frame();
}
static int io_read_byte(uint8_t *b)
{
    t_us += 2;                                              /* a few CPU cycles per call */
    if (!S.present) return 0;
    if (host_baud != S.baud) {                              /* wrong rate: framing garbage, never valid frames */
        S.rng = S.rng * 1664525u + 1013904223u;
        if ((S.rng >> 24) < 64) { *b = (uint8_t)((S.rng >> 8) & 0x7F); return 1; }
        return 0;
    }
    sim_run();
    uart_advance();
    if (mode == MODE_RING) {
        if (S.rh != S.rt) { *b = S.ring[S.rh++ % sizeof S.ring]; return 1; }
        return 0;
    }
    if (S.dr_valid || S.ore) {                              /* polled: read DR; a flagged byte is dropped */
        uint8_t d = S.dr; S.dr_valid = 0;
        if (S.ore) { S.ore = 0; return 0; }
        *b = d; return 1;
    }
    return 0;
}
static uint32_t io_now(void) { t_us += 1; return now_ms_sim(); }
static void io_delay(uint32_t ms) { t_us += (uint64_t)ms * 1000u; if (S.present && host_baud == S.baud) { sim_run(); uart_advance(); } }
static const gps_io_t io = { io_set_baud, io_write, io_read_byte, io_now, io_delay };

static void fresh(uint32_t module_baud)
{
    memset(&S, 0, sizeof S);
    S.present = 1; S.baud = module_baud; S.nmea_on = 1; S.meas_ms = 1000; S.rng = 12345;
    S.gps_ena = 1; S.gal_ena = 1; S.bds_ena = 1; S.glo_ena = 1; S.qzss_ena = 1; S.sbas_ena = 1;
    S.next_nmea = 5000 + 1000; ubx_parser_init(&S.parser);
    host_baud = 115200; t_us = 5000ull * 1000u;
}

static const gps_cfg_t cfg = { 115200u, 56u, 1, 8 };

int main(void)
{
    gps_cfg_result_t r;

    /* ---------- the failure seen on the real board: polled single-byte reads lose most of a reply ---------- */
    mode = MODE_POLLED; fresh(38400);
    int rc = gps_configure(&io, &cfg, &r);
    printf("POLLED receive, module at 38400: result %d, %lu overruns\n", rc, S.overruns);
    assert(rc != GPSCFG_OK);
    puts("  -> configuration fails, as seen on the board: OK (documents why bytes must be buffered)");

    /* ---------- the fix: every byte is buffered by an interrupt ---------- */
    mode = MODE_RING;
    fresh(38400);                                           /* factory-fresh receiver */
    assert(gps_configure(&io, &cfg, &r) == GPSCFG_OK);
    assert(r.baud_found == 38400 && r.baud_final == 115200 && S.baud == 115200 && host_baud == 115200);
    assert(S.meas_ms == 56 && S.dynmodel == 8 && S.pvt_rate == 1 && S.nmea_on == 0);
    assert(S.gps_ena == 1 && !S.gal_ena && !S.bds_ena && !S.glo_ena && !S.qzss_ena && !S.sbas_ena);
    puts("RING receive, fresh receiver at 38400: found, moved to 115200, configured: OK");

    uint64_t t0 = t_us; int frames = 0; uint8_t b; ubx_parser_t pp; ubx_parser_init(&pp);
    while (t_us - t0 < 2000000) { while (io_read_byte(&b)) if (ubx_parser_feed(&pp, b) && pp.cls == 1 && pp.id == 7) frames++; t_us += 500; }
    assert(frames >= 34 && frames <= 37);                   /* 2 s at 56 ms is about 35.7 */
    printf("position messages in 2 s: %d (about %.1f Hz)\n", frames, frames / 2.0);

    fresh(115200);                                          /* already at the target rate (MCU reset, receiver kept power) */
    S.pvt_rate = 1; S.meas_ms = 56; S.next_pvt = now_ms_sim() + 56;
    assert(gps_configure(&io, &cfg, &r) == GPSCFG_OK && r.baud_found == 115200 && r.baud_final == 115200);
    puts("receiver already at target rate: OK (no baud change)");

    fresh(9600);
    assert(gps_configure(&io, &cfg, &r) == GPSCFG_OK && r.baud_found == 9600 && S.baud == 115200);
    puts("receiver at 9600: OK");

    fresh(38400); S.present = 0;                            /* nothing connected / wrong UART */
    uint64_t ts = t_us;
    assert(gps_configure(&io, &cfg, &r) == GPSCFG_NO_RESPONSE && (t_us - ts) < 6000000);
    printf("no receiver: GPSCFG_NO_RESPONSE after %u ms\n", (unsigned)((t_us - ts) / 1000));

    fresh(38400); S.nak = 1;
    assert(gps_configure(&io, &cfg, &r) == GPSCFG_NAK);
    puts("receiver rejects the configuration: GPSCFG_NAK");

    fresh(115200); S.drop_ack = 1;
    assert(gps_configure(&io, &cfg, &r) == GPSCFG_NO_ACK);
    puts("no acknowledgement: GPSCFG_NO_ACK");

    puts("ALL GPS CONFIG TESTS PASSED");
    return 0;
}
