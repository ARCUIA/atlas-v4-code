#include "usbstream.h"
#include <stddef.h>

#define LINE_MAX 200u

/* ---- tiny append-only formatter ---- */
typedef struct { char *p; char *end; } W;
static void wc(W *w, char c) { if (w->p < w->end) *w->p++ = c; }
static void ws(W *w, const char *s) { while (*s) wc(w, *s++); }
static void wu(W *w, uint32_t u)
{
    char t[10]; int n = 0;
    do { t[n++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (n) wc(w, t[--n]);
}
static void wi(W *w, int32_t v)
{
    if (v < 0) { wc(w, '-'); wu(w, (uint32_t)(-(v + 1)) + 1u); }   /* safe for INT32_MIN */
    else wu(w, (uint32_t)v);
}
/* v is a value scaled by 10^dec; prints [-]int.frac and keeps the sign for -0.xxx */
static void wfix(W *w, int32_t v, int dec)
{
    uint32_t div = 1u;
    for (int i = 0; i < dec; i++) div *= 10u;
    uint32_t a = (v < 0) ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;
    if (v < 0) wc(w, '-');
    wu(w, a / div);
    wc(w, '.');
    uint32_t f = a % div;
    for (uint32_t d = div / 10u; d; d /= 10u) wc(w, (char)('0' + (f / d) % 10u));
}
static void wt(W *w, uint64_t t_us)
{
    wu(w, (uint32_t)(t_us / 1000000ull));
    wc(w, '.');
    uint32_t us = (uint32_t)(t_us % 1000000ull);
    for (uint32_t d = 100000u; d; d /= 10u) wc(w, (char)('0' + (us / d) % 10u));
}
static void v3(W *w, const int32_t v[3])
{
    wi(w, v[0]); wc(w, ','); wi(w, v[1]); wc(w, ','); wi(w, v[2]);
}
static int32_t clamp_acc(uint32_t v) { return (v > 999999999u) ? 999999999 : (int32_t)v; }

/* ---- store a finished line whole, or drop it whole ---- */
static void put(usbstream_t *s, const char *line, uint32_t n)
{
    if (ringbuf_space(&s->rb) < n) { s->lines_dropped++; return; }
    (void)ringbuf_write(&s->rb, (const uint8_t *)line, n);
    s->lines++;
}
#define BEGIN() char buf[LINE_MAX]; W w = { buf, buf + LINE_MAX - 2 }
#define END()   do { wc(&w, '\r'); wc(&w, '\n'); put(s, buf, (uint32_t)(w.p - buf)); } while (0)

int usbstream_init(usbstream_t *s, uint8_t *storage, uint32_t size)
{
    s->lines = s->lines_dropped = s->bytes_flushed = 0;
    return ringbuf_init(&s->rb, storage, size);
}

void usbstream_imu(usbstream_t *s, uint64_t t_us, const int32_t xl[3], const int32_t gy[3], const int32_t *hg)
{
    BEGIN(); ws(&w, "IMU t="); wt(&w, t_us);
    ws(&w, " xl[mg]="); v3(&w, xl); ws(&w, " gy[mdps]="); v3(&w, gy);
    if (hg) { ws(&w, " hg[mg]="); v3(&w, hg); }
    END();
}

void usbstream_mag(usbstream_t *s, uint64_t t_us, const int32_t m[3])
{
    BEGIN(); ws(&w, "MAG t="); wt(&w, t_us); ws(&w, " mag[mG]="); v3(&w, m); END();
}

void usbstream_baro(usbstream_t *s, uint64_t t_us, int32_t p_pa, int32_t temp_cC)
{
    BEGIN(); ws(&w, "BARO t="); wt(&w, t_us);
    ws(&w, " p[Pa]="); wi(&w, p_pa); ws(&w, " T[C]="); wfix(&w, temp_cC, 2); END();
}

void usbstream_gps(usbstream_t *s, uint64_t t_us, int fix, int sv, int32_t lat, int32_t lon,
                   int32_t hmsl, int32_t vd, uint32_t hacc, uint32_t vacc)
{
    BEGIN(); ws(&w, "GPS t="); wt(&w, t_us);
    ws(&w, " fix="); wi(&w, fix); ws(&w, " sv="); wi(&w, sv);
    ws(&w, " lat="); wfix(&w, lat, 7); ws(&w, " lon="); wfix(&w, lon, 7);
    ws(&w, " hmsl[m]="); wfix(&w, hmsl, 3); ws(&w, " vd[mps]="); wfix(&w, vd, 3);
    ws(&w, " hacc[m]="); wfix(&w, clamp_acc(hacc), 3); ws(&w, " vacc[m]="); wfix(&w, clamp_acc(vacc), 3);
    END();
}

void usbstream_text(usbstream_t *s, const char *line, uint32_t n)
{
    if (n) put(s, line, n);
}

uint32_t usbstream_pending(const usbstream_t *s) { return ringbuf_count(&s->rb); }
uint32_t usbstream_peek(const usbstream_t *s, uint8_t *out, uint32_t max) { return ringbuf_peek(&s->rb, out, max); }
void     usbstream_consume(usbstream_t *s, uint32_t n) { (void)ringbuf_drop(&s->rb, n); }
void     usbstream_flush(usbstream_t *s)
{
    uint32_t n = ringbuf_count(&s->rb);
    if (n) { s->bytes_flushed += n; (void)ringbuf_drop(&s->rb, n); }
}
