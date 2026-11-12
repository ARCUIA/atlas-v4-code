/* Host test for app/core/usbstream.c: exact line formats, whole-line drops, peek/consume/flush. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usbstream.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static uint32_t drain(usbstream_t *s, char *out, uint32_t cap)
{
    uint32_t n = usbstream_peek(s, (uint8_t *)out, cap - 1);
    usbstream_consume(s, n);
    out[n] = 0;
    return n;
}

static void test_formats(void)
{
    static uint8_t store[4096]; usbstream_t s; char out[2048];
    CHECK(usbstream_init(&s, store, sizeof store) == 0);
    int32_t a[3] = {-86, -99, 988}, g[3] = {-280, 210, 140};
    usbstream_imu(&s, 1234567890ull, a, g, NULL);
    int32_t hg[3] = {-1, 0, 80000};
    int32_t a2[3] = {-1, 0, 1000}, g2[3] = {2293000, -2293000, 5};
    usbstream_imu(&s, 1234570073ull, a2, g2, hg);
    int32_t m[3] = {123, -45, 300};
    usbstream_mag(&s, 5ull, m);
    usbstream_baro(&s, 1234567890ull, 98717, 2130);
    usbstream_baro(&s, 1234567890ull, 99000, -5);                     /* -0.05 C keeps its sign */
    usbstream_gps(&s, 1234567890ull, 3, 9, 416611234, -915301234, 221600, 100, 400, 700);
    usbstream_gps(&s, 1234600000ull, 0, 0, -123, 0, -500, -50, 0xFFFFFFFFu, 3750000u);
    usbstream_text(&s, "IMU:OK status\r\n", (uint32_t)strlen("IMU:OK status\r\n"));
    drain(&s, out, sizeof out);
    const char *exp =
        "IMU t=1234.567890 xl[mg]=-86,-99,988 gy[mdps]=-280,210,140\r\n"
        "IMU t=1234.570073 xl[mg]=-1,0,1000 gy[mdps]=2293000,-2293000,5 hg[mg]=-1,0,80000\r\n"
        "MAG t=0.000005 mag[mG]=123,-45,300\r\n"
        "BARO t=1234.567890 p[Pa]=98717 T[C]=21.30\r\n"
        "BARO t=1234.567890 p[Pa]=99000 T[C]=-0.05\r\n"
        "GPS t=1234.567890 fix=3 sv=9 lat=41.6611234 lon=-91.5301234 hmsl[m]=221.600 vd[mps]=0.100 hacc[m]=0.400 vacc[m]=0.700\r\n"
        "GPS t=1234.600000 fix=0 sv=0 lat=-0.0000123 lon=0.0000000 hmsl[m]=-0.500 vd[mps]=-0.050 hacc[m]=999999.999 vacc[m]=3750.000\r\n"
        "IMU:OK status\r\n";
    if (strcmp(out, exp) != 0) { printf("--- got\n%s--- expected\n%s", out, exp); fails++; }
    CHECK(s.lines == 8 && s.lines_dropped == 0);
}

static void test_full_ring(void)
{
    static uint8_t store[256]; usbstream_t s; char out[512];
    usbstream_init(&s, store, sizeof store);
    int32_t a[3] = {-86, -99, 988}, g[3] = {-280, 210, 140};
    for (int i = 0; i < 20; i++) usbstream_imu(&s, 1000000ull * (uint64_t)i, a, g, NULL);   /* ~60 B each */
    CHECK(s.lines_dropped > 0 && s.lines + s.lines_dropped == 20);
    uint32_t n = drain(&s, out, sizeof out);
    /* every stored line must be complete: starts with "IMU t=", ends with \r\n, count matches */
    uint32_t nl = 0; char *p = out;
    while (*p) {
        CHECK(strncmp(p, "IMU t=", 6) == 0);
        char *e = strstr(p, "\r\n"); CHECK(e != NULL); if (!e) break;
        p = e + 2; nl++;
    }
    CHECK(nl == s.lines && n > 0);
    /* space is usable again after draining */
    usbstream_imu(&s, 99ull, a, g, NULL); CHECK(usbstream_pending(&s) > 0);
}

static void test_peek_consume_flush(void)
{
    static uint8_t store[1024]; usbstream_t s; uint8_t b1[32], b2[32];
    usbstream_init(&s, store, sizeof store);
    int32_t m[3] = {1, 2, 3};
    usbstream_mag(&s, 1ull, m);
    uint32_t pend = usbstream_pending(&s);
    uint32_t n1 = usbstream_peek(&s, b1, sizeof b1), n2 = usbstream_peek(&s, b2, sizeof b2);
    CHECK(pend > 10 && pend <= 32 && n1 == pend && n2 == pend && memcmp(b1, b2, pend) == 0 && usbstream_pending(&s) == pend);   /* peek does not consume */
    usbstream_consume(&s, 10); CHECK(usbstream_pending(&s) == pend - 10);
    usbstream_flush(&s); CHECK(usbstream_pending(&s) == 0 && s.bytes_flushed == pend - 10);
    usbstream_text(&s, "x", 0); CHECK(usbstream_pending(&s) == 0);                          /* empty text ignored */
}

int main(void)
{
    test_formats(); test_full_ring(); test_peek_consume_flush();
    if (fails) { printf("%d FAILURES\n", fails); return 1; }
    printf("usbstream: all checks passed\n");
    return 0;
}
