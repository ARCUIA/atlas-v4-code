#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "ubx.h"

static void feed_all(ubx_parser_t *p, const uint8_t *d, size_t n, int *frames)
{
    for (size_t i = 0; i < n; i++) if (ubx_parser_feed(p, d[i])) (*frames)++;
}

int main(void)
{
    uint8_t f[300];

    /* Known frames from the u-blox protocol documentation: confirms our checksum and framing. */
    size_t n = ubx_build(f, 0x01, 0x07, NULL, 0);                      /* UBX-NAV-PVT poll */
    static const uint8_t poll[] = { 0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19 };
    assert(n == sizeof poll && memcmp(f, poll, n) == 0);
    static const uint8_t rate_pl[] = { 0xE8,0x03,0x01,0x00,0x01,0x00 };  /* UBX-CFG-RATE: 1000 ms */
    n = ubx_build(f, 0x06, 0x08, rate_pl, sizeof rate_pl);
    static const uint8_t rate_fr[] = { 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 };
    assert(n == sizeof rate_fr && memcmp(f, rate_fr, n) == 0);
    puts("UBX framing + checksum match the documented example frames: OK");

    /* Parser: text before a frame, back-to-back frames, a corrupted frame, an oversized frame. */
    ubx_parser_t p; ubx_parser_init(&p);
    uint8_t stream[600]; size_t k = 0;
    const char *nmea = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    memcpy(stream + k, nmea, strlen(nmea)); k += strlen(nmea);
    k += ubx_build(stream + k, 0x01, 0x07, (const uint8_t[]){1,2,3,4}, 4);
    k += ubx_build(stream + k, 0x05, 0x01, (const uint8_t[]){0x06, 0x8A}, 2);
    size_t bad_at = k;
    k += ubx_build(stream + k, 0x01, 0x07, (const uint8_t[]){9,9,9,9}, 4);
    stream[bad_at + 7] ^= 0xFF;                                        /* damage a payload byte */
    k += ubx_build(stream + k, 0x05, 0x00, (const uint8_t[]){0x06, 0x8A}, 2);
    int frames = 0; feed_all(&p, stream, k, &frames);
    assert(frames == 3 && p.frames_bad == 1);
    /* oversized frame is rejected, and the next good one still parses */
    uint8_t big[8] = { 0xB5, 0x62, 0x0A, 0x04, 0xFF, 0x01, 0, 0 };     /* claims a 511-byte payload */
    frames = 0; feed_all(&p, big, sizeof big, &frames);
    k = ubx_build(stream, 0x05, 0x01, (const uint8_t[]){0x06, 0x8A}, 2);
    feed_all(&p, stream, k, &frames);
    assert(frames == 1 && p.too_long == 1);
    puts("UBX parser (noise, back-to-back, bad checksum, oversize): OK");

    /* CFG-VALSET builder. */
    ubx_valset_t v; ubx_valset_begin(&v, 1);
    ubx_valset_add(&v, 0x30210001u, 56);                               /* CFG-RATE-MEAS, U2 */
    ubx_valset_add(&v, 0x1031001fu, 1);                                /* CFG-SIGNAL-GPS_ENA, L (1 byte) */
    ubx_valset_add(&v, 0x40520001u, 115200);                           /* CFG-UART1-BAUDRATE, U4 */
    static const uint8_t want[] = {
        0x00, 0x01, 0x00, 0x00,                                        /* version, RAM layer, reserved */
        0x01,0x00,0x21,0x30, 0x38,0x00,                                /* key 0x30210001 = 56 */
        0x1F,0x00,0x31,0x10, 0x01,                                     /* key 0x1031001F = 1 */
        0x01,0x00,0x52,0x40, 0x00,0xC2,0x01,0x00                       /* key 0x40520001 = 115200 */
    };
    assert(!v.overflow && v.len == sizeof want && memcmp(v.buf, want, v.len) == 0);
    puts("CFG-VALSET payload layout: OK");

    /* NAV-PVT decode. */
    uint8_t pv[92]; memset(pv, 0, sizeof pv);
    pv[0] = 0x78; pv[1] = 0x56; pv[2] = 0x34; pv[3] = 0x12;            /* iTOW 0x12345678 */
    pv[4] = 0xEA; pv[5] = 0x07;                                        /* year 2026 */
    pv[6] = 11; pv[7] = 5; pv[8] = 20; pv[9] = 30; pv[10] = 15;
    pv[20] = 3; pv[21] = 1; pv[23] = 9;
    int32_t lat = 416600000, lon = -916500000, hmsl = 215500; uint32_t hacc = 1800;
    memcpy(pv + 28, &lat, 4); memcpy(pv + 24, &lon, 4); memcpy(pv + 36, &hmsl, 4); memcpy(pv + 40, &hacc, 4);
    int32_t vd = -123456; memcpy(pv + 56, &vd, 4);
    pv[76] = 0x96; pv[77] = 0x00;                                      /* pDOP 1.50 */
    ubx_pvt_t o; assert(ubx_pvt_decode(pv, 92, &o) == 0 && ubx_pvt_decode(pv, 91, &o) != 0);
    assert(o.itow_ms == 0x12345678u && o.year == 2026 && o.fix_type == 3 && o.num_sv == 9 && (o.flags & 1));
    assert(o.lat_e7 == 416600000 && o.lon_e7 == -916500000 && o.hmsl_mm == 215500 && o.hacc_mm == 1800);
    assert(o.vel_d_mms == -123456 && o.pdop_x100 == 150);
    puts("NAV-PVT decode: OK");
    puts("ALL UBX TESTS PASSED");
    return 0;
}
