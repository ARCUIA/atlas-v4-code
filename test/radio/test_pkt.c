#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "pkt.h"
#include "rx_format.h"

int main(void)
{
    uint8_t buf[64];
    pkt_builder_t b;
    pkt_gps_t g = { .fix_type = 3, .num_sv = 9, .lat_e7 = 416600000, .lon_e7 = -916500000,
                    .alt_dm = 2155, .gspeed_dms = 123, .hacc_m = 2 };

    pkt_begin(&b, buf, sizeof buf, 1, 0x0102);
    assert(pkt_add_gps(&b, &g) == 0 && pkt_add_batt(&b, 7400) == 0);
    size_t n = pkt_end(&b);
    assert(n == 28);                                           /* 5 header + 19 GPS + 4 battery */
    assert(buf[0] == 0xA7 && buf[1] == 0x10 && buf[2] == 1 && buf[3] == 0x02 && buf[4] == 0x01);
    assert(buf[5] == PKT_REC_GPS && buf[6] == 17 && buf[24] == PKT_REC_BATT && buf[25] == 2);
    puts("packet layout: 28 bytes, header and record framing as documented: OK");

    pkt_header_t h; assert(pkt_parse_header(buf, n, &h) == 0);
    assert(h.version == 1 && h.src_id == 1 && h.seq == 0x0102);
    pkt_iter_t it; uint8_t type, len; const uint8_t *d; int got_gps = 0, got_batt = 0;
    pkt_iter_init(&it, buf, n);
    while (pkt_iter_next(&it, &type, &d, &len) == 1) {
        if (type == PKT_REC_GPS)  { pkt_gps_t r; assert(pkt_decode_gps(d, len, &r) == 0);
            assert(r.fix_type == 3 && r.num_sv == 9 && r.lat_e7 == 416600000 && r.lon_e7 == -916500000 &&
                   r.alt_dm == 2155 && r.gspeed_dms == 123 && r.hacc_m == 2); got_gps = 1; }
        if (type == PKT_REC_BATT) { uint16_t mv; assert(pkt_decode_batt(d, len, &mv) == 0 && mv == 7400); got_batt = 1; }
    }
    assert(got_gps && got_batt);
    puts("encode -> decode round trip (negative longitude included): OK");

    /* Unknown record types from future firmware are skipped, known ones still read. */
    uint8_t buf2[64]; memcpy(buf2, buf, n);
    buf2[n] = 9; buf2[n + 1] = 3; buf2[n + 2] = 1; buf2[n + 3] = 2; buf2[n + 4] = 3;
    pkt_iter_init(&it, buf2, n + 5); int count = 0, unknown = 0;
    while (pkt_iter_next(&it, &type, &d, &len) == 1) { count++; if (type == 9) unknown++; }
    assert(count == 3 && unknown == 1);
    puts("unknown record types are skippable: OK");

    /* Bad input is rejected, never read past the end. */
    pkt_iter_init(&it, buf, n - 1);                            /* last record cut short */
    int rc, seen = 0; while ((rc = pkt_iter_next(&it, &type, &d, &len)) == 1) seen++;
    assert(rc == -1 && seen == 2 - 1);
    uint8_t bad[8] = { 0x55, 0x10, 1, 0, 0 };
    assert(pkt_parse_header(bad, 5, &h) == -1);               /* wrong magic */
    bad[0] = 0xA7; bad[1] = 0x20;
    assert(pkt_parse_header(bad, 5, &h) == -2);               /* unknown version */
    assert(pkt_parse_header(bad, 4, &h) == -1);               /* too short */
    puts("malformed packets rejected: OK");

    /* A builder that is too small refuses cleanly. */
    uint8_t tiny[20]; pkt_begin(&b, tiny, sizeof tiny, 1, 0);
    assert(pkt_add_gps(&b, &g) == -1 && pkt_end(&b) == PKT_HDR_LEN);
    puts("overflow refused without corrupting the packet: OK");

    /* Ground-station text line. */
    char line[200];
    int ln = rx_format_line(buf, n, -97.0f, -12.5f, line, sizeof line);
    assert(ln > 0);
    assert(strstr(line, "seq=258") && strstr(line, "fix=3 sv=9") && strstr(line, "lat=41.6600000") &&
           strstr(line, "lon=-91.6500000") && strstr(line, "alt=215.5m") && strstr(line, "batt=7400mV") && strstr(line, "rssi=-97"));
    assert(rx_format_line(bad, 5, 0, 0, line, sizeof line) < 0);
    printf("ground line: %s\n", line);

    /* The flight board's packet is GPS only: header + one GPS record. */
    pkt_begin(&b, buf2, sizeof buf2, 1, 7);
    assert(pkt_add_gps(&b, &g) == 0 && pkt_end(&b) == 24);
    puts("flight packet (GPS only) is 24 bytes: OK");

    FILE *f = fopen("vectors.txt", "w");                       /* for the Python decoder test */
    for (size_t i = 0; i < n; i++) fprintf(f, "%02X", buf[i]);
    fprintf(f, "\n"); fclose(f);
    puts("ALL PACKET TESTS PASSED");
    return 0;
}
