#include "rx_format.h"
#include "pkt.h"
#include <stdio.h>

int rx_format_line(const uint8_t *pkt, size_t len, float rssi_dbm, float snr_db, char *out, size_t cap)
{
    pkt_header_t h;
    if (pkt_parse_header(pkt, len, &h) != 0) return -1;

    pkt_gps_t g; int have_gps = 0;
    uint16_t mv = 0; int have_batt = 0;
    pkt_iter_t it; uint8_t type, rlen; const uint8_t *d;
    pkt_iter_init(&it, pkt, len);
    while (pkt_iter_next(&it, &type, &d, &rlen) == 1) {
        if (type == PKT_REC_GPS  && pkt_decode_gps(d, rlen, &g) == 0) have_gps = 1;
        if (type == PKT_REC_BATT && pkt_decode_batt(d, rlen, &mv) == 0) have_batt = 1;
        /* unknown record types are skipped: newer flight software stays readable */
    }

    int n = snprintf(out, cap, "seq=%u", (unsigned)h.seq);
    if (have_gps && n > 0 && (size_t)n < cap) {
        long lat = g.lat_e7, lon = g.lon_e7;
        n += snprintf(out + n, cap - (size_t)n, " fix=%u sv=%u lat=%s%ld.%07ld lon=%s%ld.%07ld alt=%ld.%ldm spd=%u.%um/s hacc=%um",
                      g.fix_type, g.num_sv,
                      lat < 0 ? "-" : "", (lat < 0 ? -lat : lat) / 10000000L, (lat < 0 ? -lat : lat) % 10000000L,
                      lon < 0 ? "-" : "", (lon < 0 ? -lon : lon) / 10000000L, (lon < 0 ? -lon : lon) % 10000000L,
                      (long)(g.alt_dm / 10), (long)((g.alt_dm < 0 ? -g.alt_dm : g.alt_dm) % 10),
                      g.gspeed_dms / 10u, g.gspeed_dms % 10u, g.hacc_m);
    }
    if (have_batt && n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - (size_t)n, " batt=%umV", mv);
    if (n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - (size_t)n, " rssi=%d snr=%d", (int)rssi_dbm, (int)snr_db);
    return n;
}
