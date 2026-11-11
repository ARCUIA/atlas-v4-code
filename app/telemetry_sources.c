#include "telemetry_sources.h"
#include "gps.h"

#define TLM_GPS_PERIOD_MS   1000u

/* GPS: sends the last GOOD position even when the fix is lost, with fix_type 0, so the ground station
 * can still show where the rocket was last seen. */
static int gps_fill(pkt_builder_t *b, void *ctx)
{
    static pkt_gps_t last_good;
    const ubx_pvt_t *g = gps_last();
    (void)ctx;

    int ok = (g->fix_type >= 2) && (g->flags & 0x01u);
    if (ok) {
        last_good.num_sv = g->num_sv;
        last_good.lat_e7 = g->lat_e7;
        last_good.lon_e7 = g->lon_e7;
        last_good.alt_dm = g->hmsl_mm / 100;
        last_good.gspeed_dms = (uint16_t)((g->gspeed_mms < 0 ? 0 : g->gspeed_mms / 100 > 65535 ? 65535 : g->gspeed_mms / 100));
        uint32_t hacc_m = g->hacc_mm / 1000u;
        last_good.hacc_m = (uint8_t)(hacc_m > 255u ? 255u : hacc_m);
    }
    pkt_gps_t out = last_good;
    out.fix_type = ok ? g->fix_type : 0;
    if (!ok) out.num_sv = g->num_sv;
    return pkt_add_gps(b, &out);
}

void telemetry_sources_init(tlm_t *t)
{
    tlm_add(t, gps_fill,  0, TLM_GPS_PERIOD_MS);
    /* Add more telemetry here, e.g. tlm_add(t, my_fill, 0, period_ms); the packet format already
     * reserves record types for it (see app/core/pkt.h). */
}
