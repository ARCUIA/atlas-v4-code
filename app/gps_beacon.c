#include "gps_beacon.h"
#include "gps.h"
#include "radio.h"
#include "gpsmsg.h"

#define BEACON_PERIOD_MS 1000u

void gps_beacon_poll(uint32_t now_ms)
{
    static uint32_t last_sent_ms;
    static int      have_last;
    static int32_t  last_lat_e7, last_lon_e7;

    if (radio_busy()) return;
    if (last_sent_ms != 0 && (uint32_t)(now_ms - last_sent_ms) < BEACON_PERIOD_MS) return;

    const ubx_pvt_t *g = gps_last();
    int fix_ok = (g->fix_type >= 2) && (g->flags & 0x01u);
    if (fix_ok) { have_last = 1; last_lat_e7 = g->lat_e7; last_lon_e7 = g->lon_e7; }   /* remember the last good position */

    gpsmsg_in_t in = { fix_ok, have_last, last_lat_e7, last_lon_e7 };
    char msg[48];
    if (gpsmsg_format(msg, sizeof msg, &in) == 0) return;
    if (radio_send_text(msg) == 0) last_sent_ms = now_ms ? now_ms : 1u;
}
