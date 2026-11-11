#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "telemetry.h"

static int fill_gps(pkt_builder_t *b, void *c) { (void)c; pkt_gps_t g; memset(&g, 0, sizeof g); g.fix_type = 3; return pkt_add_gps(b, &g); }
static int fill_batt(pkt_builder_t *b, void *c) { (void)c; return pkt_add_batt(b, 7400); }
static int fill_slow(pkt_builder_t *b, void *c) { (void)c; uint8_t d[4] = {1,2,3,4}; return pkt_add(b, 9, d, 4); }

static int has(const uint8_t *p, size_t n, uint8_t want)
{
    pkt_iter_t it; uint8_t t, l; const uint8_t *d; pkt_iter_init(&it, p, n);
    while (pkt_iter_next(&it, &t, &d, &l) == 1) if (t == want) return 1;
    return 0;
}

int main(void)
{
    uint8_t pkt[64]; tlm_t t;

    /* Two 1 Hz sources and one slow (2.5 s) source: the slow one rides along only when due. */
    tlm_init(&t, 1, 1000);
    tlm_add(&t, fill_gps, 0, 1000); tlm_add(&t, fill_batt, 0, 1000); tlm_add(&t, fill_slow, 0, 2500);
    int sent = 0, slow_at[8], nslow = 0, gps_every = 1; uint32_t last = 0;
    for (uint32_t now = 0; now <= 7000; now += 100) {
        size_t n = tlm_build(&t, now, pkt, sizeof pkt);
        if (!n) continue;
        sent++;
        if (!has(pkt, n, PKT_REC_GPS) || !has(pkt, n, PKT_REC_BATT)) gps_every = 0;
        if (has(pkt, n, 9)) slow_at[nslow++] = (int)now;
        if (sent > 1) assert(now - last >= 1000);                 /* never faster than the minimum gap */
        last = now;
    }
    assert(gps_every && sent == 8);                              /* a packet each second from t = 0 */
    assert(nslow == 3 && slow_at[0] == 0 && slow_at[1] == 3000 && slow_at[2] == 6000);
    printf("schedule: %d packets in 7 s, slow source at t = %d, %d, %d ms: OK\n", sent, slow_at[0], slow_at[1], slow_at[2]);

    /* Called too soon: nothing is built, and the sequence number does not advance. */
    tlm_init(&t, 1, 1000); tlm_add(&t, fill_gps, 0, 1000);
    assert(tlm_build(&t, 0, pkt, sizeof pkt) > 0 && t.seq == 1);
    assert(tlm_build(&t, 500, pkt, sizeof pkt) == 0 && t.seq == 1);
    assert(tlm_build(&t, 1000, pkt, sizeof pkt) > 0 && t.seq == 2);
    puts("minimum gap respected, sequence counts packets: OK");

    /* A packet too small for everything: nothing starves, both sources get their turn. */
    tlm_init(&t, 1, 1000); tlm_add(&t, fill_gps, 0, 1000); tlm_add(&t, fill_batt, 0, 1000);
    int gps_n = 0, batt_n = 0;
    for (uint32_t now = 0; now < 6000; now += 1000) {
        size_t n = tlm_build(&t, now, pkt, 24);                  /* header + GPS = 24 bytes: no room for both */
        assert(n > 0);
        gps_n += has(pkt, n, PKT_REC_GPS); batt_n += has(pkt, n, PKT_REC_BATT);
    }
    assert(gps_n >= 2 && batt_n >= 2);
    printf("small packets, 6 sends: GPS in %d, battery in %d (no starvation): OK\n", gps_n, batt_n);

    puts("ALL TELEMETRY TESTS PASSED");
    return 0;
}
