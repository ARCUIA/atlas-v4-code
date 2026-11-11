#include "gpsmsg.h"
#include <stdio.h>

/* 416600123 -> "41.660012", -916500000 -> "-91.650000": integer maths, rounded to 1e-6 degrees (about 0.1 m). */
static int fmt_deg(char *out, size_t cap, int32_t e7)
{
    uint32_t a = e7 < 0 ? (uint32_t)(-(int64_t)e7) : (uint32_t)e7;
    uint32_t micro = (a + 5u) / 10u;
    return snprintf(out, cap, "%s%lu.%06lu", e7 < 0 ? "-" : "", (unsigned long)(micro / 1000000u), (unsigned long)(micro % 1000000u));
}

size_t gpsmsg_format(char *out, size_t cap, const gpsmsg_in_t *g)
{
    char lat[16], lon[16];
    int n;
    if (!g->fix_ok && !g->have_last) n = snprintf(out, cap, "NG");
    else {
        fmt_deg(lat, sizeof lat, g->lat_e7);
        fmt_deg(lon, sizeof lon, g->lon_e7);
        n = snprintf(out, cap, "%s,%s,%s", g->fix_ok ? "YG" : "NG", lat, lon);
    }
    return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}
