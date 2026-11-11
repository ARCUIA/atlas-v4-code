#include "lcd_layout.h"
#include "pkt.h"
#include <stdio.h>
#include <string.h>

void lcd_state_init(lcd_state_t *s) { memset(s, 0, sizeof *s); }

int lcd_state_update(lcd_state_t *s, const uint8_t *pkt, size_t len, int rssi, int snr, uint32_t now_ms)
{
    pkt_header_t h;
    if (pkt_parse_header(pkt, len, &h) != 0) return -1;

    if (s->have_pkt) {
        uint16_t gap = (uint16_t)(h.seq - s->seq - 1u);          /* packets skipped since the last good one */
        if (gap > 0 && gap < 100u) s->lost += gap;               /* a huge jump means the flight board restarted */
    }
    s->have_pkt = 1; s->last_rx_ms = now_ms; s->seq = h.seq; s->rssi = rssi; s->snr = snr; s->rx_count++;

    pkt_iter_t it; uint8_t type, rlen; const uint8_t *d;
    pkt_iter_init(&it, pkt, len);
    while (pkt_iter_next(&it, &type, &d, &rlen) == 1) {
        pkt_gps_t g;
        if (type == PKT_REC_GPS && pkt_decode_gps(d, rlen, &g) == 0) {
            s->fix = g.fix_type; s->sv = g.num_sv; s->lat_e7 = g.lat_e7; s->lon_e7 = g.lon_e7;
            s->alt_dm = g.alt_dm; s->gspeed_dms = g.gspeed_dms; s->hacc_m = g.hacc_m;
        }
    }
    return 0;
}

/* 416600000 -> "41.660000", -916500000 -> "-91.650000" (integer maths, rounded to 1e-6 degrees). */
static void fmt_deg(char *out, size_t cap, int32_t e7)
{
    uint32_t a = e7 < 0 ? (uint32_t)(-(int64_t)e7) : (uint32_t)e7;
    uint32_t micro = (a + 5u) / 10u;
    snprintf(out, cap, "%s%lu.%06lu", e7 < 0 ? "-" : "", (unsigned long)(micro / 1000000u), (unsigned long)(micro % 1000000u));
}

static void put_row(char out[LCD_MAX_ROWS][LCD_MAX_COLS + 1], uint8_t row, uint8_t cols, const char *text)
{
    size_t n = strlen(text);
    if (n > cols) n = cols;
    memset(out[row], ' ', cols);
    memcpy(out[row], text, n);
    out[row][cols] = '\0';
}

void lcd_render(const lcd_state_t *s, uint32_t now_ms, uint8_t cols, uint8_t rows, char out[LCD_MAX_ROWS][LCD_MAX_COLS + 1])
{
    char t[40], a[16], o[16];
    if (cols > LCD_MAX_COLS) cols = LCD_MAX_COLS;
    if (rows > LCD_MAX_ROWS) rows = LCD_MAX_ROWS;
    for (uint8_t r = 0; r < rows; r++) put_row(out, r, cols, "");

    if (!s->have_pkt) {
        put_row(out, 0, cols, "ATLAS GROUND RX");
        if (rows > 1) put_row(out, 1, cols, "waiting for data");
        return;
    }
    uint32_t age_s = (now_ms - s->last_rx_ms) / 1000u;
    int stale = (uint32_t)(now_ms - s->last_rx_ms) > LCD_STALE_MS;
    fmt_deg(a, sizeof a, s->lat_e7);
    fmt_deg(o, sizeof o, s->lon_e7);

    if (rows >= 4) {                                              /* one page, everything visible */
        char lat[24], lon[24];
        snprintf(lat, sizeof lat, "LAT %s", a);
        snprintf(lon, sizeof lon, "LON %s", o);
        if (stale) {
            snprintf(t, sizeof t, "NO SIGNAL %lus ago", (unsigned long)age_s); put_row(out, 0, cols, t);
            put_row(out, 1, cols, lat); put_row(out, 2, cols, lon);
            snprintf(t, sizeof t, "ALT %ldm F%u", (long)(s->alt_dm / 10), s->fix); put_row(out, 3, cols, t);
        } else {
            put_row(out, 0, cols, lat); put_row(out, 1, cols, lon);
            snprintf(t, sizeof t, "ALT %ldm F%u S%u", (long)(s->alt_dm / 10), s->fix, s->sv); put_row(out, 2, cols, t);
            snprintf(t, sizeof t, "R%d S%d #%u L%lu", s->rssi, s->snr, s->seq, (unsigned long)s->lost); put_row(out, 3, cols, t);
        }
        return;
    }

    /* 2 rows: two pages that alternate */
    if (((now_ms / LCD_PAGE_MS) & 1u) == 0) {
        snprintf(t, sizeof t, "LAT %s", a); put_row(out, 0, cols, t);
        if (rows > 1) { snprintf(t, sizeof t, "LON %s", o); put_row(out, 1, cols, t); }
    } else {
        snprintf(t, sizeof t, "A%ldm F%u S%u", (long)(s->alt_dm / 10), s->fix, s->sv); put_row(out, 0, cols, t);
        if (rows > 1) {
            if (stale) snprintf(t, sizeof t, "NO SIG %lus", (unsigned long)age_s);
            else       snprintf(t, sizeof t, "R%d S%d #%u", s->rssi, s->snr, s->seq);
            put_row(out, 1, cols, t);
        }
    }
}

lcd_rgb_t lcd_status_rgb(const lcd_state_t *s, uint32_t now_ms)
{
    lcd_rgb_t c;
    if (!s->have_pkt)                                      { c.r = 0;   c.g = 0;   c.b = 255; }
    else if ((uint32_t)(now_ms - s->last_rx_ms) > LCD_STALE_MS) { c.r = 255; c.g = 0;   c.b = 0;   }
    else if (s->fix >= 3)                                  { c.r = 0;   c.g = 255; c.b = 0;   }
    else                                                   { c.r = 255; c.g = 120; c.b = 0;   }
    return c;
}
