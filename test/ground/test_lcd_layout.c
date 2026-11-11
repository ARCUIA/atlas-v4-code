#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "lcd_layout.h"
#include "pkt.h"

static size_t make_pkt(uint8_t *buf, uint16_t seq, uint8_t fix, int32_t lat, int32_t lon, int32_t alt_dm)
{
    pkt_builder_t b; pkt_gps_t g = { .fix_type = fix, .num_sv = 9, .lat_e7 = lat, .lon_e7 = lon, .alt_dm = alt_dm,
                                     .gspeed_dms = 123, .hacc_m = 2 };
    pkt_begin(&b, buf, 64, 1, seq);
    pkt_add_gps(&b, &g);
    return pkt_end(&b);
}
static void show(const char *title, char r[LCD_MAX_ROWS][LCD_MAX_COLS + 1], int rows)
{
    printf("%s\n", title); for (int i = 0; i < rows; i++) printf("  |%s|\n", r[i]);
}
static void check_width(char r[LCD_MAX_ROWS][LCD_MAX_COLS + 1], int rows, int cols)
{ for (int i = 0; i < rows; i++) assert((int)strlen(r[i]) == cols); }

int main(void)
{
    lcd_state_t s; lcd_state_init(&s);
    char r[LCD_MAX_ROWS][LCD_MAX_COLS + 1];
    uint8_t pkt[64];

    lcd_render(&s, 0, 16, 2, r); check_width(r, 2, 16);
    assert(strncmp(r[0], "ATLAS GROUND RX", 15) == 0 && strncmp(r[1], "waiting for data", 16) == 0);
    puts("no packet yet: waiting screen: OK");

    size_t n = make_pkt(pkt, 10, 3, 416600000, -916500000, 2155);
    assert(lcd_state_update(&s, pkt, n, -97, -12, 1000) == 0);

    lcd_render(&s, 1500, 20, 4, r); check_width(r, 4, 20);
    show("20x4 live:", r, 4);
    assert(strncmp(r[0], "LAT 41.660000", 13) == 0 && strncmp(r[1], "LON -91.650000", 14) == 0);
    assert(strncmp(r[2], "ALT 215m F3 S9", 14) == 0 && strncmp(r[3], "R-97 S-12 #10 L0", 16) == 0);

    lcd_render(&s, 1500, 16, 2, r); check_width(r, 2, 16);       /* page 0 at t = 1.5 s */
    show("16x2 page 0:", r, 2);
    assert(strncmp(r[0], "LAT 41.660000", 13) == 0 && strncmp(r[1], "LON -91.650000", 14) == 0);
    lcd_render(&s, 3000, 16, 2, r); check_width(r, 2, 16);       /* page 1 at t = 3 s */
    show("16x2 page 1:", r, 2);
    assert(strncmp(r[0], "A215m F3 S9", 11) == 0 && strncmp(r[1], "R-97 S-12 #10", 13) == 0);

    /* sequence gaps count as lost packets; a restarted flight board (seq back to 0) does not */
    n = make_pkt(pkt, 11, 3, 416600000, -916500000, 2155); lcd_state_update(&s, pkt, n, -97, -12, 2000);
    n = make_pkt(pkt, 14, 3, 416600000, -916500000, 2155); lcd_state_update(&s, pkt, n, -97, -12, 5000);
    assert(s.lost == 2 && s.rx_count == 3);
    n = make_pkt(pkt, 0, 3, 416600000, -916500000, 2155);   lcd_state_update(&s, pkt, n, -97, -12, 6000);
    assert(s.lost == 2);
    puts("lost-packet counting (gap of 2 counted, restart ignored): OK");

    /* no packet for a while: NO SIGNAL, last known position stays visible */
    lcd_render(&s, 6000 + 7000, 20, 4, r); show("20x4 stale:", r, 4);
    assert(strncmp(r[0], "NO SIGNAL 7s ago", 16) == 0 && strncmp(r[1], "LAT 41.660000", 13) == 0);
    lcd_render(&s, 17500, 16, 2, r); show("16x2 stale, page 1 (17.5 s = odd page):", r, 2);
    assert(strncmp(r[1], "NO SIG 11s", 10) == 0);

    /* southern / western hemisphere and rounding */
    n = make_pkt(pkt, 1, 0, -337000005, 1518000000, -50);   lcd_state_update(&s, pkt, n, -120, -18, 20000);
    lcd_render(&s, 20500, 20, 4, r); show("20x4 no fix, negative altitude:", r, 4);
    assert(strncmp(r[0], "LAT -33.700001", 14) == 0 && strncmp(r[1], "LON 151.800000", 14) == 0);
    assert(strncmp(r[2], "ALT -5m F0 S9", 13) == 0);

    /* backlight colour tracks the link state */
    lcd_state_t c; lcd_state_init(&c);
    lcd_rgb_t k = lcd_status_rgb(&c, 0);                         assert(k.b == 255 && k.r == 0 && k.g == 0);          /* waiting: blue */
    n = make_pkt(pkt, 1, 3, 416600000, -916500000, 2155); lcd_state_update(&c, pkt, n, -90, -5, 1000);
    k = lcd_status_rgb(&c, 2000);                                assert(k.g == 255 && k.r == 0 && k.b == 0);          /* 3D fix: green */
    k = lcd_status_rgb(&c, 1000 + LCD_STALE_MS + 1);             assert(k.r == 255 && k.g == 0 && k.b == 0);          /* stale: red */
    n = make_pkt(pkt, 2, 0, 416600000, -916500000, 2155); lcd_state_update(&c, pkt, n, -90, -5, 9000);
    k = lcd_status_rgb(&c, 9500);                                assert(k.r == 255 && k.g == 120 && k.b == 0);        /* no fix: orange */
    puts("backlight colours (blue / green / orange / red): OK");

    /* garbage is rejected */
    uint8_t bad[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    assert(lcd_state_update(&s, bad, sizeof bad, 0, 0, 0) < 0);
    puts("ALL LCD LAYOUT TESTS PASSED");
    return 0;
}
