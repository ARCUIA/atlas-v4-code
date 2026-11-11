#ifndef LCD_LAYOUT_H
#define LCD_LAYOUT_H
/*
 * Ground station display logic (plain C, tested on the PC): keeps the latest received state and turns it
 * into rows of text for a character LCD. Works for 16x2 (alternates two pages) and for 20x4 (one page).
 */
#include <stddef.h>
#include <stdint.h>

#define LCD_MAX_COLS 20
#define LCD_MAX_ROWS 4
#define LCD_STALE_MS 5000u        /* no packet for this long = "NO SIGNAL" */
#define LCD_PAGE_MS  2500u        /* 16x2 only: how long each page is shown */

typedef struct {
    int      have_pkt;
    uint32_t last_rx_ms;
    uint16_t seq;
    uint8_t  fix, sv;
    int32_t  lat_e7, lon_e7, alt_dm;
    uint16_t gspeed_dms;
    uint8_t  hacc_m;
    int      rssi, snr;
    uint32_t rx_count, lost;      /* good packets, and packets missed according to the sequence numbers */
} lcd_state_t;

typedef struct { uint8_t r, g, b; } lcd_rgb_t;

void lcd_state_init(lcd_state_t *s);
/* Feeds one received packet. Returns 0 if it was an Atlas packet, negative otherwise. */
int  lcd_state_update(lcd_state_t *s, const uint8_t *pkt, size_t len, int rssi, int snr, uint32_t now_ms);
/* Fills out[row] with exactly `cols` characters (space padded, NUL terminated) for each of `rows` rows. */
void lcd_render(const lcd_state_t *s, uint32_t now_ms, uint8_t cols, uint8_t rows, char out[LCD_MAX_ROWS][LCD_MAX_COLS + 1]);

/* Backlight colour that shows the link state at a glance: blue = waiting, green = live with a 3D fix,
 * orange = live without a fix, red = no packet for a while. */
lcd_rgb_t lcd_status_rgb(const lcd_state_t *s, uint32_t now_ms);

#endif
