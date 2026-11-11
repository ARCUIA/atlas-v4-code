#include "ground_rx.h"
#include "rylr.h"
#include "rx_format.h"
#include <stdio.h>

int ground_rx_handle(lcd_state_t *s, const char *hex, uint16_t hexlen, int rssi, int snr, uint32_t now_ms, char *line, size_t cap)
{
    uint8_t bin[RYLR_MAX_DATA / 2];
    size_t n = rylr_hex_decode(hex, hexlen, bin, sizeof bin);
    if (n == 0) return GROUND_RX_NOT_HEX;
    if (lcd_state_update(s, bin, n, rssi, snr, now_ms) != 0) return GROUND_RX_NOT_ATLAS;
    int w = rx_format_line(bin, n, (float)rssi, (float)snr, line, cap);
    if (w > 0 && (size_t)w < cap)
        snprintf(line + w, cap - (size_t)w, " rx=%lu lost=%lu", (unsigned long)s->rx_count, (unsigned long)s->lost);
    return GROUND_RX_OK;
}
