#ifndef GROUND_RX_H
#define GROUND_RX_H
/* Ground station: turns one received hex string into display state and a readable serial line (plain C, tested on the PC). */
#include <stddef.h>
#include <stdint.h>
#include "lcd_layout.h"

#define GROUND_RX_OK          0
#define GROUND_RX_NOT_HEX    -1      /* the text was not valid hexadecimal */
#define GROUND_RX_NOT_ATLAS  -2      /* valid hex, but not one of our packets (another device on the same network) */

int ground_rx_handle(lcd_state_t *s, const char *hex, uint16_t hexlen, int rssi, int snr, uint32_t now_ms, char *line, size_t cap);
#endif
