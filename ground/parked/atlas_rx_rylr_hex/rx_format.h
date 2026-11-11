#ifndef RX_FORMAT_H
#define RX_FORMAT_H
/* Turns a received packet into one readable line. Used by the ground station and tested on the PC. */
#include <stddef.h>
#include <stdint.h>
/* Returns the length written, or a negative value if the packet is not an Atlas packet. */
int rx_format_line(const uint8_t *pkt, size_t len, float rssi_dbm, float snr_db, char *out, size_t cap);
#endif
