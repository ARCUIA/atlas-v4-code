#ifndef GPSMSG_H
#define GPSMSG_H
/*
 * The text the ground receiver understands (it splits the module's "+RCV=addr,len,<payload>,rssi,snr" line on commas):
 *     YG,<latitude>,<longitude>       GPS fix, e.g. "YG,41.660123,-91.650456"
 *     NG[,<latitude>,<longitude>]     no fix; the last known position follows if there ever was one
 * That receiver reads only the first three fields (and the position after YG) and ignores anything after them,
 * so more comma-separated fields can be appended later without changing it.
 */
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int     fix_ok;                  /* 1 = we have a usable fix right now */
    int     have_last;               /* 1 = lat/lon below hold a real position */
    int32_t lat_e7, lon_e7;          /* degrees x 1e7: the current position if fix_ok, else the last good one */
} gpsmsg_in_t;

/* Returns the length written (without a terminator counted), or 0 if it does not fit. */
size_t gpsmsg_format(char *out, size_t cap, const gpsmsg_in_t *g);
#endif
