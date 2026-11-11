#ifndef GPS_BEACON_H
#define GPS_BEACON_H
#include <stdint.h>
/* Once a second, sends the GPS position over the radio as "YG,<lat>,<lon>" (fix) or "NG[,<lat>,<lon>]" (no fix). */
void gps_beacon_poll(uint32_t now_ms);
#endif
