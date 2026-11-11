#ifndef PKT_H
#define PKT_H
/*
 * Telemetry packet format, shared by the flight board and the ground station (plain C99, no HAL).
 *
 *   header (5 bytes):  magic 0xA7 | version<<4 | flags | source id | sequence (u16 LE)
 *   then records until the end of the payload (its length comes from the radio):
 *                      type (u8) | length (u8) | data[length]
 *
 * A receiver skips records whose type it does not know, so new record types can be added later without
 * breaking older ground software. All multi-byte fields are little-endian.
 *
 *   type 1  GPS      17 bytes: fix_type u8, num_sv u8, lat_e7 i32, lon_e7 i32, alt_dm i32 (MSL, decimetres),
 *                              gspeed_dms u16 (0.1 m/s), hacc_m u8 (clamped to 255)
 *   type 2  BATT      2 bytes: millivolts u16 (defined, but the flight board does not send it at the moment)
 *   types 3..       reserved (flight state, barometer altitude, attitude, pyro status, ...)
 */
#include <stdint.h>
#include <stddef.h>

#define PKT_MAGIC     0xA7u
#define PKT_VERSION   1u
#define PKT_HDR_LEN   5u

enum { PKT_REC_GPS = 1, PKT_REC_BATT = 2 };

typedef struct { uint8_t *buf; size_t cap, len; } pkt_builder_t;

typedef struct {
    uint8_t  fix_type;      /* 0 = no fix (position fields then hold the last known good fix) */
    uint8_t  num_sv;
    int32_t  lat_e7, lon_e7, alt_dm;
    uint16_t gspeed_dms;
    uint8_t  hacc_m;
} pkt_gps_t;

typedef struct { uint8_t version, flags, src_id; uint16_t seq; } pkt_header_t;
typedef struct { const uint8_t *p; size_t n, pos; } pkt_iter_t;

/* ---- building ---- */
void   pkt_begin(pkt_builder_t *b, uint8_t *buf, size_t cap, uint8_t src_id, uint16_t seq);
int    pkt_add(pkt_builder_t *b, uint8_t type, const uint8_t *data, uint8_t len);   /* 0 ok, -1 no room */
size_t pkt_end(const pkt_builder_t *b);                                              /* total length so far */
int    pkt_add_gps(pkt_builder_t *b, const pkt_gps_t *g);
int    pkt_add_batt(pkt_builder_t *b, uint16_t millivolts);

/* ---- parsing ---- */
int  pkt_parse_header(const uint8_t *p, size_t n, pkt_header_t *h);                 /* 0 ok, -1 too short/bad magic, -2 unknown version */
void pkt_iter_init(pkt_iter_t *it, const uint8_t *p, size_t n);                     /* p/n = the whole packet */
int  pkt_iter_next(pkt_iter_t *it, uint8_t *type, const uint8_t **data, uint8_t *len);   /* 1 record, 0 end, -1 malformed */
int  pkt_decode_gps(const uint8_t *data, uint8_t len, pkt_gps_t *g);                /* 0 ok */
int  pkt_decode_batt(const uint8_t *data, uint8_t len, uint16_t *millivolts);       /* 0 ok */

#endif
