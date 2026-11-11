#ifndef UBX_H
#define UBX_H
/*
 * u-blox UBX protocol helpers. HAL-free (runs on the PC too).
 * Frame: B5 62 | class | id | length (u16 LE) | payload | CK_A CK_B   (Fletcher checksum over class..payload)
 */
#include <stdint.h>
#include <stddef.h>

#define UBX_MAX_PAYLOAD   128u
#define UBX_NAV_PVT_LEN   92u

#define UBX_CLS_NAV  0x01
#define UBX_ID_PVT   0x07
#define UBX_CLS_ACK  0x05
#define UBX_ID_NAK   0x00
#define UBX_ID_ACK   0x01
#define UBX_CLS_CFG  0x06
#define UBX_ID_VALSET 0x8A

typedef struct {
    uint8_t  state, cls, id, ck_a, ck_b;
    uint16_t len, pos;
    uint8_t  payload[UBX_MAX_PAYLOAD];
    uint32_t frames_ok, frames_bad, too_long;
} ubx_parser_t;

void ubx_parser_init(ubx_parser_t *p);
/* Feed one byte. Returns 1 when a complete, checksum-valid frame is available in p->cls/id/len/payload
 * (valid until the next call). Anything that is not UBX (such as NMEA text) is skipped. */
int  ubx_parser_feed(ubx_parser_t *p, uint8_t byte);

/* Builds a full frame into out (needs len + 8 bytes). Returns the frame length. */
size_t ubx_build(uint8_t *out, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

/* CFG-VALSET payload builder. layers: 1 = RAM, 2 = battery-backed RAM, 4 = flash. */
typedef struct { uint8_t buf[UBX_MAX_PAYLOAD]; uint16_t len; int overflow; } ubx_valset_t;
void ubx_valset_begin(ubx_valset_t *v, uint8_t layers);
/* Appends one key/value. The value size comes from the key itself (bits 30:28). */
void ubx_valset_add(ubx_valset_t *v, uint32_t key, uint32_t value);

/* Decoded UBX-NAV-PVT (92-byte payload). */
typedef struct {
    uint32_t itow_ms;
    uint16_t year; uint8_t month, day, hour, min, sec, valid;
    uint8_t  fix_type, flags, num_sv;
    int32_t  lon_e7, lat_e7, height_mm, hmsl_mm;
    uint32_t hacc_mm, vacc_mm;
    int32_t  vel_n_mms, vel_e_mms, vel_d_mms, gspeed_mms;
    uint16_t pdop_x100;
} ubx_pvt_t;
int ubx_pvt_decode(const uint8_t *payload, uint16_t len, ubx_pvt_t *out);   /* 0 ok */

#endif
