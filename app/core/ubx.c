#include "ubx.h"
#include <string.h>

enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB };

static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

void ubx_parser_init(ubx_parser_t *p) { memset(p, 0, sizeof *p); p->state = S_SYNC1; }

static void ck(ubx_parser_t *p, uint8_t b) { p->ck_a = (uint8_t)(p->ck_a + b); p->ck_b = (uint8_t)(p->ck_b + p->ck_a); }

int ubx_parser_feed(ubx_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_SYNC1: if (b == 0xB5) p->state = S_SYNC2; break;
    case S_SYNC2: p->state = (b == 0x62) ? S_CLASS : (b == 0xB5 ? S_SYNC2 : S_SYNC1); break;
    case S_CLASS: p->cls = b; p->ck_a = p->ck_b = 0; ck(p, b); p->state = S_ID; break;
    case S_ID:    p->id = b; ck(p, b); p->state = S_LEN1; break;
    case S_LEN1:  p->len = b; ck(p, b); p->state = S_LEN2; break;
    case S_LEN2:
        p->len = (uint16_t)(p->len | (b << 8)); ck(p, b); p->pos = 0;
        if (p->len > UBX_MAX_PAYLOAD) { p->too_long++; p->state = S_SYNC1; }
        else p->state = p->len ? S_PAYLOAD : S_CKA;
        break;
    case S_PAYLOAD:
        p->payload[p->pos++] = b; ck(p, b);
        if (p->pos >= p->len) p->state = S_CKA;
        break;
    case S_CKA:
        if (b == p->ck_a) p->state = S_CKB; else { p->frames_bad++; p->state = S_SYNC1; }
        break;
    case S_CKB:
        p->state = S_SYNC1;
        if (b == p->ck_b) { p->frames_ok++; return 1; }
        p->frames_bad++;
        break;
    default: p->state = S_SYNC1;
    }
    return 0;
}

size_t ubx_build(uint8_t *out, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t a = 0, b = 0;
    out[0] = 0xB5; out[1] = 0x62; out[2] = cls; out[3] = id;
    out[4] = (uint8_t)len; out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(out + 6, payload, len);
    for (size_t i = 2; i < (size_t)6 + len; i++) { a = (uint8_t)(a + out[i]); b = (uint8_t)(b + a); }
    out[6 + len] = a; out[7 + len] = b;
    return (size_t)len + 8;
}

void ubx_valset_begin(ubx_valset_t *v, uint8_t layers)
{
    memset(v, 0, sizeof *v);
    v->buf[0] = 0;          /* message version */
    v->buf[1] = layers;
    v->len = 4;             /* version, layers, 2 reserved bytes */
}

void ubx_valset_add(ubx_valset_t *v, uint32_t key, uint32_t value)
{
    static const uint8_t size_of_code[8] = { 0, 1, 1, 2, 4, 8, 0, 0 };   /* key bits 30:28: 1 bit and 1 byte both use 1 byte */
    uint8_t n = size_of_code[(key >> 28) & 7u];
    if (n == 0 || n > 4 || v->len + 4u + n > UBX_MAX_PAYLOAD) { v->overflow = 1; return; }
    for (int i = 0; i < 4; i++) v->buf[v->len++] = (uint8_t)(key >> (8 * i));
    for (int i = 0; i < n; i++) v->buf[v->len++] = (uint8_t)(value >> (8 * i));
}

int ubx_pvt_decode(const uint8_t *d, uint16_t len, ubx_pvt_t *o)
{
    if (len != UBX_NAV_PVT_LEN) return -1;
    o->itow_ms = u32(d + 0);
    o->year = u16(d + 4); o->month = d[6]; o->day = d[7]; o->hour = d[8]; o->min = d[9]; o->sec = d[10]; o->valid = d[11];
    o->fix_type = d[20]; o->flags = d[21]; o->num_sv = d[23];
    o->lon_e7 = (int32_t)u32(d + 24); o->lat_e7 = (int32_t)u32(d + 28);
    o->height_mm = (int32_t)u32(d + 32); o->hmsl_mm = (int32_t)u32(d + 36);
    o->hacc_mm = u32(d + 40); o->vacc_mm = u32(d + 44);
    o->vel_n_mms = (int32_t)u32(d + 48); o->vel_e_mms = (int32_t)u32(d + 52); o->vel_d_mms = (int32_t)u32(d + 56);
    o->gspeed_mms = (int32_t)u32(d + 60);
    o->pdop_x100 = u16(d + 76);
    return 0;
}
