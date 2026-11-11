#include "pkt.h"
#include <string.h>

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

void pkt_begin(pkt_builder_t *b, uint8_t *buf, size_t cap, uint8_t src_id, uint16_t seq)
{
    b->buf = buf; b->cap = cap; b->len = 0;
    if (cap < PKT_HDR_LEN) return;
    buf[0] = PKT_MAGIC;
    buf[1] = (uint8_t)(PKT_VERSION << 4);          /* flags (low nibble) = 0 */
    buf[2] = src_id;
    put16(buf + 3, seq);
    b->len = PKT_HDR_LEN;
}

int pkt_add(pkt_builder_t *b, uint8_t type, const uint8_t *data, uint8_t len)
{
    if (b->len < PKT_HDR_LEN || b->len + 2u + len > b->cap) return -1;
    b->buf[b->len++] = type;
    b->buf[b->len++] = len;
    if (len) memcpy(b->buf + b->len, data, len);
    b->len += len;
    return 0;
}

size_t pkt_end(const pkt_builder_t *b) { return b->len; }

int pkt_add_gps(pkt_builder_t *b, const pkt_gps_t *g)
{
    uint8_t d[17];
    d[0] = g->fix_type; d[1] = g->num_sv;
    put32(d + 2, (uint32_t)g->lat_e7); put32(d + 6, (uint32_t)g->lon_e7); put32(d + 10, (uint32_t)g->alt_dm);
    put16(d + 14, g->gspeed_dms);
    d[16] = g->hacc_m;
    return pkt_add(b, PKT_REC_GPS, d, sizeof d);
}

int pkt_add_batt(pkt_builder_t *b, uint16_t mv)
{
    uint8_t d[2]; put16(d, mv);
    return pkt_add(b, PKT_REC_BATT, d, sizeof d);
}

int pkt_parse_header(const uint8_t *p, size_t n, pkt_header_t *h)
{
    if (n < PKT_HDR_LEN || p[0] != PKT_MAGIC) return -1;
    h->version = p[1] >> 4; h->flags = p[1] & 0x0F; h->src_id = p[2]; h->seq = get16(p + 3);
    return (h->version == PKT_VERSION) ? 0 : -2;
}

void pkt_iter_init(pkt_iter_t *it, const uint8_t *p, size_t n) { it->p = p; it->n = n; it->pos = PKT_HDR_LEN; }

int pkt_iter_next(pkt_iter_t *it, uint8_t *type, const uint8_t **data, uint8_t *len)
{
    if (it->pos >= it->n) return 0;
    if (it->pos + 2 > it->n) return -1;
    uint8_t t = it->p[it->pos], l = it->p[it->pos + 1];
    if (it->pos + 2u + l > it->n) return -1;
    *type = t; *len = l; *data = it->p + it->pos + 2;
    it->pos += 2u + l;
    return 1;
}

int pkt_decode_gps(const uint8_t *d, uint8_t len, pkt_gps_t *g)
{
    if (len < 17) return -1;
    g->fix_type = d[0]; g->num_sv = d[1];
    g->lat_e7 = (int32_t)get32(d + 2); g->lon_e7 = (int32_t)get32(d + 6); g->alt_dm = (int32_t)get32(d + 10);
    g->gspeed_dms = get16(d + 14); g->hacc_m = d[16];
    return 0;
}

int pkt_decode_batt(const uint8_t *d, uint8_t len, uint16_t *mv)
{
    if (len < 2) return -1;
    *mv = get16(d);
    return 0;
}
