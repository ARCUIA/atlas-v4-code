#include "rylr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void rylr_lines_init(rylr_lines_t *l) { l->n = 0; l->done = 0; }

int rylr_lines_feed(rylr_lines_t *l, uint8_t b)
{
    if (l->done) { l->n = 0; l->done = 0; }              /* the previous line has been consumed */
    if (b == '\r') return 0;
    if (b == '\n') {
        if (l->n == 0) return 0;                         /* blank line */
        l->buf[l->n] = '\0';
        l->done = 1;
        return 1;
    }
    if (l->n < RYLR_MAX_LINE - 1u) l->buf[l->n++] = (char)b;
    else l->n = 0;                                       /* too long to be anything we know: start over */
    return 0;
}

static int starts_with(const char *s, const char *p) { return strncmp(s, p, strlen(p)) == 0; }

void rylr_parse_line(const char *line, rylr_msg_t *m)
{
    memset(m, 0, sizeof *m);
    m->kind = RYLR_MSG_OTHER;
    if (strcmp(line, "+OK") == 0)    { m->kind = RYLR_MSG_OK; return; }
    if (strcmp(line, "+READY") == 0 || strcmp(line, "+RESET") == 0) { m->kind = RYLR_MSG_READY; return; }
    if (starts_with(line, "+ERR=")) { m->kind = RYLR_MSG_ERR; m->err = atoi(line + 5); return; }

    if (starts_with(line, "+RCV=")) {
        const char *p = line + 5;
        char *end;
        long addr = strtol(p, &end, 10);
        if (*end != ',') return;
        p = end + 1;
        long len = strtol(p, &end, 10);
        if (*end != ',' || len < 0 || len > (long)RYLR_MAX_DATA) return;
        const char *data = end + 1;
        if (strlen(data) < (size_t)len + 1u || data[len] != ',') return;     /* data is length-delimited: it may contain commas */
        p = data + len + 1;
        long rssi = strtol(p, &end, 10);
        if (*end != ',') return;
        long snr = strtol(end + 1, &end, 10);
        m->kind = RYLR_MSG_RCV; m->addr = (uint16_t)addr; m->len = (uint16_t)len; m->data = data;
        m->rssi = (int)rssi; m->snr = (int)snr;
        return;
    }
    if (line[0] == '+') {
        const char *eq = strchr(line, '=');
        if (eq && (size_t)(eq - line - 1) < sizeof m->key) {
            memcpy(m->key, line + 1, (size_t)(eq - line - 1));
            m->key[eq - line - 1] = '\0';
            strncpy(m->value, eq + 1, sizeof m->value - 1);
            m->kind = RYLR_MSG_VALUE;
        }
    }
}

size_t rylr_hex_encode(const uint8_t *bin, size_t n, char *out, size_t cap)
{
    static const char H[] = "0123456789ABCDEF";
    if (cap < 2 * n + 1) return 0;
    for (size_t i = 0; i < n; i++) { out[2 * i] = H[bin[i] >> 4]; out[2 * i + 1] = H[bin[i] & 15]; }
    out[2 * n] = '\0';
    return 2 * n;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

size_t rylr_hex_decode(const char *hex, size_t hexlen, uint8_t *out, size_t cap)
{
    if (hexlen == 0 || (hexlen & 1u) || hexlen / 2 > cap) return 0;
    for (size_t i = 0; i < hexlen; i += 2) {
        int a = hexval(hex[i]), b = hexval(hex[i + 1]);
        if (a < 0 || b < 0) return 0;
        out[i / 2] = (uint8_t)((a << 4) | b);
    }
    return hexlen / 2;
}

size_t rylr_build_send(char *out, size_t cap, uint16_t dest, const uint8_t *bin, size_t n)
{
    if (n == 0 || 2 * n > RYLR_MAX_DATA) return 0;
    int head = snprintf(out, cap, "AT+SEND=%u,%u,", (unsigned)dest, (unsigned)(2 * n));
    if (head < 0 || (size_t)head + 2 * n + 3 > cap) return 0;
    rylr_hex_encode(bin, n, out + head, cap - (size_t)head);
    size_t len = (size_t)head + 2 * n;
    out[len++] = '\r'; out[len++] = '\n'; out[len] = '\0';
    return len;
}

size_t rylr_build_send_text(char *out, size_t cap, uint16_t dest, const char *text, size_t n)
{
    if (n == 0 || n > RYLR_MAX_DATA) return 0;
    for (size_t i = 0; i < n; i++) if ((unsigned char)text[i] < 0x20 || (unsigned char)text[i] > 0x7E) return 0;
    int head = snprintf(out, cap, "AT+SEND=%u,%u,", (unsigned)dest, (unsigned)n);
    if (head < 0 || (size_t)head + n + 3 > cap) return 0;
    memcpy(out + head, text, n);
    size_t len = (size_t)head + n;
    out[len++] = '\r'; out[len++] = '\n'; out[len] = '\0';
    return len;
}
