#ifndef RYLR_H
#define RYLR_H
/*
 * REYAX RYLR998 text protocol helpers (plain C, no hardware). See REYAX "LoRa AT COMMAND GUIDE".
 *   commands:  AT+SEND=<addr>,<length>,<data>\r\n   (data is ASCII, at most 240 bytes)
 *   replies:   +OK   +ERR=<n>   +READY   +<KEY>=<value>   +RCV=<addr>,<length>,<data>,<rssi>,<snr>
 * Binary packets travel as hexadecimal text, so any byte value is safe.
 */
#include <stddef.h>
#include <stdint.h>

#define RYLR_MAX_LINE   300u
#define RYLR_MAX_DATA   240u     /* the module's limit for AT+SEND */

typedef struct { char buf[RYLR_MAX_LINE]; uint16_t n; uint8_t done; } rylr_lines_t;

typedef enum { RYLR_MSG_OTHER = 0, RYLR_MSG_OK, RYLR_MSG_ERR, RYLR_MSG_READY, RYLR_MSG_VALUE, RYLR_MSG_RCV } rylr_kind_t;

typedef struct {
    rylr_kind_t kind;
    int         err;                  /* +ERR=n */
    char        key[16];              /* +KEY=value */
    char        value[64];
    uint16_t    addr, len;            /* +RCV */
    const char *data;                 /* +RCV: points into the line, NOT NUL-terminated at len */
    int         rssi, snr;
} rylr_msg_t;

void rylr_lines_init(rylr_lines_t *l);
/* Feed one byte. Returns 1 when a complete line is in l->buf (NUL-terminated, no CR/LF). */
int  rylr_lines_feed(rylr_lines_t *l, uint8_t b);
void rylr_parse_line(const char *line, rylr_msg_t *m);

size_t rylr_hex_encode(const uint8_t *bin, size_t n, char *out, size_t cap);        /* returns characters written (no NUL counted) */
size_t rylr_hex_decode(const char *hex, size_t hexlen, uint8_t *out, size_t cap);   /* returns bytes, 0 on any bad character or odd length */

/* "AT+SEND=<dest>,<2n>,<HEX>\r\n". Returns the length, or 0 if the data does not fit the 240-byte limit. */
size_t rylr_build_send(char *out, size_t cap, uint16_t dest, const uint8_t *bin, size_t n);

/* "AT+SEND=<dest>,<n>,<text>\r\n" for a plain-text payload. Returns the length, or 0 if the text is empty, longer than 240
 * bytes, or contains anything but printable ASCII (the module's data field is ASCII, and CR/LF would end the command). */
size_t rylr_build_send_text(char *out, size_t cap, uint16_t dest, const char *text, size_t n);

#endif
