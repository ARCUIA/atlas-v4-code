#ifndef RYLR_LINK_H
#define RYLR_LINK_H
/*
 * Drives a RYLR998 over a serial port: configures it, sends packets, delivers received ones.
 * Plain C: all hardware goes through rylr_io_t, so the same code runs on the flight board, on the ESP32
 * ground station, and in the PC tests against a simulated module.
 *
 * Configuration is a state machine, so it never needs to block. Settings that are already correct are not
 * rewritten (the module keeps address and network id in flash, which has limited write cycles).
 * If the module reboots (it prints +READY), the settings are applied again automatically.
 */
#include "rylr.h"

typedef struct {
    int      (*tx_start)(const uint8_t *p, uint16_t n);   /* start sending; p must stay valid until tx_idle(); 0 = ok */
    int      (*tx_idle)(void);                            /* 1 when the last send has finished */
    int      (*read_byte)(uint8_t *b);                    /* non-blocking: 1 if a byte was returned */
    uint32_t (*now_ms)(void);
    void     (*delay_ms)(uint32_t ms);
} rylr_io_t;

#define RYLR_KEEP 0xFFFFu    /* as own_addr: leave the module's address as it is (nothing is written) */

typedef struct {
    uint16_t own_addr, dest_addr, network_id;
    uint32_t band_hz;
    uint8_t  sf, bw_code, cr_code, preamble;              /* AT+PARAMETER values: bandwidth 7/8/9 = 125/250/500 kHz, coding rate 1..4 = 4/5..4/8 */
    uint8_t  tx_dbm;
} rylr_settings_t;

enum {
    RYLR_OK                = 0,
    RYLR_ERR_NO_RESPONSE   = -1,     /* nothing sensible came back: wiring, power, baud rate */
    RYLR_ERR_BAD_REPLY     = -2,     /* bytes arrived, but not the answers we expected */
    RYLR_ERR_REJECTED      = -100    /* the module said +ERR=n: the value is -100 - n */
};

typedef enum { RL_PROBE, RL_QUERY, RL_SET, RL_READY, RL_FAILED } rylr_state_t;

typedef void (*rylr_rcv_fn)(void *ctx, uint16_t addr, const char *hex, uint16_t hexlen, int rssi, int snr);

#define RYLR_ITEMS 5

typedef struct {
    rylr_io_t io;
    rylr_settings_t cfg;
    rylr_state_t state;
    uint8_t  item, tries, waiting, tx_pending;
    uint32_t t_cmd_ms, tx_started_ms, tx_deadline_ms, last_airtime_us;
    char     query[RYLR_ITEMS][20], key[RYLR_ITEMS][12], want[RYLR_ITEMS][32], set[RYLR_ITEMS][48];
    char     cmd[RYLR_MAX_LINE];
    rylr_lines_t lines;
    rylr_msg_t   msg;
    rylr_rcv_fn  on_rcv; void *on_rcv_ctx;
    int      last_error;
    uint32_t rx_bytes, rx_bytes_at_start;
    uint32_t tx_ok, tx_err, tx_timeout, set_commands, reconfigs, rcv_count;
} rylr_link_t;

/* Blocking start-up (runs the state machine to completion, a few seconds at most). Returns RYLR_OK or an error. */
int  rylr_link_init(rylr_link_t *l, const rylr_io_t *io, const rylr_settings_t *cfg);
void rylr_link_set_rcv(rylr_link_t *l, rylr_rcv_fn fn, void *ctx);
void rylr_link_poll(rylr_link_t *l, uint32_t now_ms);                               /* call often */
int  rylr_link_send(rylr_link_t *l, const uint8_t *bin, size_t n, uint32_t now_ms); /* 0 = started, -1 = not possible right now */
int  rylr_link_send_text(rylr_link_t *l, const char *text, size_t n, uint32_t now_ms);   /* plain ASCII payload */
int  rylr_link_busy(const rylr_link_t *l);
uint32_t rylr_bw_hz(uint8_t bw_code);

#endif
