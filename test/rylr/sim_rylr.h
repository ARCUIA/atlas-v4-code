#ifndef SIM_RYLR_H
#define SIM_RYLR_H
/* Simulated RYLR998 modules that follow REYAX's "LoRa AT COMMAND GUIDE": command syntax, value ranges,
 * the SF/bandwidth limits, the +ERR codes, which settings live in flash, and delivery over the air. */
#include <stdint.h>
#include <stddef.h>

typedef struct { uint32_t at; char text[320]; } sim_pending_t;

typedef struct sim_mod {
    uint16_t address, network;                 /* flash: survive a reboot */
    uint32_t band; uint8_t sf, bw, cr, pre, crfop;   /* RAM: back to defaults after a reboot */
    unsigned flash_writes;
    char line[400]; int ln;
    struct { uint8_t b; uint32_t at; } out[16384]; int oh, ot;
    sim_pending_t pend[16]; int npend;
    uint32_t tx_busy_until;
    int ok_after_air;                          /* 1: +OK to AT+SEND only once the transmission has finished */
    int dead;                                  /* 1: never answers */
    int garbage;                               /* 1: only ever outputs noise */
    unsigned sent, delivered, errors_sent;
    struct sim_mod *peer;
} sim_mod_t;

extern uint32_t sim_now_ms;

void sim_mod_init(sim_mod_t *m);               /* manufacturer defaults */
void sim_mod_reboot(sim_mod_t *m);             /* power glitch: RAM settings revert, "+RESET +READY" appears */
void sim_pair(sim_mod_t *a, sim_mod_t *b);
void sim_host_write(sim_mod_t *m, const uint8_t *p, uint16_t n);
int  sim_host_read(sim_mod_t *m, uint8_t *b);  /* 1 if a byte is available now */
#endif
