#ifndef DUMP_H
#define DUMP_H
/*
 * Pulls the log off the soldered SD card over USB (CDC serial). Text commands, one per line:
 *     i            -> "ATLAS used=<blocks> total=<sectors> gen=<g> session=<s>"
 *     r <a> <n>    -> "ATLD <a> <n>" then n*512 raw bytes (sectors a .. a+n-1)
 *     x!           -> erase the log (starts a new generation)
 * A command pauses logging until the next reset. While commands are being served, and for 3 s after
 * the last one, the status printout stays off the link so the replies are clean. Anything that is not
 * one of these commands (modem probes, stray terminal characters) is ignored entirely.
 *
 * dump_rx() runs in the USB interrupt; everything else runs in the main loop.
 * The platform provides dump_tx_ready()/dump_tx() (see dump_usb.c).
 */
#include <stdint.h>

void dump_init(void);
void dump_rx(const uint8_t *buf, uint32_t len);   /* call from CDC_Receive_FS */
void dump_poll(void);                             /* call every main-loop pass */
int  dump_quiet(void);                            /* 1 while a command is being served: do not print other text */
int  dump_active(void);                           /* 1 while streaming sectors */
int  dump_busy(void);                             /* 1 while a command is waiting to be handled or sectors are streaming */

/* Provided by the platform. */
int dump_tx_ready(void);                          /* 1 if a new USB transmit can start now */
int dump_tx(const uint8_t *p, uint16_t n);        /* 0 on success */
uint32_t dump_now_ms(void);                       /* milliseconds since boot */

#endif
