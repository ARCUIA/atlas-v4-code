#ifndef RADIO_H
#define RADIO_H
/*
 * LR2021 radio (Seeed Wio-LR2021 module on SPI1), 915 MHz LoRa, transmit only for now.
 * Non-blocking: radio_send() starts a transmission and returns; radio_poll() finishes it when the
 * interrupt reports "done". The ground station must use the same link settings below.
 */
#include <stdint.h>
#include <stddef.h>
#include "devices.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ---- link settings ----
 * SF12 at 500 kHz is within about 1 dB of the best sensitivity that still fits one packet per second
 * (about 330 ms of airtime for the 24-byte GPS packet), and a 500 kHz channel avoids the 400 ms dwell limit that applies
 * to narrow channels under US 915 MHz rules. Check the current rules for your use. */
#define RADIO_FREQ_MHZ    915.0f       /* RadioLib 7.8.1 wants MHz here ... */
#define RADIO_BW_KHZ      500.0f       /* ... and kHz here (not Hz!) */
#define RADIO_SF          12
#define RADIO_CR_DEN      5            /* coding rate 4/5 */
#define RADIO_PREAMBLE    8
#define RADIO_SYNC_WORD   0x12         /* "private" network word */
#define RADIO_TX_DBM      22           /* module maximum. Use about 10 on the bench, and never transmit without an antenna */
#define RADIO_IRQ_DIO     9            /* IRQ_LORA is wired to the module's DIO9 (RadioLib's default is DIO5) */

typedef struct {
    uint32_t tx_ok, tx_fail, tx_timeout;
    uint32_t last_airtime_us;
    int16_t  init_state;               /* RadioLib's result from start-up (0 = OK, negative = the reason it failed) */
    int16_t  last_state;               /* RadioLib's result from the most recent transmit attempt */
} radio_stats_t;

dev_err_t radio_init(void);
int       radio_send(const uint8_t *data, size_t len);   /* 0 = started, -1 = busy or error */
void      radio_poll(uint32_t now_ms);                   /* call every main-loop pass */
int       radio_busy(void);
void      radio_irq_isr(void);                           /* call from the IRQ_LORA EXTI interrupt */
const radio_stats_t *radio_stats(void);
const char *radio_status_text(void);             /* "OK", "n/a" (not started yet) or a short reason such as "SPI-TIMEOUT" */

#ifdef __cplusplus
}
#endif
#endif
