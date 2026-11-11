#ifndef RADIO_H
#define RADIO_H
/*
 * Radio: a REYAX RYLR998 LoRa module on USART3 (the board's EXT_TX = PB10, EXT_RX = PB11), transmit only for now.
 * It sends the GPS position as text that the ground receiver understands (see app/core/gpsmsg.h).
 * The module takes text (AT) commands over the serial port; the protocol lives in app/core/rylr_link.c.
 *
 * Non-blocking after start-up: radio_send() starts a packet, radio_poll() collects the module's answer.
 * Needs the USART3 global interrupt enabled in CubeMX (NVIC tab).
 */
#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "devices.h"

/* ---- link settings: they MUST match the receiving module ----
 * Network id, addresses and band are what the modules are set to now: this board's module is network 6, address 0;
 * the receiver's is network 6, address 1. Packets go to the receiver's address (use 0 instead to broadcast to
 * every module on network 6, which keeps working if the receiver module is ever swapped).
 * The RF parameters below are the module's factory defaults. Neither teammate sketch changes them, so unless someone
 * did with AT commands they are what the receiver uses; run ground/rylr_settings_reader on the receiver to check.
 * Range note: the module only allows SF9 with 125 kHz, SF10 with 250 kHz and SF11 with 500 kHz. All three have about
 * the same sensitivity (-129.5, -129 and -128.5 dBm), so the default is already about as good as this module gets. */
#define RADIO_OWN_ADDR     0               /* this board's module (0xFFFF = leave whatever it has) */
#define RADIO_DEST_ADDR    1               /* the receiver's module (0 = broadcast) */
#define RADIO_NETWORK_ID   6               /* both ends must match; 3..15 or 18 */
#define RADIO_BAND_HZ      915000000UL
#define RADIO_SF           9
#define RADIO_BW_CODE      7               /* 7 = 125 kHz, 8 = 250 kHz, 9 = 500 kHz */
#define RADIO_CR_CODE      1               /* 1 = 4/5 */
#define RADIO_PREAMBLE     12              /* networks other than 18 only allow 12 */
#define RADIO_TX_DBM       22              /* 0..22. Use about 10 on the bench, and never transmit without an antenna */

typedef struct {
    uint32_t tx_ok, tx_fail, tx_timeout;
    uint32_t last_airtime_us;
    int16_t  init_state;               /* 0 = OK; negative = why start-up failed (-1 no reply, -2 bad reply, -100-n = module said +ERR=n) */
    int16_t  last_state;
} radio_stats_t;

dev_err_t radio_init(void);                              /* configures the module (blocks for under a second normally) */
int       radio_send_text(const char *text);             /* 0 = started, -1 = busy or not ready. Printable ASCII, at most 240 characters. */
int       radio_send(const uint8_t *data, size_t len);   /* binary data, sent as hex text. At most 120 bytes. */
void      radio_poll(uint32_t now_ms);                   /* call every main-loop pass */
int       radio_busy(void);
const radio_stats_t *radio_stats(void);
const char *radio_status_text(void);                     /* "OK", "n/a", "CONFIG", or a reason such as "NO-RESP rx=0" */

/* UART callbacks, called from app_isr.c */
void radio_uart_rx_cplt(UART_HandleTypeDef *huart);
void radio_uart_tx_cplt(UART_HandleTypeDef *huart);
void radio_uart_error(UART_HandleTypeDef *huart);

#endif
