#include "radio.h"
#include "rylr_link.h"
#include "ringbuf.h"
#include "usart.h"          /* huart3 (CubeMX) */
#include "main.h"
#include <stdio.h>
#include <string.h>

#define RING_SIZE 512u      /* power of two */

static uint8_t       ring_store[RING_SIZE];
static ringbuf_t     ring;
static uint8_t       rx_byte;
static volatile uint8_t tx_done = 1;
static uint32_t      tx_started_ms, tx_irq_missed;
static volatile uint32_t uart_errors;
static rylr_link_t   link;
static radio_stats_t st;
static uint8_t       attempted;
static uint32_t      last_ok_count;

/* ---- hardware access for the link ---- */
static int io_tx_start(const uint8_t *p, uint16_t n)
{
    tx_done = 0;
    tx_started_ms = HAL_GetTick();
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)p, n) != HAL_OK) { tx_done = 1; return -1; }
    return 0;
}
static int io_tx_idle(void)
{
    if (tx_done) return 1;
    if (HAL_GetTick() - tx_started_ms > 100u) {          /* the completion interrupt never came: USART3 interrupt not enabled? */
        tx_irq_missed++;
        (void)HAL_UART_AbortTransmit(&huart3);
        tx_done = 1;
        return 1;
    }
    return 0;
}
static int io_read_byte(uint8_t *b) { return ringbuf_read(&ring, b, 1) == 1; }
static uint32_t io_now(void) { return HAL_GetTick(); }
static void io_delay(uint32_t ms) { HAL_Delay(ms); }

/* ---- UART interrupt callbacks (dispatched from app_isr.c) ---- */
void radio_uart_rx_cplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    ringbuf_write(&ring, &rx_byte, 1);
    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
}
void radio_uart_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    tx_done = 1;
}
void radio_uart_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    uart_errors++;
    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);       /* an overrun aborts reception: re-arm it */
}

dev_err_t radio_init(void)
{
    static const rylr_io_t io = { io_tx_start, io_tx_idle, io_read_byte, io_now, io_delay };
    const rylr_settings_t cfg = { RADIO_OWN_ADDR, RADIO_DEST_ADDR, RADIO_NETWORK_ID, RADIO_BAND_HZ,
                                  RADIO_SF, RADIO_BW_CODE, RADIO_CR_CODE, RADIO_PREAMBLE, RADIO_TX_DBM };
    attempted = 1;
    ringbuf_init(&ring, ring_store, RING_SIZE);

    (void)HAL_UART_AbortReceive(&huart3);
    huart3.Init.BaudRate = 115200;                        /* the module's factory rate */
    HAL_UART_Init(&huart3);
    (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    int rc = rylr_link_init(&link, &io, &cfg);
    st.init_state = (int16_t)rc;
    st.last_state = (int16_t)rc;
    if (rc == RYLR_OK) return DEV_OK;
    if (rc == RYLR_ERR_NO_RESPONSE) return DEV_ERR_TIMEOUT;
    if (rc == RYLR_ERR_BAD_REPLY)   return DEV_ERR_PROTO;
    return DEV_ERR_CFG;                                   /* the module rejected a setting */
}

void radio_poll(uint32_t now_ms)
{
    if (!attempted || link.state == RL_FAILED) return;
    rylr_link_poll(&link, now_ms);
    st.tx_ok = link.tx_ok; st.tx_fail = link.tx_err; st.tx_timeout = link.tx_timeout;
    st.last_airtime_us = link.last_airtime_us;
    if (link.last_error != 0) st.last_state = (int16_t)link.last_error;
    if (link.tx_ok != last_ok_count) {                    /* LORA_LED blinks on every packet the module accepted and sent */
        last_ok_count = link.tx_ok;
        HAL_GPIO_TogglePin(LORA_LED_GPIO_Port, LORA_LED_Pin);
    }
}

int radio_send_text(const char *text)
{
    if (!attempted || text == NULL) return -1;
    int rc = rylr_link_send_text(&link, text, strlen(text), HAL_GetTick());
    if (rc == 0) st.last_airtime_us = link.last_airtime_us;
    return rc;
}

int radio_send(const uint8_t *data, size_t len)
{
    if (!attempted || len == 0 || len > 120u) return -1;
    int rc = rylr_link_send(&link, data, len, HAL_GetTick());
    if (rc == 0) st.last_airtime_us = link.last_airtime_us;
    return rc;
}

int radio_busy(void) { return attempted && rylr_link_busy(&link); }
const radio_stats_t *radio_stats(void) { return &st; }

const char *radio_status_text(void)
{
    static char text[40];
    if (!attempted) return "n/a";
    if (link.state == RL_READY) return "OK";
    if (link.state != RL_FAILED) return "CONFIG";         /* being set up again after the module restarted */
    const char *why = tx_irq_missed                             ? "NO-IRQ" :       /* enable the USART3 interrupt in CubeMX */
                      link.last_error == RYLR_ERR_NO_RESPONSE   ? "NO-RESP" :
                      link.last_error == RYLR_ERR_BAD_REPLY     ? "BAD-REPLY" : "REJECTED";
    snprintf(text, sizeof text, "%s rx=%lu", why, (unsigned long)link.rx_bytes);
    return text;
}
