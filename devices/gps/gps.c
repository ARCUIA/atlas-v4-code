#include "gps.h"
#include "gps_cfg.h"
#include "ringbuf.h"
#include "usart.h"          /* huart4 (CubeMX) */
#include "main.h"
#include <string.h>

#define RING_SIZE 1024u     /* power of two */

static uint8_t     ring_store[RING_SIZE];
static ringbuf_t   ring;
static uint8_t     rx_byte;
static ubx_parser_t parser;
static uint8_t     pvt_raw[UBX_NAV_PVT_LEN];
static ubx_pvt_t   last;
static uint8_t     pvt_new;
static int         cfg_code = GPSCFG_NO_RESPONSE;
static uint32_t    baud_found;
static uint32_t    n_pvt, n_bytes, armed_ms;
static volatile uint32_t err_count;

/* ---- hardware access for the configuration sequence ----
 * Reception is interrupt-driven from the very first byte: the F4 UART has a single-byte data register,
 * so polling it with pauses in between loses most of a 100-byte reply (that is why the first version of
 * this code reported "no response"). The ISR fills the ring buffer; the configuration code reads from it. */
static void io_set_baud(uint32_t baud)
{
    (void)HAL_UART_AbortReceive(&huart4);
    huart4.Init.BaudRate = baud;
    HAL_UART_Init(&huart4);                                  /* re-initialising stops any reception in progress ... */
    (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);         /* ... so start it again at the new rate */
}
static void io_write(const uint8_t *p, uint16_t n) { HAL_UART_Transmit(&huart4, (uint8_t *)p, n, 200); }
static int io_read_byte(uint8_t *b) { return ringbuf_read(&ring, b, 1) == 1; }
static uint32_t io_now(void) { return HAL_GetTick(); }
static void io_delay(uint32_t ms) { HAL_Delay(ms); }

int gps_init(void)
{
    static const gps_io_t io = { io_set_baud, io_write, io_read_byte, io_now, io_delay };
    gps_cfg_t cfg = { GPS_TARGET_BAUD, GPS_MEAS_PERIOD_MS, GPS_GPS_ONLY, GPS_DYNMODEL };
    gps_cfg_result_t res;

    ringbuf_init(&ring, ring_store, RING_SIZE);
    ubx_parser_init(&parser);
    n_pvt = n_bytes = err_count = 0; pvt_new = 0;

    cfg_code = gps_configure(&io, &cfg, &res);
    baud_found = res.baud_found;

    /* Whatever happened, leave UART4 at the target rate with reception running, so a receiver that only
     * needed more time still gets picked up. */
    if (res.baud_final != GPS_TARGET_BAUD) io_set_baud(GPS_TARGET_BAUD);
    err_count = 0;                                           /* errors while hunting for the baud rate do not count */
    armed_ms = HAL_GetTick();
    (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);         /* already running: harmless if so */
    return cfg_code;
}

/* Interrupt context: keep tiny. Called from app_isr.c, which dispatches the HAL UART callbacks. */
void gps_uart_rx_cplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART4) return;
    ringbuf_write(&ring, &rx_byte, 1);
    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void gps_uart_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART4) return;
    err_count++;
    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);          /* an overrun aborts reception: re-arm it */
}

void gps_process(void)
{
    uint8_t chunk[32];
    uint32_t n;
    while ((n = ringbuf_read(&ring, chunk, sizeof chunk)) > 0) {
        n_bytes += n;
        for (uint32_t i = 0; i < n; i++) {
            if (ubx_parser_feed(&parser, chunk[i]) && parser.cls == UBX_CLS_NAV && parser.id == UBX_ID_PVT &&
                parser.len == UBX_NAV_PVT_LEN) {
                memcpy(pvt_raw, parser.payload, UBX_NAV_PVT_LEN);
                (void)ubx_pvt_decode(pvt_raw, UBX_NAV_PVT_LEN, &last);
                pvt_new = 1;
                n_pvt++;
            }
        }
    }
}

int gps_pop_pvt(const uint8_t **payload92)
{
    if (!pvt_new) return 0;
    pvt_new = 0;
    *payload92 = pvt_raw;
    return 1;
}

const ubx_pvt_t *gps_last(void)   { return &last; }
int  gps_cfg_code(void)           { return cfg_code; }
int  gps_cfg_baud_found(void)     { return (int)baud_found; }
uint32_t gps_pvt_count(void)      { return n_pvt; }
uint32_t gps_byte_count(void)     { return n_bytes; }
uint32_t gps_error_count(void)    { return err_count; }
int  gps_irq_stalled(void)        { return n_bytes == 0 && (HAL_GetTick() - armed_ms) > 2000u; }

void gps_led_poll(uint32_t now_ms)
{
    int fix = (last.fix_type >= 2) && (last.flags & 0x01u);
    uint32_t period = fix ? 15000u : 1000u;
    GPIO_PinState on = ((now_ms % period) < 100u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, on);
}
