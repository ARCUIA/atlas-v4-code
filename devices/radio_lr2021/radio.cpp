#include "radio.h"
#include "radiolib_hal.h"
#include <stdio.h>

static StmHal hal(&hspi1);
static Module mod(&hal, RL_PIN_CS, RL_PIN_IRQ, RL_PIN_RST, RL_PIN_BUSY);   /* last argument is BUSY */
static LR2021 lr(&mod);

static volatile bool irq_flag;
static bool          tx_active, ready, attempted;
static uint32_t      tx_started_ms;
static radio_stats_t st;

static void on_irq(void) { irq_flag = true; }          /* RadioLib calls this from the interrupt */

extern "C" dev_err_t radio_init(void)
{
    /* Must be set before begin(): the module's TCXO is powered at 3.0 V, and IRQ is wired to DIO9. */
    lr.tcxoVoltage = 3.0f;
    lr.irqDioNum   = RADIO_IRQ_DIO;

    ConfigLoRa_t cfg;
    cfg.frequency       = RADIO_FREQ_MHZ;
    cfg.bandwidth       = RADIO_BW_KHZ;
    cfg.spreadingFactor = RADIO_SF;
    cfg.codingRate      = RADIO_CR_DEN;
    cfg.syncWord        = RADIO_SYNC_WORD;
    cfg.power           = RADIO_TX_DBM;
    cfg.preambleLength  = RADIO_PREAMBLE;

    attempted = true;
    st.init_state = lr.begin(cfg);
    st.last_state = st.init_state;
    ready = false;
    if (st.init_state != RADIOLIB_ERR_NONE) {
        /* Return a reason the existing "RADIO:%s" status text already shows: TIMEOUT, ID, CFG, or BUS. */
        switch (st.init_state) {
        case RADIOLIB_ERR_SPI_CMD_TIMEOUT:          return DEV_ERR_TIMEOUT;   /* BUSY never went low */
        case RADIOLIB_ERR_CHIP_NOT_FOUND:           return DEV_ERR_ID;        /* SPI link to the chip does not work */
        case RADIOLIB_ERR_INVALID_FREQUENCY:
        case RADIOLIB_ERR_INVALID_BANDWIDTH:
        case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
        case RADIOLIB_ERR_INVALID_CODING_RATE:
        case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
        case RADIOLIB_ERR_INVALID_TCXO_VOLTAGE:     return DEV_ERR_CFG;       /* a setting was rejected */
        default:                                    return DEV_ERR_BUS;
        }
    }
    lr.setIrqAction(on_irq);
    ready = true;
    return DEV_OK;
}

extern "C" int radio_send(const uint8_t *data, size_t len)
{
    if (!ready || tx_active || len == 0 || len > 255) return -1;      /* a radio that failed to start is left alone */
    irq_flag = false;
    st.last_state = lr.startTransmit(data, len);
    if (st.last_state != RADIOLIB_ERR_NONE) { st.tx_fail++; return -1; }
    st.last_airtime_us = (uint32_t)lr.getTimeOnAir(len);
    tx_started_ms = HAL_GetTick();
    tx_active = true;
    return 0;
}

extern "C" void radio_poll(uint32_t now_ms)
{
    if (!tx_active) return;
    if (irq_flag) {                                      /* transmission finished */
        irq_flag = false;
        lr.finishTransmit();
        tx_active = false;
        st.tx_ok++;
        HAL_GPIO_TogglePin(LORA_LED_GPIO_Port, LORA_LED_Pin);
    } else if ((uint32_t)(now_ms - tx_started_ms) > st.last_airtime_us / 1000u + 500u) {
        lr.standby();                                    /* the interrupt never came: give up on this packet */
        tx_active = false;
        st.tx_timeout++;
    }
}

extern "C" int radio_busy(void)           { return tx_active; }
extern "C" void radio_irq_isr(void)       { hal.irqFired(); }
extern "C" const radio_stats_t *radio_stats(void) { return &st; }

/* One letter for how CubeMX configured a pin: i = input, o = output, f = alternate function (e.g. hardware NSS), a = analog. */
static char pin_mode_char(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t m = (port->MODER >> (2u * (uint32_t)__builtin_ctz((unsigned)pin))) & 3u;
    return "iofa"[m];
}
static unsigned pin_level(GPIO_TypeDef *port, uint16_t pin) { return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET ? 1u : 0u; }

static const char *init_name(void)
{
    switch (st.init_state) {
    case RADIOLIB_ERR_CHIP_NOT_FOUND:        return "NO-CHIP";      /* SPI link to the chip does not work */
    case RADIOLIB_ERR_SPI_CMD_TIMEOUT:       return "SPI-TIMEOUT";  /* BUSY never went low */
    case RADIOLIB_ERR_SPI_CMD_INVALID:       return "SPI-INVALID";
    case RADIOLIB_ERR_SPI_CMD_FAILED:        return "SPI-FAILED";
    case RADIOLIB_ERR_INVALID_FREQUENCY:     return "BAD-FREQ";     /* wrong unit or out of range */
    case RADIOLIB_ERR_INVALID_BANDWIDTH:     return "BAD-BW";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "BAD-SF";
    case RADIOLIB_ERR_INVALID_CODING_RATE:   return "BAD-CR";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER:  return "BAD-PWR";
    case RADIOLIB_ERR_INVALID_TCXO_VOLTAGE:  return "BAD-TCXO";
    default:                                 return "INIT-ERR";
    }
}

/* "OK", "n/a", or, when start-up failed, the reason plus the LIVE state of the three control pins:
 *   B = BUSY (healthy and ready: 0i)   R = RESET (healthy: 1o)   C = chip select (healthy: 1o)
 * digit = pin level now, letter = how the pin is configured (i input, o output, f alternate function, a analog). */
extern "C" const char *radio_status_text(void)
{
    static char text[48];
    if (!attempted) return "n/a";
    if (ready)      return "OK";
    snprintf(text, sizeof text, "%s B%u%c R%u%c C%u%c", init_name(),
             pin_level(BUSY_LORA_GPIO_Port, BUSY_LORA_Pin),   pin_mode_char(BUSY_LORA_GPIO_Port, BUSY_LORA_Pin),
             pin_level(RESET_LORA_GPIO_Port, RESET_LORA_Pin), pin_mode_char(RESET_LORA_GPIO_Port, RESET_LORA_Pin),
             pin_level(CS_LORA_GPIO_Port, CS_LORA_Pin),       pin_mode_char(CS_LORA_GPIO_Port, CS_LORA_Pin));
    return text;
}
