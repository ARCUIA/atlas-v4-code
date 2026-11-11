/* USB CDC side of the dump tool (target only). */
#include "dump.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "main.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int dump_tx_ready(void)
{
    USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return h != NULL && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && h->TxState == 0;
}

int dump_tx(const uint8_t *p, uint16_t n)
{
    return CDC_Transmit_FS((uint8_t *)p, n) == USBD_OK ? 0 : -1;
}

uint32_t dump_now_ms(void) { return HAL_GetTick(); }
