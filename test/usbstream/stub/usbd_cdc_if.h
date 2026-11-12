#ifndef STUB_USBD_CDC_IF_H
#define STUB_USBD_CDC_IF_H
#include <stdint.h>
#include "usbd_cdc.h"
#define USBD_OK 0
#define USBD_BUSY 1
uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);
#endif
