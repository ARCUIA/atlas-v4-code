#ifndef STUB_USBD_CDC_H
#define STUB_USBD_CDC_H
#include <stdint.h>
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { void *pClassData; int dev_state; } USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 3
#endif
