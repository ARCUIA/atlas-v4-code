#ifndef SPI_H
#define SPI_H
#include "main.h"
#define SPI_NSS_SOFT 0x200u
#define SPI_BAUDRATEPRESCALER_4   0x08u
#define SPI_BAUDRATEPRESCALER_128 0x30u
typedef struct { uint32_t NSS; uint32_t BaudRatePrescaler; } SPI_InitTypeDef;
typedef struct { SPI_InitTypeDef Init; } SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi3;
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
