#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
typedef struct { uint32_t MODER; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
#define CS_FLASH_Pin (1u << 15)
extern GPIO_TypeDef sim_gpioa;
#define CS_FLASH_GPIO_Port (&sim_gpioa)
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
#endif
