/* Interrupt callbacks. HAL provides empty "weak" versions; defining them here overrides them. One place dispatches
 * to every driver that uses the callback, and each driver checks which UART it is for. */
#include "main.h"
#include "gps.h"
#include "radio.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
    /* EXTI sources (IMU and baro data-ready, the standby button) get their own cases here later. */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { gps_uart_rx_cplt(huart); radio_uart_rx_cplt(huart); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { radio_uart_tx_cplt(huart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)  { gps_uart_error(huart); radio_uart_error(huart); }
