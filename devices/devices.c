#include "devices.h"
#include "main.h"

const char *dev_err_str(dev_err_t e)
{
    switch (e) {
    case DEV_OK:       return "OK";
    case DEV_NO_DATA:  return "none";
    case DEV_ERR_BUS:  return "BUS";
    case DEV_ERR_ID:   return "ID";
    case DEV_ERR_TIMEOUT: return "TIMEOUT";
    case DEV_ERR_PROTO:   return "PROTO";
    case DEV_ERR_CFG:     return "CFG";
    default:           return "?";
    }
}

void devices_cs_idle(void)
{
    /* Chip selects are active-low, so idle = high. CS_MAG is driven too even
     * though the magnetometer is not fitted: it keeps the pin in a known state. */
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port,  CS_IMU_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_BARO_GPIO_Port, CS_BARO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_MAG_GPIO_Port,  CS_MAG_Pin,  GPIO_PIN_SET);
}
