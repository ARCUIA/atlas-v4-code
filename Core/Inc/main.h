/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PYRO_3_Pin GPIO_PIN_13
#define PYRO_3_GPIO_Port GPIOC
#define PYRO_2_Pin GPIO_PIN_14
#define PYRO_2_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_15
#define ERROR_LED_GPIO_Port GPIOC
#define PYRO_MULTISENSE_Pin GPIO_PIN_0
#define PYRO_MULTISENSE_GPIO_Port GPIOC
#define BATT_SENSE_Pin GPIO_PIN_1
#define BATT_SENSE_GPIO_Port GPIOC
#define GPS_LED_Pin GPIO_PIN_2
#define GPS_LED_GPIO_Port GPIOC
#define LORA_LED_Pin GPIO_PIN_3
#define LORA_LED_GPIO_Port GPIOC
#define IRQ_LORA_Pin GPIO_PIN_2
#define IRQ_LORA_GPIO_Port GPIOA
#define IRQ_LORA_EXTI_IRQn EXTI2_IRQn
#define BUSY_LORA_Pin GPIO_PIN_3
#define BUSY_LORA_GPIO_Port GPIOA
#define CS_LORA_Pin GPIO_PIN_4
#define CS_LORA_GPIO_Port GPIOA
#define SCK_LORA_Pin GPIO_PIN_5
#define SCK_LORA_GPIO_Port GPIOA
#define MISO_LORA_Pin GPIO_PIN_6
#define MISO_LORA_GPIO_Port GPIOA
#define MOSI_LORA_Pin GPIO_PIN_7
#define MOSI_LORA_GPIO_Port GPIOA
#define RESET_LORA_Pin GPIO_PIN_4
#define RESET_LORA_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_5
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_INT1_EXTI_IRQn EXTI9_5_IRQn
#define BARO_INT_Pin GPIO_PIN_0
#define BARO_INT_GPIO_Port GPIOB
#define BARO_INT_EXTI_IRQn EXTI0_IRQn
#define IMU_INT2_Pin GPIO_PIN_1
#define IMU_INT2_GPIO_Port GPIOB
#define IMU_INT2_EXTI_IRQn EXTI1_IRQn
#define CS_IMU_Pin GPIO_PIN_2
#define CS_IMU_GPIO_Port GPIOB
#define EXT_TX_Pin GPIO_PIN_10
#define EXT_TX_GPIO_Port GPIOB
#define EXT_RX_Pin GPIO_PIN_11
#define EXT_RX_GPIO_Port GPIOB
#define CS_BARO_Pin GPIO_PIN_12
#define CS_BARO_GPIO_Port GPIOB
#define SCK_MEMS_Pin GPIO_PIN_13
#define SCK_MEMS_GPIO_Port GPIOB
#define MISO_MEMS_Pin GPIO_PIN_14
#define MISO_MEMS_GPIO_Port GPIOB
#define MOSI_MEMS_Pin GPIO_PIN_15
#define MOSI_MEMS_GPIO_Port GPIOB
#define CS_MAG_Pin GPIO_PIN_8
#define CS_MAG_GPIO_Port GPIOA
#define STANDBY_TOGGLE_Pin GPIO_PIN_10
#define STANDBY_TOGGLE_GPIO_Port GPIOA
#define STANDBY_TOGGLE_EXTI_IRQn EXTI15_10_IRQn
#define CS_FLASH_Pin GPIO_PIN_15
#define CS_FLASH_GPIO_Port GPIOA
#define SCK_FLASH_Pin GPIO_PIN_10
#define SCK_FLASH_GPIO_Port GPIOC
#define MISO_FLASH_Pin GPIO_PIN_11
#define MISO_FLASH_GPIO_Port GPIOC
#define MOSI_FLASH_Pin GPIO_PIN_12
#define MOSI_FLASH_GPIO_Port GPIOC
#define MOTOR_POWER_SWITCH_Pin GPIO_PIN_2
#define MOTOR_POWER_SWITCH_GPIO_Port GPIOD
#define CAMERA_POWER_SWITCH_Pin GPIO_PIN_4
#define CAMERA_POWER_SWITCH_GPIO_Port GPIOB
#define PYRO_SEL0_Pin GPIO_PIN_5
#define PYRO_SEL0_GPIO_Port GPIOB
#define EXT_SCL_Pin GPIO_PIN_6
#define EXT_SCL_GPIO_Port GPIOB
#define EXT_SDA_Pin GPIO_PIN_7
#define EXT_SDA_GPIO_Port GPIOB
#define PYRO_1_Pin GPIO_PIN_8
#define PYRO_1_GPIO_Port GPIOB
#define PYRO_SEL1_Pin GPIO_PIN_9
#define PYRO_SEL1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
