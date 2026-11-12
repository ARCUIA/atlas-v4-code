/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "devices.h"
#include "imu.h"
#include "baro.h"
#include "gps.h"
#include "sd_spi.h"
#include "app_time.h"
#include "log_service.h"
#include "dump.h"
#include "radio.h"
#include "telemetry.h"
#include "telemetry_sources.h"
#include "gps_beacon.h"
#include "usb_stream.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static imu_raw_t     imu;
static baro_sample_t baro;
static sd_info_t     sdi;
static dev_err_t     st_imu, st_baro, st_sd, st_log, st_radio = DEV_NO_DATA;
static uint32_t      last_hb, last_print, last_status;
static int           st_gps;
static tlm_t         tlm;

#define ATLAS_SOURCE_ID 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  devices_cs_idle();      /* deselect every SPI2 device before any traffic */
  HAL_Delay(20);          /* let the sensors finish power-up */
  time_init();            /* initialize the time service */
  dump_init();             /* initialize the dump service */
  usb_stream_init();       /* live-viewer text stream over USB CDC (see app/usb_stream.h) */
  st_imu  = imu_init();
  st_baro = baro_init();
  st_sd = sd_init(&sdi);
  if (st_sd == DEV_OK)
  {
    st_log = log_service_init(&sdi);
  }
  st_radio = radio_init();
  tlm_init(&tlm, ATLAS_SOURCE_ID, 1000u);
  telemetry_sources_init(&tlm);
  st_gps = gps_init();                 /* takes up to a second or two */
     log_gps_cfg((uint32_t)gps_cfg_baud_found(), 
                  GPS_TARGET_BAUD, 
                  GPS_MEAS_PERIOD_MS, 
                  (int8_t)st_gps, 
                  GPS_DYNMODEL, 
                  GPS_GPS_ONLY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    uint32_t pxl = imu.n_xl, phg = imu.n_hg, pgy = imu.n_gy;
    if (imu_poll(&imu) == DEV_OK) {
      uint8_t fl = (uint8_t)((imu.n_xl != pxl) | ((imu.n_hg != phg) << 1) | ((imu.n_gy != pgy) << 2));
      log_imu(&imu, fl);
      if (usb_stream_active()) usb_stream_imu(time_us(), &imu);
    }
    if (baro_poll(&baro) == DEV_OK) {
      log_baro(&baro);
      if (usb_stream_active()) usb_stream_baro(time_us(), &baro);
    }

    if (now - last_status >= 1000u) { last_status = now; log_status(); }

    gps_process();
     { const uint8_t *pvt; if (gps_pop_pvt(&pvt)) { log_gps(pvt); if (usb_stream_active()) usb_stream_gps(time_us(), gps_last()); } }
    gps_led_poll(now);

    radio_poll(now);
    /*
    if (!radio_busy()) {
      uint8_t pkt[48];
      size_t plen = tlm_build(&tlm, now, pkt, sizeof pkt);
      if (plen) (void)radio_send(pkt, plen);
    }
    */
    gps_beacon_poll(now);

    log_service_poll();
    dump_poll();
    usb_stream_poll();       /* after dump_poll(): the dump protocol has priority on the port */

    if (now - last_hb >= 500u) {
      last_hb = now;
      HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
    }

    if (!dump_quiet() && now - last_print >= 200u) {
      last_print = now;
      const log_stats_t *ls = log_stats();
      static char line[360];
      int n = snprintf(line, sizeof line,
          "IMU:%s xl[mg]=%d,%d,%d gy[mdps]=%d,%d,%d | BARO:%s %ld Pa | SD:%s LOG:%s "
          "sess=%lu next=%lu wr=%lu drop=%lu q=%lu/%lu err=%lu | "
          "GPS:%s%s fix=%u sv=%u pvt=%lu rxerr=%lu | "
          "RADIO:%s(%d) tx=%lu fail=%lu air=%lums | "
          "USB:%s ln=%lu drop=%lu flush=%lu\r\n",
          dev_err_str(st_imu),
          (int)imu_xl_mg(imu.xl_raw[0]), (int)imu_xl_mg(imu.xl_raw[1]), (int)imu_xl_mg(imu.xl_raw[2]),
          (int)imu_gy_mdps(imu.gy_raw[0]), (int)imu_gy_mdps(imu.gy_raw[1]), (int)imu_gy_mdps(imu.gy_raw[2]),
          dev_err_str(st_baro), (long)(baro.pressure_hpa * 100.0f),
          dev_err_str(st_sd), ls->paused ? "PAUSED" : (st_log == DEV_NO_DATA ? "-" : dev_err_str(st_log)),
          (unsigned long)ls->session, (unsigned long)ls->next_sector, (unsigned long)ls->blocks_written,
          (unsigned long)ls->blocks_dropped, (unsigned long)ls->queue, (unsigned long)ls->max_queue,
          (unsigned long)ls->sd_errors,
          gps_cfg_str(st_gps), gps_irq_stalled() ? "(NO-IRQ)" : "",
          gps_last()->fix_type, gps_last()->num_sv,
          (unsigned long)gps_pvt_count(), (unsigned long)gps_error_count(),
          radio_status_text(), (int)radio_stats()->init_state, (unsigned long)radio_stats()->tx_ok,
          (unsigned long)(radio_stats()->tx_fail + radio_stats()->tx_timeout),
          (unsigned long)(radio_stats()->last_airtime_us / 1000u),
          usb_stream_active() ? "OK" : "IDLE",
          (unsigned long)usb_stream_stats()->lines, (unsigned long)usb_stream_stats()->lines_dropped,
          (unsigned long)usb_stream_stats()->bytes_flushed);
      if (n > 0)
        usb_stream_text(line, n < (int)sizeof line ? n : (int)sizeof line - 1);
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
