#ifndef GPS_H
#define GPS_H
/*
 * SAM-M10Q GPS on UART4 (PA0 TX / PA1 RX).
 *
 * gps_init() finds the receiver's baud rate, moves it to GPS_TARGET_BAUD and configures it
 * (UBX only, fast update rate, airborne <4 g model) -- see app/core/gps_cfg.c. Afterwards
 * every byte arrives by UART4 interrupt into a ring buffer, and gps_process() (main loop)
 * turns them into position solutions.
 *
 * REQUIRES the "UART4 global interrupt" to be enabled in CubeMX (NVIC tab).
 */
#include <stdint.h>
#include "devices.h"
#include "ubx.h"
#include "gps_cfg.h"      /* GPSCFG_* result codes and gps_cfg_str() */
#include "stm32f4xx_hal.h"

/* ---- Settings (see the notes in the README about the update-rate options) ---- */
#define GPS_TARGET_BAUD      115200u
#define GPS_MEAS_PERIOD_MS   56u        /* 56 ms = about 17.9 Hz; 18 Hz is the datasheet maximum for GPS-only */
#define GPS_GPS_ONLY         1          /* 1: GPS only (fastest). 0: leave all constellations as the receiver has them */
#define GPS_DYNMODEL         8          /* 8 = airborne <4 g */

int  gps_init(void);                                   /* returns a GPSCFG_* code (0 = OK); never leaves UART4 unusable */
void gps_process(void);                                /* call every main-loop pass */
int  gps_pop_pvt(const uint8_t **payload92);           /* 1 if a new position solution arrived; payload valid until the next gps_process() */
const ubx_pvt_t *gps_last(void);                       /* the latest decoded solution */
int  gps_cfg_code(void);                               /* result of gps_init() */
int  gps_cfg_baud_found(void);
uint32_t gps_pvt_count(void);
uint32_t gps_byte_count(void);
uint32_t gps_error_count(void);
int  gps_irq_stalled(void);                            /* 1 if no byte arrived by interrupt: UART4 interrupt not enabled? */
void gps_uart_rx_cplt(UART_HandleTypeDef *huart);      /* called from app_isr.c */
void gps_uart_error(UART_HandleTypeDef *huart);
void gps_led_poll(uint32_t now_ms);                    /* GPS_LED: flash every 1 s searching, every 15 s with a fix */

#endif
