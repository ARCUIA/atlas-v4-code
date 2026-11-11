#ifndef APP_TIME_H
#define APP_TIME_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void     time_init(void);        /* enables the DWT cycle counter */
uint32_t time_ms(void);          /* milliseconds since boot (SysTick) */
uint64_t time_us(void);          /* microseconds since boot, 64-bit, from the CPU cycle counter */
uint16_t time_cpu_mhz(void);
#ifdef __cplusplus
}
#endif
#endif
