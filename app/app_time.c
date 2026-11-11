#include "app_time.h"
#include "main.h"

/* The 32-bit cycle counter wraps every ~25 s at 168 MHz. time_us() extends it to 64 bits by
 * accumulating deltas, so it must be called at least every ~25 s (the main loop does, constantly). */
static uint32_t last_cyc;
static uint64_t total_cyc;

void time_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    last_cyc = 0;
    total_cyc = 0;
}

uint32_t time_ms(void)       { return HAL_GetTick(); }
uint16_t time_cpu_mhz(void)  { return (uint16_t)(SystemCoreClock / 1000000u); }

uint64_t time_us(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = DWT->CYCCNT;
    total_cyc += (uint32_t)(now - last_cyc);
    last_cyc = now;
    uint64_t cyc = total_cyc;
    __set_PRIMASK(primask);
    return cyc / (SystemCoreClock / 1000000u);
}
