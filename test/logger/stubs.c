/* Stand-ins for the target-only pieces, so log_service.c and dump.c run on a PC. */
#include "app_time.h"
#include "imu.h"
#include "baro.h"

uint64_t g_us;                                    /* test-controlled clock */
uint64_t time_us(void)  { return g_us; }
uint32_t time_ms(void)  { return (uint32_t)(g_us / 1000u); }
uint16_t time_cpu_mhz(void) { return 168; }
void     time_init(void) {}
void imu_get_cfg(imu_cfg_t *c)
{
    c->xl_fs_g = 16; c->hg_fs_g = 80; c->gy_fs_dps = 2000;
    c->xl_odr_hz = 480; c->hg_odr_hz = 480; c->gy_odr_hz = 480;
}
uint16_t baro_odr_x100(void) { return 2500; }
