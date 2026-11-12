#include "usb_stream.h"
#include "dump.h"
#include "app_time.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t     ring_store[USB_STREAM_RING_BYTES];
static uint8_t     txbuf[USB_STREAM_TX_CHUNK];   /* only rewritten while no transfer is in flight */
static usbstream_t st;
static uint8_t     active, stalled, busy_flag;
static uint32_t    busy_t0, imu_div;

static int32_t iround(float f) { return (int32_t)(f + (f < 0.0f ? -0.5f : 0.5f)); }

void usb_stream_init(void)
{
    (void)usbstream_init(&st, ring_store, USB_STREAM_RING_BYTES);
    active = stalled = busy_flag = 0;
    imu_div = 0;
}

int usb_stream_active(void) { return active; }
const usbstream_t *usb_stream_stats(void) { return &st; }

void usb_stream_poll(void)
{
    uint32_t now = time_ms();
    USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    /* 1. The dump protocol owns the port: say nothing, and drop what is queued. */
    if (dump_quiet() || dump_busy()) { active = 0; usbstream_flush(&st); busy_flag = 0; return; }

    /* 2. No host: pClassData does not exist until the host configures the device. */
    if (h == NULL || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        active = 0; usbstream_flush(&st); busy_flag = 0; return;
    }

    /* 3. A transfer is still in flight: leave the buffers alone. If it never finishes, nobody is
     *    reading the port (terminal closed): stop queueing and drop stale data. */
    if (h->TxState != 0) {
        if (!busy_flag) { busy_flag = 1; busy_t0 = now; }
        else if (!stalled && (now - busy_t0) > USB_STREAM_STALL_MS) stalled = 1;
        if (stalled) { active = 0; usbstream_flush(&st); }
        return;
    }
    busy_flag = 0;
    stalled = 0;
    active = 1;

    /* 4. Idle link: hand over the next chunk; only forget it once the core accepted it. */
    uint32_t n = usbstream_peek(&st, txbuf, USB_STREAM_TX_CHUNK);
    if (n && CDC_Transmit_FS(txbuf, (uint16_t)n) == USBD_OK)
        usbstream_consume(&st, n);
}

void usb_stream_imu(uint64_t t_us, const imu_raw_t *s)
{
    if (!active) return;
    if (++imu_div < USB_STREAM_IMU_DIV) return;
    imu_div = 0;
    int32_t xl[3], gy[3];
    for (int i = 0; i < 3; i++) {
        xl[i] = iround(imu_xl_mg(s->xl_raw[i]));
        gy[i] = iround(imu_gy_mdps(s->gy_raw[i]));
    }
#if USB_STREAM_IMU_HG
    int32_t hg[3];
    for (int i = 0; i < 3; i++) hg[i] = iround(imu_hg_mg(s->hg_raw[i]));
    usbstream_imu(&st, t_us, xl, gy, hg);
#else
    usbstream_imu(&st, t_us, xl, gy, NULL);
#endif
}

void usb_stream_baro(uint64_t t_us, const baro_sample_t *b)
{
    if (!active) return;
    usbstream_baro(&st, t_us, iround(b->pressure_hpa * 100.0f), iround(b->temp_c * 100.0f));
}

void usb_stream_gps(uint64_t t_us, const ubx_pvt_t *p)
{
    if (!active) return;
    usbstream_gps(&st, t_us, p->fix_type, p->num_sv, p->lat_e7, p->lon_e7,
                  p->hmsl_mm, p->vel_d_mms, p->hacc_mm, p->vacc_mm);
}

void usb_stream_mag(uint64_t t_us, const int32_t mag_mG[3])
{
    if (!active) return;
    usbstream_mag(&st, t_us, mag_mG);
}

void usb_stream_text(const char *line, int n)
{
    if (!active || n <= 0) return;
    usbstream_text(&st, line, (uint32_t)n);
}
