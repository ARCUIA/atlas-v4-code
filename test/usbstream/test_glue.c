/* Host test of app/usb_stream.c (the USB glue) against a fake USB layer:
 * gating by the dump protocol, no transmit before enumeration, no rewrite of an in-flight buffer,
 * stall detection, recovery, and the IMU wrapper's unit conversion. */
#include <stdio.h>
#include <string.h>
#include "usb_stream.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

/* ---- fakes ---- */
USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t now_ms;
static int quiet, busy_dump;
static int tx_result;                      /* what CDC_Transmit_FS returns */
static const uint8_t *tx_ptr; static uint16_t tx_len; static uint8_t tx_copy[1024]; static int tx_calls;
static char sent[65536]; static uint32_t sent_n;

uint32_t time_ms(void) { return now_ms; }
int dump_quiet(void) { return quiet; }
int dump_busy(void)  { return busy_dump; }
uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len)
{
    tx_calls++;
    if (cdc.TxState) return USBD_BUSY;
    if (tx_result != USBD_OK) return (uint8_t)tx_result;
    tx_ptr = buf; tx_len = len; memcpy(tx_copy, buf, len);
    cdc.TxState = 1;                       /* stays busy until the test "completes" the transfer */
    return USBD_OK;
}
static void complete_tx(void) { if (cdc.TxState) { memcpy(sent + sent_n, tx_copy, tx_len); sent_n += tx_len; cdc.TxState = 0; } }
/* imu.c / baro.c are not linked: same conversions as the ST driver */
float imu_xl_mg(int16_t r)   { return (float)r * 0.488f; }
float imu_hg_mg(int16_t r)   { return (float)r * 3.904f; }
float imu_gy_mdps(int16_t r) { return (float)r * 70.0f; }

static void reset(void)
{
    memset(&cdc, 0, sizeof cdc); hUsbDeviceFS.pClassData = &cdc; hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    quiet = busy_dump = 0; tx_result = USBD_OK; tx_calls = 0; sent_n = 0; memset(sent, 0, sizeof sent);
    now_ms = 1000; usb_stream_init();
}
static void line(void) { imu_raw_t s = {0}; s.xl_raw[2] = 2049; usb_stream_imu(123456789ull, &s); }

static void test_not_enumerated(void)
{
    reset(); hUsbDeviceFS.pClassData = NULL;                       /* no host: pClassData does not exist */
    usb_stream_poll(); CHECK(!usb_stream_active()); line(); usb_stream_poll();
    CHECK(tx_calls == 0 && usbstream_pending(usb_stream_stats()) == 0);
    reset(); hUsbDeviceFS.dev_state = 0;                           /* attached but not configured */
    usb_stream_poll(); CHECK(!usb_stream_active() && tx_calls == 0);
}

static void test_normal_flow_and_inflight(void)
{
    reset(); usb_stream_poll(); CHECK(usb_stream_active());
    line(); line();
    usb_stream_poll();                                             /* idle link: chunk handed over */
    CHECK(tx_calls == 1 && cdc.TxState == 1);
    uint16_t first = tx_len; CHECK(first > 0 && strncmp((char *)tx_ptr, "IMU t=123.456789", 16) == 0);
    for (int i = 0; i < 5; i++) { line(); now_ms += 5; usb_stream_poll(); }   /* keep producing while in flight */
    CHECK(tx_calls == 1);                                          /* no second transfer while busy */
    CHECK(memcmp(tx_ptr, tx_copy, first) == 0);                    /* buffer the core is reading was NOT rewritten */
    complete_tx(); usb_stream_poll();                              /* link idle again: next chunk goes out */
    CHECK(tx_calls == 2);
    complete_tx();
    CHECK(strstr(sent, "xl[mg]=0,0,1000") != NULL);                /* 2049 * 0.488 = 1000.0 mg, rounded */
}

static void test_transmit_refused_keeps_data(void)
{
    reset(); usb_stream_poll(); line();
    uint32_t pend = usbstream_pending(usb_stream_stats());
    tx_result = USBD_BUSY; usb_stream_poll();                      /* core said no */
    CHECK(usbstream_pending(usb_stream_stats()) == pend);          /* data kept */
    tx_result = USBD_OK; usb_stream_poll(); CHECK(usbstream_pending(usb_stream_stats()) == 0);
}

static void test_dump_has_priority(void)
{
    reset(); usb_stream_poll(); line(); CHECK(usbstream_pending(usb_stream_stats()) > 0);
    quiet = 1; usb_stream_poll();                                  /* a dump command arrived */
    CHECK(!usb_stream_active() && usbstream_pending(usb_stream_stats()) == 0 && tx_calls == 0);
    line(); usb_stream_poll(); CHECK(tx_calls == 0);               /* stays silent while quiet */
    quiet = 0; busy_dump = 1; usb_stream_poll(); CHECK(!usb_stream_active() && tx_calls == 0);   /* streaming sectors */
    busy_dump = 0; usb_stream_poll(); CHECK(usb_stream_active());  /* resumes by itself afterwards */
    line(); usb_stream_poll(); CHECK(tx_calls == 1);
}

static void test_host_stops_reading(void)
{
    reset(); usb_stream_poll(); line(); usb_stream_poll(); CHECK(cdc.TxState == 1);   /* transfer starts, never completes */
    for (int i = 0; i < 10; i++) { line(); now_ms += 5; usb_stream_poll(); }          /* 50 ms: still waiting */
    CHECK(usb_stream_active());
    now_ms += 100; usb_stream_poll();                                                  /* > USB_STREAM_STALL_MS */
    CHECK(!usb_stream_active() && usbstream_pending(usb_stream_stats()) == 0 && usb_stream_stats()->bytes_flushed > 0);
    line(); CHECK(usbstream_pending(usb_stream_stats()) == 0);                         /* producers are silent */
    complete_tx();                                                                     /* viewer opens the port */
    usb_stream_poll(); CHECK(usb_stream_active());                                     /* recovers, no stale data queued */
    CHECK(usbstream_pending(usb_stream_stats()) == 0);
    line(); usb_stream_poll(); CHECK(tx_calls == 2);
}

static void test_wrappers(void)
{
    reset(); usb_stream_poll();
    imu_raw_t s = {0}; s.xl_raw[0] = -2; s.xl_raw[1] = 1; s.gy_raw[2] = -1;                // -0.976 mg, 0.488 mg, -70 mdps
    usb_stream_imu(1000000ull, &s);
    baro_sample_t b = {0}; b.pressure_hpa = 987.17f; b.temp_c = 21.3f; usb_stream_baro(2000000ull, &b);
    ubx_pvt_t p = {0}; p.fix_type = 3; p.num_sv = 9; p.lat_e7 = 416611234; p.lon_e7 = -915301234;
    p.hmsl_mm = 221600; p.vel_d_mms = 100; p.hacc_mm = 400; p.vacc_mm = 700; usb_stream_gps(3000000ull, &p);
    usb_stream_poll(); complete_tx();
    CHECK(strstr(sent, "IMU t=1.000000 xl[mg]=-1,0,0 gy[mdps]=0,0,-70") != NULL);
    CHECK(strstr(sent, "BARO t=2.000000 p[Pa]=98717 T[C]=21.30") != NULL);
    CHECK(strstr(sent, "GPS t=3.000000 fix=3 sv=9 lat=41.6611234 lon=-91.5301234 hmsl[m]=221.600 vd[mps]=0.100 hacc[m]=0.400 vacc[m]=0.700") != NULL);
    usb_stream_text("STATUS\r\n", 8); usb_stream_poll(); complete_tx(); CHECK(strstr(sent, "STATUS\r\n") != NULL);
}

int main(void)
{
    test_not_enumerated(); test_normal_flow_and_inflight(); test_transmit_refused_keeps_data();
    test_dump_has_priority(); test_host_stops_reading(); test_wrappers();
    if (fails) { printf("%d FAILURES\n", fails); return 1; }
    printf("usb glue: all checks passed\n");
    return 0;
}
