#ifndef USBSTREAM_H
#define USBSTREAM_H
/*
 * USB text stream, HAL-free (runs on the PC too, see test/usbstream).
 *
 * Formats timestamped, human-readable sample lines into a byte ring buffer. The platform side
 * (app/usb_stream.c) moves the ring to the USB CDC port. Line formats (all end in \r\n):
 *
 *   IMU  t=1234.567890 xl[mg]=-86,-99,988 gy[mdps]=-280,210,140 [hg[mg]=x,y,z]
 *   MAG  t=1234.567890 mag[mG]=123,-45,300
 *   BARO t=1234.567890 p[Pa]=98717 T[C]=21.30
 *   GPS  t=1234.567890 fix=3 sv=9 lat=41.6611234 lon=-91.5301234 hmsl[m]=221.600 vd[mps]=0.100 hacc[m]=0.400 vacc[m]=0.700
 *
 * t is seconds since boot from the same clock as the SD log (time_us()).
 *
 * Rules: one context only (the main loop) calls everything here. A line is either stored whole or
 * dropped whole and counted: the stream never contains half a line from a full buffer. No float
 * printf (newlib-nano has it off): every value is a fixed-point integer.
 */
#include <stdint.h>
#include "ringbuf.h"

typedef struct {
    ringbuf_t rb;
    uint32_t  lines;            /* lines stored */
    uint32_t  lines_dropped;    /* lines rejected because the buffer was full */
    uint32_t  bytes_flushed;    /* bytes discarded on purpose (dump running, host stalled, no host) */
} usbstream_t;

/* storage_size must be a power of two. Returns 0 on success. */
int  usbstream_init(usbstream_t *s, uint8_t *storage, uint32_t storage_size);

void usbstream_imu (usbstream_t *s, uint64_t t_us, const int32_t xl_mg[3], const int32_t gy_mdps[3],
                    const int32_t *hg_mg /* NULL = omit */);
void usbstream_mag (usbstream_t *s, uint64_t t_us, const int32_t mag_mG[3]);
void usbstream_baro(usbstream_t *s, uint64_t t_us, int32_t pressure_pa, int32_t temp_centi_c);
/* Argument types match UBX-NAV-PVT: lat/lon 1e-7 deg, hMSL mm, velD mm/s, hAcc/vAcc mm.
 * An invalid accuracy (0xFFFFFFFF) is clamped to 999999.999 m for printing. */
void usbstream_gps (usbstream_t *s, uint64_t t_us, int fix, int sv, int32_t lat_e7, int32_t lon_e7,
                    int32_t hmsl_mm, int32_t vd_mmps, uint32_t hacc_mm, uint32_t vacc_mm);
/* A line that is already formatted (the 1 Hz-ish status line). Stored whole or dropped whole. */
void usbstream_text(usbstream_t *s, const char *line, uint32_t n);

uint32_t usbstream_pending(const usbstream_t *s);
uint32_t usbstream_peek(const usbstream_t *s, uint8_t *out, uint32_t max);   /* copy, do not consume */
void     usbstream_consume(usbstream_t *s, uint32_t n);                      /* call after the send was accepted */
void     usbstream_flush(usbstream_t *s);                                    /* discard everything buffered */

#endif
