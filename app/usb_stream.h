#ifndef USB_STREAM_H
#define USB_STREAM_H
/*
 * USB CDC streaming for the live viewer (tools/atlas_ekf/atlas_live.py), target side.
 *
 * Everything that leaves over USB for display goes through ONE ring buffer and ONE transmit
 * path, so nothing can overwrite a buffer the USB core is still reading:
 *   - timestamped sample lines (IMU / BARO / GPS, MAG when it exists)
 *   - the human-readable status line from main.c
 *
 * The stream stays silent (and discards what it holds) while
 *   - the dump protocol is active or just ended (dump_quiet() / dump_busy()),
 *   - the host has not enumerated the device,
 *   - the host is not reading (a transfer has been stuck for USB_STREAM_STALL_MS).
 * so it cannot corrupt a dump, and a viewer that connects late never receives stale data.
 *
 * Call everything from the main loop (single context). Nothing here blocks.
 */
#include <stdint.h>
#include "imu.h"
#include "baro.h"
#include "ubx.h"
#include "usbstream.h"

#define USB_STREAM_RING_BYTES  16384u   /* power of two; ~0.5 s of 455 Hz IMU lines */
#define USB_STREAM_TX_CHUNK      512u   /* bytes handed to the USB core per transfer */
#define USB_STREAM_STALL_MS      100u   /* a transfer stuck this long = host is not reading */
#define USB_STREAM_IMU_DIV         1u   /* 1 = every IMU sample, 2 = every other one, ... */
#define USB_STREAM_IMU_HG          0    /* 1 = also print hg[mg]= (high-g accel; scale not verified) */

void usb_stream_init(void);
void usb_stream_poll(void);              /* every main-loop pass, after dump_poll() */
int  usb_stream_active(void);            /* 1 if lines are currently being sent */

void usb_stream_imu (uint64_t t_us, const imu_raw_t *s);
void usb_stream_baro(uint64_t t_us, const baro_sample_t *b);
void usb_stream_gps (uint64_t t_us, const ubx_pvt_t *p);
void usb_stream_mag (uint64_t t_us, const int32_t mag_mG[3]);   /* for the LIS2MDL driver, when it exists */
void usb_stream_text(const char *line, int n);                  /* an already formatted line incl. \r\n */

const usbstream_t *usb_stream_stats(void);   /* lines / lines_dropped / bytes_flushed */

#endif
