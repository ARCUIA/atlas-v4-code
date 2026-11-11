#ifndef GPS_CFG_H
#define GPS_CFG_H
/*
 * Brings a u-blox M10 receiver (SAM-M10Q) to a known state over UART: finds its baud rate,
 * moves it to the target baud, and applies the configuration to RAM. HAL-free: all hardware
 * access goes through gps_io_t, so the sequence is tested on the PC against a simulated receiver.
 *
 * Settings go to the receiver's RAM only (they last until it loses power) and are re-applied at every
 * boot, so nothing is written to its flash or OTP memory.
 */
#include <stdint.h>

typedef struct {
    void     (*set_baud)(uint32_t baud);
    void     (*write)(const uint8_t *p, uint16_t n);     /* blocking send */
    int      (*read_byte)(uint8_t *b);                   /* non-blocking: 1 if a byte was returned */
    uint32_t (*now_ms)(void);
    void     (*delay_ms)(uint32_t ms);
} gps_io_t;

typedef struct {
    uint32_t target_baud;       /* e.g. 115200 */
    uint16_t meas_period_ms;    /* CFG-RATE-MEAS; 56 ms is about 17.9 Hz */
    uint8_t  gps_only;          /* 1: GPS only (needed for the highest rates), 0: leave other constellations alone */
    uint8_t  dynmodel;          /* CFG-NAVSPG-DYNMODEL; 8 = airborne <4 g */
} gps_cfg_t;

typedef struct {
    uint32_t baud_found;        /* baud rate the receiver answered at */
    uint32_t baud_final;
    int      code;              /* one of GPSCFG_* */
} gps_cfg_result_t;

enum {
    GPSCFG_OK          =  0,
    GPSCFG_NO_RESPONSE = -1,    /* nothing answered at any baud rate: wiring, power, or wrong UART */
    GPSCFG_BAUD_FAIL   = -2,    /* receiver answered, but not at the new baud after the switch */
    GPSCFG_NAK         = -3,    /* receiver rejected the configuration */
    GPSCFG_NO_ACK      = -4,    /* configuration sent, no acknowledgement */
    GPSCFG_NO_PVT      = -5     /* configured, but no position messages arrived (yet) */
};

int gps_configure(const gps_io_t *io, const gps_cfg_t *cfg, gps_cfg_result_t *res);
const char *gps_cfg_str(int code);      /* short name for a GPSCFG_* code, for status lines */

#endif
