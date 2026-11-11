#include "gps_cfg.h"
#include "ubx.h"
#include <string.h>

/* Configuration keys, from u-blox's published definitions (u-blox M10 SPG 5.10 interface description). */
#define KEY_UART1_BAUDRATE     0x40520001u   /* U4 */
#define KEY_UART1INPROT_UBX    0x10730001u   /* L  */
#define KEY_UART1INPROT_NMEA   0x10730002u
#define KEY_UART1OUTPROT_UBX   0x10740001u
#define KEY_UART1OUTPROT_NMEA  0x10740002u
#define KEY_RATE_MEAS          0x30210001u   /* U2, ms */
#define KEY_RATE_NAV           0x30210002u   /* U2, measurements per solution */
#define KEY_MSGOUT_NAV_PVT_U1  0x20910007u   /* U1, output every N solutions */
#define KEY_NAVSPG_DYNMODEL    0x20110021u   /* E1 */
#define KEY_SIGNAL_GPS_ENA     0x1031001fu   /* L  */
#define KEY_SIGNAL_SBAS_ENA    0x10310020u
#define KEY_SIGNAL_GAL_ENA     0x10310021u
#define KEY_SIGNAL_BDS_ENA     0x10310022u
#define KEY_SIGNAL_QZSS_ENA    0x10310024u
#define KEY_SIGNAL_GLO_ENA     0x10310025u

#define LAYER_RAM 0x01u

static void send_frame(const gps_io_t *io, uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
    uint8_t f[UBX_MAX_PAYLOAD + 8];
    size_t n = ubx_build(f, cls, id, pl, len);
    io->write(f, (uint16_t)n);
}

static void drain(const gps_io_t *io, uint32_t ms)
{
    uint8_t b;
    uint32_t t0 = io->now_ms();
    while (io->now_ms() - t0 < ms) {
        while (io->read_byte(&b)) { }
        io->delay_ms(1);
    }
}

/* Poll for a position message: any valid UBX-NAV-PVT frame proves the link works at this baud rate. */
static int ping(const gps_io_t *io, ubx_parser_t *p, uint32_t wait_ms)
{
    uint8_t b;
    send_frame(io, UBX_CLS_NAV, UBX_ID_PVT, 0, 0);
    uint32_t t0 = io->now_ms();
    while (io->now_ms() - t0 < wait_ms) {
        while (io->read_byte(&b))
            if (ubx_parser_feed(p, b) && p->cls == UBX_CLS_NAV && p->id == UBX_ID_PVT) return 1;
        io->delay_ms(1);
    }
    return 0;
}

/* Returns 1 = ACK, 0 = NAK, -1 = timeout. */
static int wait_ack(const gps_io_t *io, ubx_parser_t *p, uint8_t cls, uint8_t id, uint32_t wait_ms)
{
    uint8_t b;
    uint32_t t0 = io->now_ms();
    while (io->now_ms() - t0 < wait_ms) {
        while (io->read_byte(&b)) {
            if (ubx_parser_feed(p, b) && p->cls == UBX_CLS_ACK && p->len == 2 && p->payload[0] == cls && p->payload[1] == id)
                return p->id == UBX_ID_ACK ? 1 : 0;
        }
        io->delay_ms(1);
    }
    return -1;
}

int gps_configure(const gps_io_t *io, const gps_cfg_t *cfg, gps_cfg_result_t *res)
{
    ubx_parser_t p;
    ubx_valset_t v;
    uint32_t cand[3] = { cfg->target_baud, 38400u, 9600u };
    int found = -1;

    memset(res, 0, sizeof *res);
    ubx_parser_init(&p);

    /* 1. Find the baud rate the receiver is using now. Its RAM config survives an MCU reset, so it may
     *    already be at the target rate; a factory-fresh receiver uses 38400 (older ones 9600). */
    for (int i = 0; i < 3 && found < 0; i++) {
        if (i > 0 && cand[i] == cfg->target_baud) continue;
        io->set_baud(cand[i]);
        drain(io, 30);
        for (int t = 0; t < 3; t++)
            if (ping(io, &p, 200)) { found = i; break; }
    }
    if (found < 0) { res->code = GPSCFG_NO_RESPONSE; return res->code; }
    res->baud_found = cand[found];
    res->baud_final = cand[found];

    /* 2. Move it to the target baud rate. The ACK can arrive at either rate, so it is not waited for:
     *    the proof is that it answers at the new rate. */
    if (cand[found] != cfg->target_baud) {
        ubx_valset_begin(&v, LAYER_RAM);
        ubx_valset_add(&v, KEY_UART1_BAUDRATE, cfg->target_baud);
        send_frame(io, UBX_CLS_CFG, UBX_ID_VALSET, v.buf, v.len);
        io->delay_ms(100);
        io->set_baud(cfg->target_baud);
        drain(io, 30);
        int ok = 0;
        for (int t = 0; t < 5 && !ok; t++) ok = ping(io, &p, 300);
        if (!ok) { res->code = GPSCFG_BAUD_FAIL; return res->code; }
        res->baud_final = cfg->target_baud;
    }

    /* 3. Apply the configuration (RAM only). */
    ubx_valset_begin(&v, LAYER_RAM);
    ubx_valset_add(&v, KEY_UART1INPROT_UBX,   1);
    ubx_valset_add(&v, KEY_UART1OUTPROT_UBX,  1);
    ubx_valset_add(&v, KEY_UART1OUTPROT_NMEA, 0);                 /* UBX only: less traffic, nothing to skip */
    ubx_valset_add(&v, KEY_RATE_MEAS, cfg->meas_period_ms);
    ubx_valset_add(&v, KEY_RATE_NAV, 1);
    ubx_valset_add(&v, KEY_MSGOUT_NAV_PVT_U1, 1);                 /* a NAV-PVT with every solution */
    ubx_valset_add(&v, KEY_NAVSPG_DYNMODEL, cfg->dynmodel);
    if (cfg->gps_only) {
        ubx_valset_add(&v, KEY_SIGNAL_GPS_ENA,  1);
        ubx_valset_add(&v, KEY_SIGNAL_GAL_ENA,  0);
        ubx_valset_add(&v, KEY_SIGNAL_BDS_ENA,  0);
        ubx_valset_add(&v, KEY_SIGNAL_GLO_ENA,  0);
        ubx_valset_add(&v, KEY_SIGNAL_QZSS_ENA, 0);
        ubx_valset_add(&v, KEY_SIGNAL_SBAS_ENA, 0);
    }
    if (v.overflow) { res->code = GPSCFG_NO_ACK; return res->code; }
    send_frame(io, UBX_CLS_CFG, UBX_ID_VALSET, v.buf, v.len);
    int a = wait_ack(io, &p, UBX_CLS_CFG, UBX_ID_VALSET, 1000);
    if (a == 0) { res->code = GPSCFG_NAK;    return res->code; }
    if (a < 0)  { res->code = GPSCFG_NO_ACK; return res->code; }

    /* 4. Confirm that position messages are flowing at the new rate. */
    uint8_t b; int seen = 0;
    uint32_t t0 = io->now_ms();
    while (io->now_ms() - t0 < 3000 && seen < 2) {
        while (io->read_byte(&b))
            if (ubx_parser_feed(&p, b) && p.cls == UBX_CLS_NAV && p.id == UBX_ID_PVT) seen++;
        io->delay_ms(1);
    }
    res->code = (seen >= 2) ? GPSCFG_OK : GPSCFG_NO_PVT;
    return res->code;
}

const char *gps_cfg_str(int code)
{
    switch (code) {
    case GPSCFG_OK:          return "OK";
    case GPSCFG_NO_RESPONSE: return "NO-RESP";
    case GPSCFG_BAUD_FAIL:   return "BAUD";
    case GPSCFG_NAK:         return "NAK";
    case GPSCFG_NO_ACK:      return "NO-ACK";
    case GPSCFG_NO_PVT:      return "NO-PVT";
    default:                 return "?";
    }
}
