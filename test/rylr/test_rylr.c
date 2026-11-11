#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "rylr.h"
#include "rylr_link.h"
#include "lora_airtime.h"
#include "pkt.h"
#include "sim_rylr.h"
#include "ground_rx.h"
#include "gpsmsg.h"
#include "lcd_layout.h"

/* ---- two links (flight = A, ground = B) wired to their simulated modules ---- */
static sim_mod_t MA, MB;
static uint32_t tx_busy_a, tx_busy_b;
#define MAKE_IO(N, MOD, BUSY)                                                                         \
    static int tx_start_##N(const uint8_t *p, uint16_t n) { sim_host_write(&MOD, p, n); BUSY = sim_now_ms + (uint32_t)n * 10u * 1000u / 115200u + 1u; return 0; } \
    static int tx_idle_##N(void) { return sim_now_ms >= BUSY; }                                      \
    static int read_##N(uint8_t *b) { return sim_host_read(&MOD, b); }
MAKE_IO(a, MA, tx_busy_a)
MAKE_IO(b, MB, tx_busy_b)
static uint32_t now_ms(void) { return sim_now_ms; }
static void delay_ms(uint32_t ms) { sim_now_ms += ms; }
static const rylr_io_t IO_A = { tx_start_a, tx_idle_a, read_a, now_ms, delay_ms };
static const rylr_io_t IO_B = { tx_start_b, tx_idle_b, read_b, now_ms, delay_ms };

/* flight settings (devices/radio/radio.h) and ground settings (ground sketch): same RF, mirrored addresses */
static const rylr_settings_t CFG_A = { 1, 2, 6, 915000000u, 11, 9, 1, 12, 22 };
static const rylr_settings_t CFG_B = { 2, 1, 6, 915000000u, 11, 9, 1, 12, 22 };

static uint8_t got[128]; static size_t got_n; static int got_rssi, got_snr, got_count; static uint16_t got_addr;
static void on_rcv(void *ctx, uint16_t addr, const char *hex, uint16_t hexlen, int rssi, int snr)
{
    (void)ctx; got_n = rylr_hex_decode(hex, hexlen, got, sizeof got); got_rssi = rssi; got_snr = snr; got_addr = addr; got_count++;
}

static void reset_world(void) { sim_now_ms = 5000; sim_mod_init(&MA); sim_mod_init(&MB); sim_pair(&MA, &MB); tx_busy_a = tx_busy_b = 0; got_n = 0; got_count = 0; }
static void run(rylr_link_t *a, rylr_link_t *b, uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { sim_now_ms++; if (a) rylr_link_poll(a, sim_now_ms); if (b) rylr_link_poll(b, sim_now_ms); } }


/* The ground receiver's parsing, copied from Reserve_GPS_Reciever_Code_Works.ino: same strtok sequence, same 150-byte buffer.
 * Returns 2 for "YG" (lat/lon filled in), 1 for "NG", negative for what that sketch prints as an error. */
static int teammate_parse(const char *line, char *connection, char *lat, char *lon)
{
    char dataArray[150];
    if (strlen(line) >= sizeof dataArray) return -1;                 /* the Arduino code would truncate: treat as a failure */
    strcpy(dataArray, line);
    char *data = strtok(dataArray, ",");
    if (data == NULL || strncmp(data, "+RCV=", 5) != 0) return -2;
    data = strtok(NULL, ",");                                        /* payload length */
    if (data == NULL) return -3;
    char *conn = strtok(NULL, ",");                                  /* GPS status: NG or YG */
    if (conn == NULL) return -4;
    strcpy(connection, conn);
    if (strcmp(conn, "NG") == 0) return 1;
    if (strcmp(conn, "YG") == 0) {
        char *la = strtok(NULL, ","), *lo = strtok(NULL, ",");
        if (la == NULL || lo == NULL) return -5;
        strcpy(lat, la); strcpy(lon, lo);
        return 2;
    }
    return -6;                                                       /* "Unknown GPS status" */
}
/* everything the module printed to the receiver's serial port so far, as one string */
static void module_output(sim_mod_t *m, char *out, size_t cap)
{
    size_t n = 0; uint8_t b;
    while (n + 1 < cap && sim_host_read(m, &b)) out[n++] = (char)b;
    out[n] = '\0';
}

static size_t make_packet(uint8_t *buf, uint16_t seq)
{
    pkt_builder_t b; pkt_gps_t g = { .fix_type = 3, .num_sv = 9, .lat_e7 = 416600000, .lon_e7 = -916500000, .alt_dm = 2155, .gspeed_dms = 123, .hacc_m = 2 };
    pkt_begin(&b, buf, 64, 1, seq); pkt_add_gps(&b, &g); return pkt_end(&b);
}

int main(void)
{
    /* ---------------- text protocol helpers ---------------- */
    rylr_lines_t L; rylr_lines_init(&L); const char *in = "\r\n+OK\r\n+ERR=17\r\n+RCV=50,5,HE,LO,-99,40\r\n";
    int lines = 0; char seen[4][64];
    for (const char *p = in; *p; p++) if (rylr_lines_feed(&L, (uint8_t)*p)) { strncpy(seen[lines++], L.buf, 63); }
    assert(lines == 3 && strcmp(seen[0], "+OK") == 0 && strcmp(seen[1], "+ERR=17") == 0);
    rylr_msg_t m;
    rylr_parse_line("+OK", &m);                    assert(m.kind == RYLR_MSG_OK);
    rylr_parse_line("+ERR=17", &m);                assert(m.kind == RYLR_MSG_ERR && m.err == 17);
    rylr_parse_line("+READY", &m);                 assert(m.kind == RYLR_MSG_READY);
    rylr_parse_line("+PARAMETER=11,9,1,12", &m);   assert(m.kind == RYLR_MSG_VALUE && !strcmp(m.key, "PARAMETER") && !strcmp(m.value, "11,9,1,12"));
    rylr_parse_line("+RCV=50,5,HE,LO,-99,40", &m); assert(m.kind == RYLR_MSG_RCV && m.addr == 50 && m.len == 5 && !strncmp(m.data, "HE,LO", 5) && m.rssi == -99 && m.snr == 40);
    rylr_parse_line("+RCV=50,9,HE,LO,-99,40", &m); assert(m.kind == RYLR_MSG_OTHER);          /* stated length does not match */
    uint8_t bin[24] = {0xA7, 0x10, 0x01, 0x02, 0x01, 0xFF, 0x00}; char hex[64]; uint8_t back[24];
    assert(rylr_hex_encode(bin, 24, hex, sizeof hex) == 48 && rylr_hex_decode(hex, 48, back, sizeof back) == 24 && !memcmp(bin, back, 24));
    assert(rylr_hex_decode("ABC", 3, back, sizeof back) == 0 && rylr_hex_decode("ZZ", 2, back, sizeof back) == 0);
    char cmd[400]; size_t n = rylr_build_send(cmd, sizeof cmd, 2, bin, 24);
    assert(n == strlen(cmd) && !strncmp(cmd, "AT+SEND=2,48,A71001020", 22) && !strcmp(cmd + n - 2, "\r\n"));
    uint8_t big[121] = {0}; assert(rylr_build_send(cmd, sizeof cmd, 2, big, 120) > 0 && rylr_build_send(cmd, sizeof cmd, 2, big, 121) == 0);
    puts("text protocol: line splitting, parsing (commas inside data, length check), hex, AT+SEND format and 240-byte limit: OK");

    uint32_t air = lora_airtime_us(11, 500000, 5, 12, 56, 1, 0) / 1000;
    printf("airtime SF11/500 kHz, 24-byte packet as hex (+8 module bytes): %u ms\n", (unsigned)air);
    assert(air >= 320 && air <= 330);

    /* ---------------- fresh module gets configured, and only what differs is written ---------------- */
    reset_world();
    rylr_link_t A, B;
    assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK);
    assert(MA.address == 1 && MA.network == 6 && MA.band == 915000000u && MA.sf == 11 && MA.bw == 9 && MA.cr == 1 && MA.pre == 12 && MA.crfop == 22);
    assert(A.set_commands == 3 && MA.flash_writes == 2);          /* address, network and parameters differed from the defaults */
    printf("fresh module configured: %u settings written, %u flash writes (band and power already matched): OK\n", (unsigned)A.set_commands, MA.flash_writes);
    unsigned fw = MA.flash_writes;
    assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK && A.set_commands == 0 && MA.flash_writes == fw);
    puts("restart with a configured module: nothing rewritten (no flash wear): OK");

    /* ---------------- packet from the flight board to the ground station ---------------- */
    reset_world();
    assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK && rylr_link_init(&B, &IO_B, &CFG_B) == RYLR_OK);
    rylr_link_set_rcv(&B, on_rcv, 0);
    uint8_t pk[64]; size_t pn = make_packet(pk, 258);
    assert(rylr_link_send(&A, pk, pn, sim_now_ms) == 0 && rylr_link_busy(&A));
    assert(rylr_link_send(&A, pk, pn, sim_now_ms) == -1);                              /* one at a time */
    run(&A, &B, 800);
    assert(A.tx_ok == 1 && A.tx_err == 0 && !rylr_link_busy(&A));
    assert(got_count == 1 && got_n == pn && !memcmp(got, pk, pn) && got_addr == 1 && got_rssi == -90 && got_snr == 10);
    pkt_header_t h; assert(pkt_parse_header(got, got_n, &h) == 0 && h.seq == 258);
    puts("packet crosses the simulated air and arrives byte-for-byte, with RSSI/SNR: OK");

    {   /* the ground station's handler, fed the same +RCV text the module would print */
        lcd_state_t ls; lcd_state_init(&ls); char line[240], hexs[80];
        rylr_hex_encode(pk, pn, hexs, sizeof hexs);
        assert(ground_rx_handle(&ls, hexs, (uint16_t)(2 * pn), -97, 8, 1000, line, sizeof line) == GROUND_RX_OK);
        assert(strstr(line, "seq=258") && strstr(line, "lat=41.6600000") && strstr(line, "lon=-91.6500000") && strstr(line, "rssi=-97") && strstr(line, "rx=1"));
        assert(ls.fix == 3 && ls.sv == 9 && ls.lat_e7 == 416600000 && ls.lon_e7 == -916500000 && ls.alt_dm == 2155);
        assert(ground_rx_handle(&ls, "ZZ", 2, 0, 0, 0, line, sizeof line) == GROUND_RX_NOT_HEX);
        assert(ground_rx_handle(&ls, "0102030405", 10, 0, 0, 0, line, sizeof line) == GROUND_RX_NOT_ATLAS);
        char rows[LCD_MAX_ROWS][LCD_MAX_COLS + 1]; lcd_render(&ls, 1500, 16, 2, rows);
        assert(strncmp(rows[0], "LAT 41.660000", 13) == 0 && strncmp(rows[1], "LON -91.650000", 14) == 0);
        printf("ground side: %s\n", line);
        puts("ground handler: decode, display state and serial line; non-hex and foreign packets rejected: OK");
    }

    /* one packet per second for a minute, both ways of answering AT+SEND */
    for (int ok_after_air = 0; ok_after_air < 2; ok_after_air++) {
        reset_world(); MA.ok_after_air = ok_after_air;
        assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK && rylr_link_init(&B, &IO_B, &CFG_B) == RYLR_OK);
        rylr_link_set_rcv(&B, on_rcv, 0);
        int started = 0, refused = 0;
        for (int sec = 0; sec < 60; sec++) {
            pn = make_packet(pk, (uint16_t)sec);
            if (rylr_link_send(&A, pk, pn, sim_now_ms) == 0) started++; else refused++;
            run(&A, &B, 1000);
        }
        assert(started == 60 && refused == 0 && A.tx_ok == 60 && A.tx_err == 0 && A.tx_timeout == 0 && MA.errors_sent == 0 && got_count == 60);
    }
    puts("60 s at 1 packet/s: all sent, all received, never a +ERR=17 (+OK immediate or after airtime): OK");

    /* sending again before the last transmission ended is refused by us, so the module never says +ERR=17 */
    reset_world(); assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK);
    pn = make_packet(pk, 1); assert(rylr_link_send(&A, pk, pn, sim_now_ms) == 0); run(&A, 0, 20);
    assert(rylr_link_send(&A, pk, pn, sim_now_ms) == -1);
    puts("too-early send refused locally: OK");

    /* ---------------- failure modes ---------------- */
    reset_world(); MA.dead = 1; uint32_t t0 = sim_now_ms;
    assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_ERR_NO_RESPONSE && sim_now_ms - t0 < 3000);
    printf("no module answering: NO_RESPONSE after %u ms: OK\n", (unsigned)(sim_now_ms - t0));
    reset_world(); MA.garbage = 1;
    assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_ERR_BAD_REPLY);
    puts("module sending only noise: BAD_REPLY: OK");
    reset_world(); rylr_settings_t bad = CFG_A; bad.bw_code = 7;                       /* SF11 at 125 kHz is not allowed by the module */
    assert(rylr_link_init(&A, &IO_A, &bad) == RYLR_ERR_REJECTED - 4);
    puts("settings the module forbids (SF11 at 125 kHz): REJECTED with its error number: OK");
    reset_world(); bad = CFG_A; bad.network_id = 7; bad.preamble = 20;                 /* programmable preamble only exists on network 18 */
    assert(rylr_link_init(&A, &IO_A, &bad) == RYLR_ERR_REJECTED - 18);
    puts("preamble other than 12 on a non-default network: REJECTED(+ERR=18): OK");

    /* ---------------- module reboots in the middle of operation ---------------- */
    reset_world(); assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK && rylr_link_init(&B, &IO_B, &CFG_B) == RYLR_OK);
    rylr_link_set_rcv(&B, on_rcv, 0);
    pn = make_packet(pk, 1); rylr_link_send(&A, pk, pn, sim_now_ms); run(&A, &B, 900);
    sim_mod_reboot(&MA);                                                               /* brown-out: SF/BW/band/power revert to defaults */
    assert(MA.sf == 9 && MA.bw == 7);
    run(&A, &B, 1500);
    assert(A.reconfigs == 1 && A.state == RL_READY && MA.sf == 11 && MA.bw == 9 && MA.band == 915000000u);
    got_count = 0; pn = make_packet(pk, 2);
    assert(rylr_link_send(&A, pk, pn, sim_now_ms) == 0); run(&A, &B, 800);
    assert(got_count == 1);
    puts("module reboot mid-flight: detected, settings re-applied by itself, link works again: OK");

    /* the module stops answering AT+SEND: the link does not get stuck */
    reset_world(); assert(rylr_link_init(&A, &IO_A, &CFG_A) == RYLR_OK);
    MA.dead = 1; pn = make_packet(pk, 1); assert(rylr_link_send(&A, pk, pn, sim_now_ms) == 0);
    run(&A, 0, 3500);
    assert(A.tx_timeout == 1 && !rylr_link_busy(&A));
    puts("lost reply to AT+SEND: counted as a timeout, link keeps going: OK");

    /* ---------------- the message format ---------------- */
    char msg[48]; gpsmsg_in_t gi = { 1, 1, 416600123, -916504560 };
    assert(gpsmsg_format(msg, sizeof msg, &gi) == strlen("YG,41.660012,-91.650456") && !strcmp(msg, "YG,41.660012,-91.650456"));
    gi.lat_e7 = 416600125; gi.lon_e7 = -5; gpsmsg_format(msg, sizeof msg, &gi);   assert(!strcmp(msg, "YG,41.660013,-0.000001"));
    gi.lat_e7 = 899999999; gi.lon_e7 = -1799999999; gpsmsg_format(msg, sizeof msg, &gi);  assert(!strcmp(msg, "YG,90.000000,-180.000000"));
    gi.fix_ok = 0; gi.lat_e7 = 416600123; gi.lon_e7 = -916504560; gpsmsg_format(msg, sizeof msg, &gi); assert(!strcmp(msg, "NG,41.660012,-91.650456"));
    gi.have_last = 0; assert(gpsmsg_format(msg, sizeof msg, &gi) == 2 && !strcmp(msg, "NG"));
    gi.fix_ok = 1; gi.have_last = 1; assert(gpsmsg_format(msg, 8, &gi) == 0);                                  /* does not fit: reports it */
    puts("message format: YG,lat,lon / NG[,lat,lon], rounding to 1e-6 degrees, west and south, overflow: OK");

    /* ---------------- the teammate's receiver ----------------
     * Its sketch never configures its module: that module is set to network 6, address 1 (RF parameters at factory
     * defaults). Our module starts at factory defaults and must end up on network 6, address 0, sending to address 1. */
    reset_world();
    MB.network = 6; MB.address = 1;
    const rylr_settings_t CFG_FLIGHT = { 0, 1, 6, 915000000u, 9, 7, 1, 12, 22 };            /* = devices/radio/radio.h */
    assert(rylr_link_init(&A, &IO_A, &CFG_FLIGHT) == RYLR_OK);
    assert(MA.network == 6 && MA.address == 0 && MA.band == 915000000u);
    assert(A.set_commands == 1 && MA.flash_writes == 1);                                      /* only the network id differed */
    puts("flight module from factory defaults to network 6 / address 0: one setting written: OK");
    assert(rylr_link_init(&A, &IO_A, &CFG_FLIGHT) == RYLR_OK && A.set_commands == 0 && MA.flash_writes == 1);
    puts("flight module already configured: nothing written: OK");
    reset_world(); MB.network = 6; MB.address = 1; MA.network = 6;                            /* control: if our network were left at 18, nobody hears us */
    MA.network = 18; {
        rylr_settings_t keep_net = CFG_FLIGHT; keep_net.network_id = 18;
        assert(rylr_link_init(&A, &IO_A, &keep_net) == RYLR_OK);
        assert(rylr_link_send_text(&A, "YG,1,2", 6, sim_now_ms) == 0); run(&A, 0, 800);
        char o[200]; module_output(&MB, o, sizeof o); assert(strstr(o, "+RCV") == NULL);
    }
    puts("control: a flight module left on the wrong network is not heard: OK");
    reset_world(); MB.network = 6; MB.address = 1;
    assert(rylr_link_init(&A, &IO_A, &CFG_FLIGHT) == RYLR_OK);
    char out[600], con[16], lat[24], lon[24];
    const char *cases[] = { "YG,41.660012,-91.650456", "YG,-33.700001,151.800000", "NG,41.660012,-91.650456", "NG" };
    const int  want[]   = { 2, 2, 1, 1 };
    for (int i = 0; i < 4; i++) {
        assert(rylr_link_send_text(&A, cases[i], strlen(cases[i]), sim_now_ms) == 0);
        run(&A, 0, 1000);
        assert(A.tx_ok == (uint32_t)i + 1 && A.tx_err == 0);
        module_output(&MB, out, sizeof out);                                  /* what the receiver's serial port gets */
        char *cr = strstr(out, "\r\n"); assert(cr); *cr = '\0';
        assert(strncmp(out, "+RCV=0,", 7) == 0);                                 /* sender address 0 */
        printf("receiver line: %s\n", out);
        assert(teammate_parse(out, con, lat, lon) == want[i]);
        if (want[i] == 2) { assert(strncmp(cases[i], "YG,", 3) == 0 && !strncmp(cases[i] + 3, lat, strlen(lat)) && strstr(cases[i], lon)); }
    }
    printf("flight packet airtime (default SF9, 125 kHz): %u ms\n", (unsigned)(A.last_airtime_us / 1000));
    assert(A.last_airtime_us / 1000 >= 100 && A.last_airtime_us / 1000 <= 400);
    puts("the teammate's receiver parser reads our messages (YG lat/lon, NG with and without last position): OK");
    assert(rylr_link_send_text(&A, "bad\r\ntext", 9, sim_now_ms + 5000) == -1);
    assert(rylr_link_send_text(&A, "caf\xc3\xa9", 5, sim_now_ms + 5000) == -1);
    puts("text with CR/LF or non-ASCII bytes refused before it reaches the module: OK");
    {   /* sending to address 0 (broadcast) reaches the receiver too, and sending to some other address does not */
        rylr_settings_t bc = CFG_FLIGHT; bc.dest_addr = 0;
        reset_world(); MB.network = 6; MB.address = 1;
        assert(rylr_link_init(&A, &IO_A, &bc) == RYLR_OK && rylr_link_send_text(&A, "YG,1,2", 6, sim_now_ms) == 0); run(&A, 0, 800);
        char o[200]; module_output(&MB, o, sizeof o); assert(strstr(o, "+RCV=0,6,YG,1,2,"));
        rylr_settings_t other = CFG_FLIGHT; other.dest_addr = 7;
        reset_world(); MB.network = 6; MB.address = 1;
        assert(rylr_link_init(&A, &IO_A, &other) == RYLR_OK && rylr_link_send_text(&A, "YG,1,2", 6, sim_now_ms) == 0); run(&A, 0, 800);
        module_output(&MB, o, sizeof o); assert(strstr(o, "+RCV") == NULL);
        puts("destination: address 1 and broadcast (0) reach the receiver, another address does not: OK");
    }
    {   char longest[48]; gi.fix_ok = 1; gi.have_last = 1; gi.lat_e7 = 899999999; gi.lon_e7 = -1799999999;
        size_t ln = gpsmsg_format(longest, sizeof longest, &gi);
        char worst[200]; snprintf(worst, sizeof worst, "+RCV=65535,%u,%s,-120,-20", (unsigned)ln, longest);
        assert(strlen(worst) < 150 && teammate_parse(worst, con, lat, lon) == 2);
        puts("longest possible line stays inside the receiver's 150-byte buffer: OK");
    }

    puts("ALL RYLR LINK TESTS PASSED");
    return 0;
}
