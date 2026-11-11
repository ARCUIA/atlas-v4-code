#include "lora_airtime.h"

uint32_t lora_airtime_us(uint8_t sf, uint32_t bw_hz, uint8_t cr_den, uint16_t preamble,
                         uint16_t payload_bytes, int crc_on, int implicit_header)
{
    uint64_t tsym_ns = ((1ull << sf) * 1000000000ull) / bw_hz;
    int de  = (tsym_ns > 16000000ull) ? 1 : 0;                 /* low data rate optimisation */
    int ih  = implicit_header ? 1 : 0;
    int num = 8 * (int)payload_bytes - 4 * (int)sf + 28 + (crc_on ? 16 : 0) - 20 * ih;
    int den = 4 * ((int)sf - 2 * de);
    int npay = 8 + (num > 0 ? ((num + den - 1) / den) * (int)cr_den : 0);
    uint64_t quarter_symbols = (uint64_t)preamble * 4u + 17u + (uint64_t)npay * 4u;   /* preamble + 4.25 + payload */
    return (uint32_t)((quarter_symbols * tsym_ns / 4u) / 1000u);
}
