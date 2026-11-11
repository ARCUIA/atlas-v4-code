#ifndef LORA_AIRTIME_H
#define LORA_AIRTIME_H
#include <stdint.h>
/* LoRa time on air (the standard formula), integer maths. cr_den: 5..8 for coding rate 4/5..4/8. */
uint32_t lora_airtime_us(uint8_t sf, uint32_t bw_hz, uint8_t cr_den, uint16_t preamble,
                         uint16_t payload_bytes, int crc_on, int implicit_header);
#endif
