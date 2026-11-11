#ifndef SIM_CARD_H
#define SIM_CARD_H
#include <stdint.h>
/* Simulated SD card in SPI mode for host tests (sdhc: block addressing; c_size: CSD C_SIZE field). */
void     sim_reset_card(int sdhc, int v2, uint32_t c_size);
void     sim_set_busy_len(uint32_t n);      /* how many byte-clocks the card stays busy after each write */
void     sim_set_present(int present);
int      sim_cmd0_seen(void);
uint8_t *sim_sector(uint32_t sector);       /* direct access to the stored data (sectors < 1100) */
#endif
