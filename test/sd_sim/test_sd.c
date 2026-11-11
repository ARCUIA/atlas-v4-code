#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "spi.h"
#include "sd_spi.h"
#include "sim_card.h"

int main(void)
{
    sd_info_t info; dev_err_t rc;

    sim_reset_card(1, 1, 3000);                     /* SDHC, v2 */
    rc = sd_init(&info);
    assert(rc == DEV_OK); assert(info.v2 == 1 && info.ccs == 1);
    assert(info.sectors == 3001u * 1024u); assert(info.cid[0] == 0x9A); assert(sim_cmd0_seen());
    assert(sd_selftest(1000) == DEV_OK);
    assert(sim_sector(1000)[0] == (uint8_t)(3u + 1000u));
    puts("SDHC v2 card: init + selftest OK");

    sim_reset_card(0, 1, 3719);                     /* standard capacity (~1.86 GB class), byte addressing */
    rc = sd_init(&info);
    assert(rc == DEV_OK); assert(info.v2 == 1 && info.ccs == 0);
    assert(info.sectors == 3720u * 1024u);          /* (C_SIZE+1) * 2^(MULT+2) * 2^10 bytes / 512 */
    assert(sd_selftest(1000) == DEV_OK);
    assert(sim_sector(1000)[5] == (uint8_t)(5u * 7u + 3u + 1000u));
    puts("SDSC v2 card (byte addressing): init + selftest OK");

    sim_reset_card(1, 1, 3000);                     /* non-blocking write: card stays busy for a while */
    assert(sd_init(&info) == DEV_OK);
    sim_set_busy_len(50);
    uint8_t buf[512]; memset(buf, 0x5A, sizeof buf);
    assert(sd_write_block_nb(7, buf) == DEV_OK);
    int polls = 0; while (!sd_ready()) polls++;
    assert(polls >= 40 && polls <= 60);              /* busy lasts about as long as the card says */
    assert(sim_sector(7)[100] == 0x5A);
    puts("Non-blocking write: busy/ready handshake OK");

    sim_reset_card(1, 1, 3000); sim_set_present(0);  /* nothing connected: MISO floats high */
    assert(sd_init(&info) == DEV_ERR_TIMEOUT);
    puts("No card: DEV_ERR_TIMEOUT");

    sim_reset_card(1, 1, 3000); hspi3.Init.NSS = 0;  /* hardware NSS left on in CubeMX */
    assert(sd_init(&info) == DEV_ERR_CFG);
    sim_reset_card(1, 1, 3000); sim_gpioa.MODER = 2u << 30;   /* PA15 still an alternate function */
    assert(sd_init(&info) == DEV_ERR_CFG);
    puts("CubeMX config guards: DEV_ERR_CFG");

    puts("ALL SD SIM TESTS PASSED");
    return 0;
}
