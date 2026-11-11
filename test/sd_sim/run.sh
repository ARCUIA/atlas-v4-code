#!/bin/sh
# Host-side test of devices/sd/sd_spi.c against a simulated SD card (no hardware needed).
set -e
cd "$(dirname "$0")/../.."
gcc -std=gnu11 -Wall -Wextra -Werror -g -Itest/sd_sim/stub -Itest/sd_sim -Idevices -Idevices/sd \
    test/sd_sim/test_sd.c test/sd_sim/sim_card.c devices/sd/sd_spi.c -o /tmp/atlas_sd_sim
/tmp/atlas_sd_sim
