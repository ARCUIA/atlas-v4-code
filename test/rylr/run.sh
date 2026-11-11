#!/bin/sh
# Host tests for the RYLR998 link (protocol helpers, configuration state machine, send/receive) against simulated modules.
set -e
cd "$(dirname "$0")/../.."
gcc -std=gnu11 -Wall -Wextra -Werror -g -Iapp/core -Itest/rylr test/rylr/test_rylr.c test/rylr/sim_rylr.c \
    app/core/rylr.c app/core/rylr_link.c app/core/lora_airtime.c app/core/pkt.c app/core/gpsmsg.c app/core/ground_rx.c app/core/rx_format.c app/core/lcd_layout.c -o /tmp/atlas_test_rylr
/tmp/atlas_test_rylr
