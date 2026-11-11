#!/bin/sh
# Host tests for the UBX helpers and the GPS configuration sequence. No hardware needed.
set -e
cd "$(dirname "$0")/../.."
CC="gcc -std=gnu11 -Wall -Wextra -Werror -g -Iapp/core"
$CC test/gps/test_ubx.c app/core/ubx.c -o /tmp/atlas_test_ubx && /tmp/atlas_test_ubx
$CC test/gps/test_gps_cfg.c app/core/ubx.c app/core/gps_cfg.c -o /tmp/atlas_test_gps_cfg && /tmp/atlas_test_gps_cfg
