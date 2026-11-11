#!/bin/sh
# Host test of the ground-station display logic (no hardware needed).
set -e
cd "$(dirname "$0")/../.."
gcc -std=gnu11 -Wall -Wextra -Werror -g -Iapp/core test/ground/test_lcd_layout.c app/core/lcd_layout.c app/core/pkt.c -o /tmp/atlas_test_lcd
/tmp/atlas_test_lcd
