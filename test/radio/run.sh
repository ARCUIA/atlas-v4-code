#!/bin/sh
# Host tests for the telemetry packet format, scheduler, battery maths, ground-station formatter and Python decoder.
set -e
cd "$(dirname "$0")/../.."
OUT=/tmp/atlas_radio_test; rm -rf $OUT; mkdir -p $OUT
CC="gcc -std=gnu11 -Wall -Wextra -Werror -g -Iapp/core"
$CC test/radio/test_pkt.c app/core/pkt.c app/core/rx_format.c -o $OUT/test_pkt
(cd $OUT && ./test_pkt)
$CC test/radio/test_telemetry.c app/core/pkt.c app/core/telemetry.c -o $OUT/test_telemetry && $OUT/test_telemetry
PYTHONPATH=tools python3 test/radio/check_vectors.py $OUT/vectors.txt
echo "ALL RADIO HOST TESTS PASSED"
