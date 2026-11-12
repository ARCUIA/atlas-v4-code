#!/bin/sh
# Host tests for the USB text stream: formatter + ring, the USB glue (against a fake USB layer),
# and a round trip of the firmware's own lines through the viewer's parser.
set -e
cd "$(dirname "$0")/../.."
OUT=/tmp/atlas_usbstream_test; rm -rf $OUT; mkdir -p $OUT
CC="gcc -std=gnu11 -Wall -Wextra -Werror -g -Iapp/core"

$CC test/usbstream/test_usbstream.c app/core/usbstream.c app/core/ringbuf.c -o $OUT/test_usbstream
$OUT/test_usbstream

$CC -Itest/usbstream/stub -Iapp -Idevices -Idevices/imu -Idevices/baro \
    test/usbstream/test_glue.c app/usb_stream.c app/core/usbstream.c app/core/ringbuf.c -o $OUT/test_glue
$OUT/test_glue

$CC test/usbstream/dump_lines.c app/core/usbstream.c app/core/ringbuf.c -o $OUT/dump_lines
$OUT/dump_lines > $OUT/lines.txt
python3 test/usbstream/check_lines.py $OUT/lines.txt
echo "ALL USB STREAM TESTS PASSED"
