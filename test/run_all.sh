#!/bin/sh
# Runs every host-side test: SD driver, logger + Python tools, UBX/GPS configuration, USB stream, radio packets.
set -e
cd "$(dirname "$0")/.."
./test/sd_sim/run.sh | tail -1
./test/logger/run.sh | tail -1
./test/gps/run.sh | tail -1
./test/usbstream/run.sh | tail -1
./test/radio/run.sh | tail -1
./test/ground/run.sh | tail -1
./test/rylr/run.sh | tail -1
python3 test/rylr/check_link_config.py | head -2
./test/ground/run_lcd_driver.sh | tail -1
echo "EVERYTHING PASSED"
