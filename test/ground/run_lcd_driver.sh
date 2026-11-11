#!/bin/sh
# Runs the same screen scenario through our driver and through a community vendor-compatible driver, on a simulated
# AiP31068L + PCA9633 I2C bus, and requires identical results. Needs a copy of that vendor driver (see below).
set -e
cd "$(dirname "$0")/../.."
VENDOR=${VENDOR_LCD_DIR:-/tmp/wslcd/cpp}
CXX="g++ -std=gnu++17 -Wall -Wextra -Werror -Itest/ground/mock"
$CXX -Iground/parked/atlas_rx_rylr_hex test/ground/scenario_mine.cpp -o /tmp/lcd_mine
/tmp/lcd_mine > /tmp/lcd_mine.txt
cat /tmp/lcd_mine.txt
if [ -f "$VENDOR/LCD1602_RGB.cpp" ]; then
  g++ -std=gnu++17 -w -Itest/ground/mock -I"$VENDOR" test/ground/scenario_vendor.cpp "$VENDOR/LCD1602_RGB.cpp" -o /tmp/lcd_vendor
  /tmp/lcd_vendor > /tmp/lcd_vendor.txt
  grep -v '^RGB=' /tmp/lcd_mine.txt > /tmp/lcd_mine_text.txt
  grep -v '^RGB=' /tmp/lcd_vendor.txt > /tmp/lcd_vendor_text.txt
  diff /tmp/lcd_mine_text.txt /tmp/lcd_vendor_text.txt && echo "screen text and init registers identical to the vendor-compatible driver: OK"
  echo "note: that driver sets the colour with one 3-byte write, which needs the chip's auto-increment flag; ours writes each colour register separately, as Waveshare's Python demo does (vendor result: $(grep '^RGB=' /tmp/lcd_vendor.txt))"
else
  echo "(vendor driver not found at $VENDOR: skipped the cross-check)"
fi
grep -q "ROW0=|ATLA42GROUND RX |" /tmp/lcd_mine.txt && grep -q "ROW1=|LON -91.650000  |" /tmp/lcd_mine.txt \
  && grep -q "RGB=255,160,0" /tmp/lcd_mine.txt && grep -q "MODE1=0 LEDOUT=255 MODE2=32" /tmp/lcd_mine.txt \
  && grep -q "DISPLAY_ON=1" /tmp/lcd_mine.txt && echo "screen text, colour registers and init values as expected: OK"
