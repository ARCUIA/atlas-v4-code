#!/bin/sh
# Only needed for the PARKED hex receiver (ground/parked/atlas_rx_rylr_hex): Arduino cannot include files from elsewhere,
# so that sketch folder holds copies of app/core. The teammate's receiver needs none of this.
cd "$(dirname "$0")/.."
D=ground/parked/atlas_rx_rylr_hex
for f in pkt rx_format lcd_layout rylr rylr_link lora_airtime ground_rx; do cp app/core/$f.c app/core/$f.h $D/; done
echo "$D updated from app/core"
