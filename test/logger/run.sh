#!/bin/sh
# Host tests for the logger and the USB dump path. No hardware needed.
set -e
cd "$(dirname "$0")/../.."
OUT=/tmp/atlas_logger_test; rm -rf $OUT; mkdir -p $OUT
INC="-Itest/sd_sim/stub -Itest/sd_sim -Idevices -Idevices/sd -Idevices/imu -Idevices/baro -Iapp -Iapp/core"
CC="gcc -std=gnu11 -Wall -Wextra -Werror -g $INC"
COMMON="test/sd_sim/sim_card.c devices/sd/sd_spi.c app/log_service.c app/core/logblk.c test/logger/stubs.c"

$CC test/logger/test_logger.c $COMMON -o $OUT/test_logger
(cd $OUT && ./test_logger)

$CC test/logger/dump_host.c $COMMON app/dump.c app/core/ringbuf.c -o $OUT/dump_host
$CC test/logger/test_dump_stray.c $COMMON app/dump.c app/core/ringbuf.c -o $OUT/test_dump_stray
$OUT/test_dump_stray

echo "--- Python parser on the exported log ---"
python3 tools/atlas_parse.py $OUT/dump.bin --outdir $OUT/csv
python3 - <<PY
exp = dict(l.split() for l in open("$OUT/expected.txt"))
imu = sum(1 for _ in open("$OUT/csv/imu.csv")) - 1
baro = sum(1 for _ in open("$OUT/csv/baro.csv")) - 1
assert imu == int(exp["imu"]) and baro == int(exp["baro"]), (imu, baro, exp)
gps_rows = list(__import__("csv").DictReader(open("$OUT/csv/gps.csv")))
assert len(gps_rows) == int(exp["gps"]), (len(gps_rows), exp)
assert gps_rows[0]["lat_deg"] == "41.6600000" and gps_rows[0]["lon_deg"] == "-91.6500000", gps_rows[0]
assert gps_rows[0]["fix_type"] == "3" and gps_rows[0]["num_sv"] == "9" and gps_rows[0]["utc"].startswith("2026-11-05"), gps_rows[0]
print(f"parser output matches the firmware: {imu} IMU rows, {baro} baro rows, {len(gps_rows)} GPS rows")
PY

echo "--- Python dump tool against the firmware's dump protocol ---"
timeout 60 python3 tools/atlas_dump.py --exec "$OUT/dump_host" -o $OUT/dumped.bin
timeout 60 python3 tools/atlas_dump.py --exec "$OUT/dump_host" --info
python3 tools/atlas_parse.py $OUT/dumped.bin --outdir $OUT/csv2 | head -3
echo "ALL LOGGER + TOOL TESTS PASSED"
