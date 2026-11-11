"""Checks that the Python decoder reads the packet the C code built."""
import sys
sys.path.insert(0, "tools")
from atlas_pkt_decode import decode
r = decode(bytes.fromhex(open(sys.argv[1]).read().strip()))
assert r["seq"] == 258 and r["src"] == 1 and r["version"] == 1, r
g = r["gps"]
assert (g["fix"], g["sv"]) == (3, 9) and abs(g["lat"] - 41.66) < 1e-9 and abs(g["lon"] + 91.65) < 1e-9, g
assert abs(g["alt_m"] - 215.5) < 1e-9 and abs(g["speed_mps"] - 12.3) < 1e-9 and g["hacc_m"] == 2, g
assert r["batt_mv"] == 7400 and not r["unknown"], r
print("Python decoder agrees with the C encoder: OK")
# the design numbers quoted for the link settings
from lora_airtime import airtime_ms, sensitivity_dbm
a = airtime_ms(12, 500e3, 24)
assert 320 < a < 340, a
assert airtime_ms(12, 125e3, 24) > 1400                # SF12 at 125 kHz cannot fit one packet per second
print(f"link budget check: SF12/500 kHz, 24 bytes = {a:.0f} ms, sensitivity about {sensitivity_dbm(12, 500e3):.0f} dBm: OK")
