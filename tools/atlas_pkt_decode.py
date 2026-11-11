#!/usr/bin/env python3
"""Decode an Atlas telemetry packet given as hex (the format is documented in app/core/pkt.h).

    python3 tools/atlas_pkt_decode.py A7100102...
"""
import struct, sys

MAGIC, VERSION = 0xA7, 1


def decode(data: bytes) -> dict:
    if len(data) < 5 or data[0] != MAGIC:
        raise ValueError("not an Atlas packet")
    ver, flags = data[1] >> 4, data[1] & 0x0F
    if ver != VERSION:
        raise ValueError(f"unknown packet version {ver}")
    out = dict(version=ver, flags=flags, src=data[2], seq=struct.unpack_from("<H", data, 3)[0], unknown=[])
    pos = 5
    while pos < len(data):
        if pos + 2 > len(data):
            raise ValueError("truncated record header")
        rtype, rlen = data[pos], data[pos + 1]
        if pos + 2 + rlen > len(data):
            raise ValueError("truncated record")
        d = data[pos + 2:pos + 2 + rlen]
        pos += 2 + rlen
        if rtype == 1 and rlen >= 17:
            fix, sv, lat, lon, alt, spd, hacc = struct.unpack_from("<BBiiiHB", d)
            out["gps"] = dict(fix=fix, sv=sv, lat=lat / 1e7, lon=lon / 1e7, alt_m=alt / 10.0,
                              speed_mps=spd / 10.0, hacc_m=hacc)
        elif rtype == 2 and rlen >= 2:
            out["batt_mv"] = struct.unpack_from("<H", d)[0]
        else:
            out["unknown"].append((rtype, d.hex()))
    return out


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    r = decode(bytes.fromhex("".join(sys.argv[1:])))
    print(f"seq {r['seq']}  source {r['src']}  version {r['version']}")
    if "gps" in r:
        g = r["gps"]
        print(f"GPS fix={g['fix']} sv={g['sv']} lat={g['lat']:.7f} lon={g['lon']:.7f} alt={g['alt_m']:.1f} m "
              f"speed={g['speed_mps']:.1f} m/s hAcc={g['hacc_m']} m")
    if "batt_mv" in r:
        print(f"battery {r['batt_mv']} mV")
    for t, h in r["unknown"]:
        print(f"unknown record type {t}: {h}")
