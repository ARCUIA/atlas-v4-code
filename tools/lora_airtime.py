#!/usr/bin/env python3
"""LoRa airtime and an estimate of receiver sensitivity (standard formula, 6 dB noise figure assumed).

    python3 tools/lora_airtime.py                      # comparison table for a 28-byte packet
    python3 tools/lora_airtime.py --sf 12 --bw 500 --len 28
"""
import argparse, math

SNR_LIMIT = {5: -2.5, 6: -5, 7: -7.5, 8: -10, 9: -12.5, 10: -15, 11: -17.5, 12: -20}


def airtime_ms(sf, bw_hz, payload, cr_den=5, preamble=8, crc=True, implicit=False):
    tsym = (2 ** sf) / bw_hz
    de = 1 if tsym > 0.016 else 0
    ih = 1 if implicit else 0
    num = 8 * payload - 4 * sf + 28 + (16 if crc else 0) - 20 * ih
    npay = 8 + max(math.ceil(num / (4 * (sf - 2 * de))) * cr_den, 0)
    return (preamble + 4.25 + npay) * tsym * 1000


def sensitivity_dbm(sf, bw_hz, nf_db=6):
    return -174 + 10 * math.log10(bw_hz) + nf_db + SNR_LIMIT[sf]


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sf", type=int); ap.add_argument("--bw", type=float, help="kHz")
    ap.add_argument("--len", type=int, default=28)
    a = ap.parse_args()
    rows = [(a.sf, a.bw * 1000)] if a.sf and a.bw else [(10, 125e3), (11, 125e3), (12, 125e3), (11, 500e3), (12, 500e3)]
    print(f"payload {a.len} bytes, CR 4/5, 8-symbol preamble, CRC on, explicit header")
    for sf, bw in rows:
        print(f"SF{sf:<2} / {bw / 1000:>5.0f} kHz: airtime {airtime_ms(sf, bw, a.len):7.0f} ms, "
              f"sensitivity about {sensitivity_dbm(sf, bw):6.1f} dBm")
