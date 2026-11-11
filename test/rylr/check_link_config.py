"""Guards the flight radio settings in devices/radio/radio.h against the rules in REYAX's AT command guide."""
import re, sys

def macro(text, name):
    m = re.search(r"#define\s+" + name + r"\s+(0[xX][0-9A-Fa-f]+|[0-9]+(?:\.[0-9]+)?)", text)   # suffixes like UL are ignored
    assert m, f"{name} not found in radio.h"
    return int(m.group(1), 16) if m.group(1).lower().startswith("0x") else float(m.group(1))

h = open("devices/radio/radio.h").read()
own, dest, net = (int(macro(h, n)) for n in ("RADIO_OWN_ADDR", "RADIO_DEST_ADDR", "RADIO_NETWORK_ID"))
band, sf, bw, cr, pre, dbm = (int(macro(h, n)) for n in ("RADIO_BAND_HZ", "RADIO_SF", "RADIO_BW_CODE", "RADIO_CR_CODE", "RADIO_PREAMBLE", "RADIO_TX_DBM"))

RECEIVER = {"network": 6, "address": 1, "band": 915000000}      # what the receiver module is set to (as reported)
assert net == RECEIVER["network"], f"flight network {net} but the receiver module is on network {RECEIVER['network']}"
assert dest in (0, RECEIVER["address"]), f"destination {dest} would not reach a receiver at address {RECEIVER['address']} (0 = broadcast)"
assert own != RECEIVER["address"], "both modules must not share one address"
assert band == RECEIVER["band"], "band differs from the receiver"
assert 0 <= dest <= 65535, "destination address 0..65535 (0 = broadcast)"
assert own == 0xFFFF or 0 <= own <= 65534, "own address 0..65535 (0xFFFF here means: leave it alone)"
assert (3 <= net <= 15) or net == 18, "network id must be 3..15 or 18"
assert 5 <= sf <= 11, "spreading factor 5..11"
assert bw in (7, 8, 9), "bandwidth code 7 (125 kHz), 8 (250 kHz) or 9 (500 kHz)"
assert (bw == 7 and sf <= 9) or (bw == 8 and sf <= 10) or bw == 9, "module limit: SF9 at 125 kHz, SF10 at 250 kHz, SF11 at 500 kHz"
assert 1 <= cr <= 4, "coding rate 1..4"
assert (net == 18 and 4 <= pre <= 24) or pre == 12, "preamble 4..24 only on network 18; otherwise it must be 12"
assert 0 <= dbm <= 22, "output power 0..22 dBm"
bw_hz = {7: 125e3, 8: 250e3, 9: 500e3}[bw]
assert 902e6 + bw_hz / 2 <= band <= 928e6 - bw_hz / 2, "the whole channel must sit inside 902-928 MHz"

def airtime_ms(sf, bw_hz, cr_den, pre, payload):
    tsym = (2 ** sf) / bw_hz; de = 1 if tsym > 0.016 else 0
    n = 8 + max(-(-(8 * payload - 4 * sf + 28 + 16) // (4 * (sf - 2 * de))) * cr_den, 0)
    return (pre + 4.25 + n) * tsym * 1000

worst = len("YG,-179.999999,-179.999999") + 8          # longest message plus the module's 8 bytes
a = airtime_ms(sf, bw_hz, cr + 4, pre, worst)
assert a < 600, f"airtime {a:.0f} ms is too long for one message per second"
sens = -174 + 10 * __import__("math").log10(bw_hz) + 6 - {5: 2.5, 6: 5, 7: 7.5, 8: 10, 9: 12.5, 10: 15, 11: 17.5}[sf]
print(f"radio settings OK: network {net}, SF{sf}, {bw_hz/1e3:.0f} kHz, {dbm} dBm; longest message {a:.0f} ms on air, sensitivity about {sens:.1f} dBm")
if (sf, bw, cr, pre) == (9, 7, 1, 12):
    print("(the RF parameters are the module's factory defaults: neither teammate sketch changes them on its module)")
