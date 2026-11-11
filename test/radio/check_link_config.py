"""Link-settings guard: the flight board and the ground station must agree, and every value must be one that
RadioLib's LR2021 code accepts. If third_party/RadioLib is present, the accepted values are read from its source."""
import re, sys, os

def macro(text, name):
    m = re.search(r"#define\s+" + name + r"\s+(0[xX][0-9A-Fa-f]+|[0-9]+(?:\.[0-9]+)?)", text)   # suffixes like f / UL are ignored
    assert m, f"{name} not found"
    return float(int(m.group(1), 16)) if m.group(1).lower().startswith("0x") else float(m.group(1))

flight = open("devices/radio/radio.h").read()
ground = open("ground/atlas_rx/atlas_rx.ino").read()
pairs = [("RADIO_FREQ_MHZ", "FREQ_MHZ"), ("RADIO_BW_KHZ", "BW_KHZ"), ("RADIO_SF", "SF"),
         ("RADIO_CR_DEN", "CR_DEN"), ("RADIO_SYNC_WORD", "SYNC_WORD"), ("RADIO_PREAMBLE", "PREAMBLE")]
for f, g in pairs:
    a, b = macro(flight, f), macro(ground, g)
    assert a == b, f"flight {f}={a} but ground {g}={b}: the two ends would not hear each other"
freq, bw, sf, cr = (macro(flight, n) for n in ("RADIO_FREQ_MHZ", "RADIO_BW_KHZ", "RADIO_SF", "RADIO_CR_DEN"))
power = macro(flight, "RADIO_TX_DBM")

# defaults if RadioLib's source is not in the tree
freq_ranges = [(150.0, 1090.0), (1900.0, 2200.0), (2400.0, 2500.0)]
bw_div2 = {15, 20, 31, 41, 50, 101, 62, 125, 203, 250, 406, 500}
src = "third_party/RadioLib/src/modules/LR2021/LR2021_config.cpp"
if os.path.exists(src):
    t = open(src).read()
    freq_ranges = [(float(a), float(b)) for a, b in re.findall(r"freq >= ([\d.]+)f\) && \(freq <= ([\d.]+)f", t)]
    body = t[t.index("int16_t LR2021::setBandwidth(float bw)"):]
    body = body[:body.index("setLoRaModulationParams")]
    bw_div2 = {int(x) for x in re.findall(r"case (\d+):", body)}
    print(f"(accepted values read from RadioLib's own source: {len(freq_ranges)} frequency ranges, {len(bw_div2)} bandwidths)")
assert any(lo <= freq <= hi for lo, hi in freq_ranges), f"frequency {freq} MHz is outside RadioLib's accepted ranges {freq_ranges}"
assert int(bw / 2 + 0.01) in bw_div2, f"bandwidth {bw} kHz is not one RadioLib accepts"
assert 5 <= sf <= 12 and 4 <= cr <= 8
assert -9 <= power <= 22, "sub-GHz output power range is -9..22 dBm"
assert 902.0 + bw / 2000 <= freq <= 928.0 - bw / 2000, "the whole channel must sit inside 902-928 MHz"
print(f"link settings OK and identical on both ends: {freq} MHz, {bw} kHz, SF{int(sf)}, CR 4/{int(cr)}, {power:.0f} dBm")
