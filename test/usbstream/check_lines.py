#!/usr/bin/env python3
"""Feeds the firmware's own text output to the viewer's parser (tools/atlas_ekf/stream.py)."""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "tools", "atlas_ekf"))
from stream import StreamParser

lines = open(sys.argv[1], "r").read().splitlines()
p = StreamParser()
ev = [e for ln in lines for e in p.parse(ln)]
k = [e[0] for e in ev]
assert k.count("I") == 3 and k.count("M") == 1 and k.count("B") == 1 and k.count("G") == 2 and k.count("S") == 1, k
I = [e for e in ev if e[0] == "I"]
assert abs(I[0][1] - 1234.567890) < 1e-9 and I[0][2] == [-0.086, -0.099, 0.988] and I[0][3] == [-0.28, 0.21, 0.14], I[0]
assert I[1][4] and I[1][5], "rail flags"
assert abs(I[2][2][0] + 0.001) < 1e-9, "sign of -1 mg"
G = [e for e in ev if e[0] == "G"]
assert abs(G[0][4] - 41.6611234) < 1e-9 and abs(G[0][5] + 91.5301234) < 1e-9 and abs(G[0][6] - 221.6) < 1e-9
assert abs(G[1][4] + 0.0000123) < 1e-9 and abs(G[1][6] + 0.5) < 1e-9 and abs(G[1][8] - 999999.999) < 1e-6
assert math.isnan(G[0][10]), "no sacc in the stream is fine (nan)"
S = [e for e in ev if e[0] == "S"][0][1]
assert S["USB"].startswith("USB:OK") and S["SD"].startswith("SD:OK") and S["RADIO"].startswith("RADIO:NO-IRQ")
assert p.errors == 0
print(f"parser accepted all {len(lines)} firmware lines")
