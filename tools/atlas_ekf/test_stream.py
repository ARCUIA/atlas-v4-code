"""Parser tests, incl. YOUR current status line.  Run: python3 test_stream.py"""
import math, os, subprocess, shutil, sys, tempfile
from stream import StreamParser, fmt_imu, fmt_baro, fmt_gps, fmt_mag

LEGACY = ("IMU:OK xl[mg]=-86,-99,988 gy[mdps]=-280,210,140 | BARO:OK 98717 Pa | SD:OK LOG:OK sess=34 "
          "next=445392 wr=2067 drop=0 q=0/1 err=0 | GPS:OK fix=0 sv=0 pvt=1242 rxerr=0 | "
          "RADIO:NO-IRQ rx=0(-1) tx=0 fail=0 air=0ms")

def kinds(ev): return [e[0] for e in ev]

def test_legacy():
    p = StreamParser(time_fn=lambda: 5.0)
    ev = p.parse(LEGACY)
    assert kinds(ev) == ["I", "B", "G", "S"], kinds(ev)
    I = ev[0]; assert I[2] == [-0.086, -0.099, 0.988] and I[3] == [-0.28, 0.21, 0.14], I
    assert ev[1][2] == 98717.0
    assert ev[2][2:4] == (0, 0) and math.isnan(ev[2][4])
    st = ev[3][1]; assert st["SD"].startswith("SD:OK") and st["RADIO"].startswith("RADIO:NO-IRQ")
    print("legacy status line ok")

def test_new():
    p = StreamParser()
    ev = p.parse("IMU t=1234.567890 xl[mg]=-86,-99,988 gy[mdps]=-280,210,140")
    assert ev[0][0] == "I" and abs(ev[0][1] - 1234.56789) < 1e-9
    ev = p.parse("MAG t=1234.6 mag[mG]=123,-45,300"); assert abs(ev[0][2][0] - 12.3) < 1e-9
    ev = p.parse("BARO t=1234.6 p[Pa]=98717 T[C]=21.30"); assert ev[0][2] == 98717.0
    ev = p.parse("GPS t=1234.6 fix=3 sv=9 lat=41.6611234 lon=-91.5301234 hmsl[m]=221.600 vd[mps]=0.100 hacc[m]=0.400 vacc[m]=0.700 sacc[mps]=0.200")
    g = ev[0]; assert g[2] == 3 and g[3] == 9 and abs(g[4] - 41.6611234) < 1e-9 and abs(g[6] - 221.6) < 1e-9
    assert kinds(p.parse("IMU t=1 xl[mg]=1,2")) == ["L"]          # malformed -> log line, no crash
    assert kinds(p.parse("garbage")) == ["L"]
    ev = p.parse("IMU t=2 xl[mg]=-15990,0,0 gy[mdps]=0,0,2293000"); assert ev[0][4] and ev[0][5]   # rails
    print("new-format lines ok")

def test_modes():
    p = StreamParser(time_fn=lambda: 1.0)
    assert kinds(p.parse(LEGACY)) == ["I", "B", "G", "S"]
    ev = p.parse("IMU t=10.0 xl[mg]=0,0,1000 gy[mdps]=0,0,0")
    assert kinds(ev) == ["RESET", "I"], kinds(ev)                  # legacy -> timestamped switch
    assert kinds(p.parse(LEGACY)) == ["S"]                         # legacy now status-only
    ev = p.parse("IMU t=1.0 xl[mg]=0,0,1000 gy[mdps]=0,0,0"); assert kinds(ev) == ["RESET", "I"]   # MCU reset
    print("mode switching ok")

def test_roundtrip_python_formatters():
    p = StreamParser()
    ev = p.parse(fmt_imu(12.5, [0.1, -0.2, 0.98], [10.0, -5.0, 0.0]))[0]; assert abs(ev[2][1] + 0.2) < 1e-9
    assert p.parse(fmt_baro(12.5, 98717.4))[0][2] == 98717.0
    assert p.parse(fmt_mag(12.5, [120, -45, 300]))[0][0] == "M"
    g = p.parse(fmt_gps(12.5, 3, 9, 41.66, -91.53, 221.6, 0.1, 0.4, 0.7, 0.2))[0]; assert g[2] == 3
    print("python formatter round-trip ok")

def test_c_module():
    """Round trip through the firmware's own formatter (app/core/usbstream.c), if this folder sits in the repo."""
    here = os.path.dirname(os.path.abspath(__file__)); repo = os.path.abspath(os.path.join(here, "..", ".."))
    run = os.path.join(repo, "test", "usbstream", "run.sh")
    if not (shutil.which("gcc") and os.path.exists(run)):
        print("C round trip skipped (needs gcc and the firmware repo: test/usbstream/run.sh)"); return
    out = subprocess.check_output([run], text=True, stderr=subprocess.STDOUT)
    assert "ALL USB STREAM TESTS PASSED" in out, out
    print("firmware formatter round trip ok (test/usbstream/run.sh)")

if __name__ == "__main__":
    test_legacy(); test_new(); test_modes(); test_roundtrip_python_formatters(); test_c_module(); print("ALL OK")
