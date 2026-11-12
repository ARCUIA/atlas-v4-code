"""Atlas USB text stream: parsing, formatting, and data sources.

Two kinds of line are understood.

1) NEW timestamped sample lines (recommended; same human-readable style as the status line):
     IMU  t=1234.567890 xl[mg]=-86,-99,988 gy[mdps]=-280,210,140
     MAG  t=1234.567890 mag[mG]=123,-45,300
     BARO t=1234.567890 p[Pa]=98717 T[C]=21.30
     GPS  t=1234.567890 fix=3 sv=9 lat=41.6611234 lon=-91.5301234 hmsl[m]=221.600 vd[mps]=0.100 hacc[m]=0.400 vacc[m]=0.700 sacc[mps]=0.200
   t is the MCU time in seconds (same clock as the SD log). Missing fields are tolerated.

2) The LEGACY 1-line status:
     IMU:OK xl[mg]=-86,-99,988 gy[mdps]=-280,210,140 | BARO:OK 98717 Pa | SD:OK LOG:OK sess=34 ... | GPS:OK fix=0 sv=0 ... | RADIO:...
   It has no timestamps, so samples from it are stamped with host arrival time.  It carries no GPS
   position and no mag.  Once timestamped sample lines are seen, sensor values from legacy lines are
   ignored and the line is only shown as status.

Events produced (tuples):
  ("I", t, f_g[3], w_dps[3], rail_xl, rail_gy)     ("M", t, m_uT[3])      ("B", t, p_pa)
  ("G", t, fix, nsv, lat, lon, hmsl, vd, hacc, vacc, sacc)   (nan where unknown)
  ("S", {segment: text}, raw)   ("L", raw)   ("RESET", why)   ("ERR", msg)
"""
from __future__ import annotations
import math, os, queue, re, sys, threading, time
import numpy as np

NAN = float("nan")
XL_RAIL_G = 15.98          # accel axis at/above this => treated as full-scale rail
GY_RAIL_DPS = 2290.0
MG_TO_UT = 0.1             # 1 milligauss = 0.1 microtesla


def _kv(tokens):
    d = {}
    for tok in tokens:
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d


def _vec(s, n=3):
    v = [float(x) for x in s.split(",")]
    if len(v) != n:
        raise ValueError("bad vector")
    return v


class StreamParser:
    def __init__(self, time_fn=time.monotonic):
        self.time_fn = time_fn
        self.t0 = time_fn()
        self.timestamped = False
        self.legacy_samples = False
        self.last_t = None
        self.errors = 0

    def host_t(self):
        return self.time_fn() - self.t0

    def _stamp(self, d):
        """Return (t, events_prefix) using MCU time when present."""
        pre = []
        if "t" in d:
            t = float(d["t"])
            if not self.timestamped:
                self.timestamped = True
                if self.legacy_samples:
                    pre.append(("RESET", "switched to timestamped stream"))
                    self.legacy_samples = False
            if self.last_t is not None and t < self.last_t - 1.0:
                pre.append(("RESET", "MCU time went backwards (reset?)"))
            self.last_t = t
            return t, pre
        return self.host_t(), pre

    def parse(self, line: str):
        line = line.strip()
        if not line:
            return []
        try:
            if line.startswith("IMU:"):
                return self._legacy(line)
            tok = line.split()
            tag = tok[0]
            if tag in ("IMU", "MAG", "BARO", "GPS"):
                d = _kv(tok[1:])
                t, ev = self._stamp(d)
                if tag == "IMU":
                    f = [x / 1000.0 for x in _vec(d["xl[mg]"])]
                    w = [x / 1000.0 for x in _vec(d["gy[mdps]"])]
                    ev.append(("I", t, f, w, max(abs(x) for x in f) >= XL_RAIL_G,
                               max(abs(x) for x in w) >= GY_RAIL_DPS))
                elif tag == "MAG":
                    ev.append(("M", t, [x * MG_TO_UT for x in _vec(d["mag[mG]"])]))
                elif tag == "BARO":
                    ev.append(("B", t, float(d["p[Pa]"])))
                else:
                    g = lambda k: float(d[k]) if k in d else NAN
                    ev.append(("G", t, int(d.get("fix", 0)), int(d.get("sv", 0)), g("lat"), g("lon"),
                               g("hmsl[m]"), g("vd[mps]"), g("hacc[m]"), g("vacc[m]"), g("sacc[mps]")))
                return ev
        except (KeyError, ValueError, IndexError):
            self.errors += 1
            return [("L", line)]
        return [("L", line)]

    # ---- legacy status line
    def _legacy(self, line):
        segs = [s.strip() for s in line.split("|")]
        status = {}
        for s in segs:
            name = s.split(":", 1)[0].strip()
            status[name] = s
        ev = [("S", status, line)]
        if self.timestamped:
            return ev
        t = self.host_t()
        out = []
        m = re.search(r"xl\[mg\]=([-\d.,]+)\s+gy\[mdps\]=([-\d.,]+)", status.get("IMU", ""))
        if m and status["IMU"].startswith("IMU:OK"):
            try:
                f = [x / 1000.0 for x in _vec(m.group(1))]
                w = [x / 1000.0 for x in _vec(m.group(2))]
                out.append(("I", t, f, w, max(abs(x) for x in f) >= XL_RAIL_G,
                            max(abs(x) for x in w) >= GY_RAIL_DPS))
            except ValueError:
                pass
        m = re.search(r"(\d+)\s*Pa", status.get("BARO", ""))
        if m and status["BARO"].startswith("BARO:OK"):
            out.append(("B", t, float(m.group(1))))
        m = re.search(r"fix=(\d+)\s+sv=(\d+)", status.get("GPS", ""))
        if m:
            out.append(("G", t, int(m.group(1)), int(m.group(2)), NAN, NAN, NAN, NAN, NAN, NAN, NAN))
        if out:
            self.legacy_samples = True
        return out + ev


# ------------------------------------------------------------------ formatting
# Mirrors firmware/usb_stream.c so replay exercises the same text path as the real stream.
def fmt_t(t): return f"{t:.6f}"


def fmt_imu(t, f_g, w_dps):
    return (f"IMU t={fmt_t(t)} xl[mg]={f_g[0]*1000:.0f},{f_g[1]*1000:.0f},{f_g[2]*1000:.0f} "
            f"gy[mdps]={w_dps[0]*1000:.0f},{w_dps[1]*1000:.0f},{w_dps[2]*1000:.0f}")


def fmt_mag(t, m_mG): return f"MAG t={fmt_t(t)} mag[mG]={m_mG[0]:.0f},{m_mG[1]:.0f},{m_mG[2]:.0f}"
def fmt_baro(t, p): return f"BARO t={fmt_t(t)} p[Pa]={p:.0f}"


def fmt_gps(t, fix, sv, lat, lon, hmsl, vd, hacc, vacc, sacc):
    return (f"GPS t={fmt_t(t)} fix={fix} sv={sv} lat={lat:.7f} lon={lon:.7f} hmsl[m]={hmsl:.3f} "
            f"vd[mps]={vd:.3f} hacc[m]={hacc:.3f} vacc[m]={vacc:.3f} sacc[mps]={sacc:.3f}")


# --------------------------------------------------------------------- sources
def serial_thread(port, baud, q, stop, parser=None):
    """Read lines from the CDC port, parse, push events. Reconnects on failure."""
    try:
        import serial
    except ImportError:
        q.put(("ERR", "pip install pyserial")); return
    parser = parser or StreamParser()
    while not stop.is_set():
        try:
            s = serial.Serial(port, baud, timeout=0.2)
        except Exception as e:                                   # noqa: BLE001
            q.put(("ERR", f"cannot open {port}: {e}")); time.sleep(1.0); continue
        q.put(("ERR", f"connected {port}"))
        buf = b""
        try:
            while not stop.is_set():
                buf += s.read(4096)
                while b"\n" in buf:
                    ln, buf = buf.split(b"\n", 1)
                    for ev in parser.parse(ln.decode("ascii", "ignore")):
                        try: q.put_nowait(ev)
                        except queue.Full: pass
                if len(buf) > 100000: buf = b""
        except Exception as e:                                   # noqa: BLE001
            q.put(("ERR", f"serial lost: {e}"))
        try: s.close()
        except Exception: pass                                   # noqa: BLE001
        time.sleep(1.0)


def replay_thread(d, start, speed, q, stop, fake_mag=False, parser=None):
    """Replay flight CSVs in real time as *text lines* through StreamParser (tests the real path)."""
    from flightdata import load_flight
    D = load_flight(d)
    parser = parser or StreamParser(time_fn=lambda: 0.0)       # replay lines are all timestamped
    i = int(np.searchsorted(D["imu_t"], start)); b = int(np.searchsorted(D["baro_t"], start))
    g = int(np.searchsorted(D["gps_t"], start)); n = len(D["imu_t"])
    wall0 = time.monotonic(); next_mag = start; next_stat = start
    rng = np.random.default_rng(0)

    def emit(line):
        for ev in parser.parse(line):
            try: q.put_nowait(ev)
            except queue.Full: pass

    while not stop.is_set() and i < n:
        tgt = start + (time.monotonic() - wall0) * speed
        while i < n and D["imu_t"][i] <= tgt:
            t = D["imu_t"][i]
            while b < len(D["baro_t"]) and D["baro_t"][b] <= t:
                emit(fmt_baro(D["baro_t"][b], D["baro_p"][b])); b += 1
            while g < len(D["gps_t"]) and D["gps_t"][g] <= t:
                emit(fmt_gps(D["gps_t"][g], D["gps_fix"][g], D["gps_nsv"][g], D["gps_lat"][g],
                             D["gps_lon"][g], D["gps_h"][g], D["gps_vd"][g], D["gps_hacc"][g],
                             D["gps_vacc"][g], D["gps_sacc"][g])); g += 1
            emit(fmt_imu(t, D["imu_f"][i], D["imu_w"][i]))
            if fake_mag and t >= next_mag:                  # SYNTHETIC, only to exercise the mag plot
                next_mag += 0.02
                th = 0.15 * t
                m = np.array([200 + 40*np.sin(th), -20 + 30*np.cos(th), 450 + 20*np.sin(0.5*th)])
                emit(fmt_mag(t, m + rng.normal(0, 3, 3)))
            if t >= next_stat:                              # status line like the firmware prints
                next_stat += 1.0
                fx = D["gps_fix"][min(g, len(D['gps_fix'])-1)]; sv = D["gps_nsv"][min(g, len(D['gps_nsv'])-1)]
                emit(f"IMU:OK xl[mg]={D['imu_f'][i][0]*1000:.0f},{D['imu_f'][i][1]*1000:.0f},{D['imu_f'][i][2]*1000:.0f} "
                     f"gy[mdps]={D['imu_w'][i][0]*1000:.0f},{D['imu_w'][i][1]*1000:.0f},{D['imu_w'][i][2]*1000:.0f} "
                     f"| BARO:OK {D['baro_p'][max(b-1, 0)]:.0f} Pa | SD:OK LOG:OK sess=REPLAY | GPS:OK fix={fx} sv={sv} "
                     f"| RADIO:REPLAY")
            i += 1
        time.sleep(0.004)


def file_thread(path, line_rate, q, stop, parser=None):
    """Replay a captured text stream. Pacing: --line-rate if given; else the t= stamps of timestamped
    lines (real time); else 10 lines/s (legacy status lines have no stamps)."""
    parser = parser or StreamParser()
    with open(path, "r", errors="ignore") as fh:
        lines = fh.readlines()
    wall0 = time.monotonic(); t_first = None; k = 0
    for ln in lines:
        if stop.is_set():
            return
        m = re.search(r"\bt=([0-9.]+)", ln)
        if line_rate > 0:
            due = k / line_rate
        elif m:
            tt = float(m.group(1)); t_first = tt if t_first is None else t_first; due = tt - t_first
        else:
            due = k / 10.0
        k += 1
        while not stop.is_set() and time.monotonic() - wall0 < due:
            time.sleep(0.002)
        for ev in parser.parse(ln):
            try: q.put_nowait(ev)
            except queue.Full: pass
