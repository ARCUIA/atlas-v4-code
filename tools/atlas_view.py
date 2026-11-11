#!/usr/bin/env python3
"""
atlas_view.py - flight data viewer for Atlas v4 (reads the CSVs from atlas_parse.py)

    python3 tools/atlas_view.py cols  flight_csv                 # show headers + what got mapped
    python3 tools/atlas_view.py sessions flight_csv              # list sessions (rows, duration)
    python3 tools/atlas_view.py plot  flight_csv -o flight.html  # interactive plot (open in browser)
    python3 tools/atlas_view.py hgcal flight_csv                 # check/fit the high-g scale factor

Use it from an EKF prototype as a library:

    from atlas_view import load_flight, plot_flight
    f = load_flight("flight_csv")               # f.imu / f.baro / f.gps / f.events, SI units, launch-relative time
    ...run your EKF over f.imu, f.baro, f.gps...
    plot_flight(f, "flight.html", overlays={"EKF": {"alt": (t, h), "vel": (t, v)}})

Units after loading: accel in g, gyro in dps, pressure in Pa, altitude in m, speed in m/s, time in s
(t = 0 at detected launch).  Everything the loader had to guess (units, time base) is printed in f.notes.
"""
import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

G0 = 9.80665

# ---- sensor scale defaults (override on the command line) -------------------------------------
LG_G_PER_LSB = 16.0 / 32768.0          # +-16 g low-g accel      (0.488 mg/LSB)
HG_G_PER_LSB = 80.0 / 32768.0          # +-80 g high-g accel     UNVERIFIED -> run `hgcal`
GYRO_DPS_PER_LSB = 0.070               # +-2000 dps              (70 mdps/LSB)
LG_FULL_SCALE_G = 16.0

# ---- column-name guesses; override any of them with --map map.json ---------------------------
AX = "xyz"
_T = ["t", "time", "t_s", "t_ms", "t_us", "time_s", "time_ms", "time_us", "timestamp", "timestamp_ms",
      "timestamp_us", "tick", "tick_ms", "tick_us", "ts", "ts_ms", "ts_us", "mcu_ms", "mcu_us"]
_LG = ["a{a}", "acc_{a}", "accel_{a}", "a_{a}", "la{a}", "lg_{a}", "acc{a}", "a{a}_raw", "acc_{a}_raw",
       "accel_{a}_raw", "lg_{a}_raw", "a{a}_lg", "lg{a}", "xl_{a}_mg"]
_HG = ["ha{a}", "hg_{a}", "hacc_{a}", "a{a}_hg", "highg_{a}", "hg{a}", "hg_{a}_raw", "ha{a}_raw",
       "a{a}_hg_raw", "acc_hg_{a}", "accel_hg_{a}", "hg_{a}_mg"]
_GY = ["g{a}", "gyr_{a}", "gyro_{a}", "g_{a}", "w{a}", "g{a}_raw", "gyro_{a}_raw", "gyr_{a}_raw", "gyr{a}", "gy_{a}_mdps"]
ALIASES = {
    "imu_t": _T, "baro_t": _T, "gps_t": _T,
    "p": ["p", "press", "pressure", "press_pa", "pressure_pa", "p_pa", "p_hpa", "press_hpa", "pressure_hpa",
          "press_raw", "pressure_raw", "p_raw", "baro", "baro_raw"],
    "lat": ["lat_deg", "lat", "latitude"], "lon": ["lon_deg", "lon", "lng", "longitude"],
    "alt": ["hmsl_m", "hmsl", "height_msl", "alt", "altitude", "alt_m", "height", "alt_mm", "hmsl_mm"],
    "gspeed": ["gspeed_mps", "gspeed", "g_speed", "gs", "ground_speed", "speed"],
    "veld": ["vel_d_mps", "veld", "vel_d", "velocity_d", "vd"],
    "fix": ["fixtype", "fix_type", "fix", "gpsfix"],
    "nsv": ["numsv", "num_sv", "nsv", "sats", "satellites", "numsats"],
}
for i, a in enumerate(AX):
    ALIASES[f"lg{a}"] = [s.format(a=a) for s in _LG]
    ALIASES[f"hg{a}"] = [s.format(a=a) for s in _HG]
    ALIASES[f"gy{a}"] = [s.format(a=a) for s in _GY]


@dataclass
class Flight:
    imu: pd.DataFrame
    baro: pd.DataFrame
    gps: pd.DataFrame
    status: pd.DataFrame = field(default_factory=pd.DataFrame)
    events: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    t_launch_abs: float = 0.0


def _to_unit(v, col, lsb, kind):
    """-> (converted values, description). kind: 'g' or 'dps'. Honors the unit suffix atlas_parse.py writes."""
    c = col.lower()
    if c.endswith("_mg") or c.endswith("_mdps"):
        return v / 1000.0, "milli-units / 1000"
    if _is_int_valued(v):
        return v * lsb, "raw counts * %.4g" % lsb
    return v, kind


# ---- loading ------------------------------------------------------------------------------------
def _pick(df, key, cmap):
    if key in cmap:
        return cmap[key] if cmap[key] in df.columns else None
    low = {c.lower().strip(): c for c in df.columns}
    for a in ALIASES.get(key, []):
        if a in low:
            return low[a]
    return None


def _is_int_valued(v):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    return v.size > 0 and np.all(v == np.round(v))


def _time_s(df, key, cmap, hz, notes, label):
    col = _pick(df, key, cmap)
    if col is None:
        raise SystemExit(f"[{label}] no time column found (columns: {list(df.columns)}). Use --map.")
    raw = df[col].to_numpy(dtype=float)
    if col.lower().endswith("_s"):
        notes.append(f"{label}: time column '{col}' is seconds")
        return raw
    dt = np.nanmedian(np.diff(raw))
    scale = min((1.0, 1e-3, 1e-6), key=lambda s: abs(np.log(max(abs(dt) * s, 1e-12) / (1.0 / hz))))
    unit = {1.0: "s", 1e-3: "ms", 1e-6: "us"}[scale]
    notes.append(f"{label}: time column '{col}' interpreted as {unit} (median step {dt:g} -> {dt*scale*1e3:.2f} ms, "
                 f"expected ~{1e3/hz:.1f} ms)")
    return raw * scale


def session_table(path, chunk=1_000_000):
    """One row per session id in imu.csv (rows, time span). Reads only 'session' and 't_s', in chunks."""
    hdr = list(pd.read_csv(path, nrows=0).columns)
    if "session" not in hdr:
        return pd.DataFrame()
    tc = next((c for c in hdr if c.lower() in _T), None)
    use = ["session"] + ([tc] if tc else [])
    parts = []
    for ch in pd.read_csv(path, usecols=use, dtype={"session": str}, chunksize=chunk):
        g = ch.groupby("session", sort=False)
        part = pd.DataFrame({"imu_rows": g.size()})
        if tc:
            part["t_first"] = g[tc].min()
            part["t_last"] = g[tc].max()
        parts.append(part)
    allp = pd.concat(parts)
    g = allp.groupby(level=0, sort=False)
    out = pd.DataFrame({"imu_rows": g.imu_rows.sum()})
    if tc:
        out["t_first"] = g.t_first.min()
        out["t_last"] = g.t_last.max()
        out["span"] = out.t_last - out.t_first          # in the file's own time unit (t_s -> seconds)
    return out


def _resolve_session(tab, session):
    """Exact text match first; then '32' -> the unique 'X.32'; then numeric match."""
    ids = list(tab.index)
    want = str(session).strip()
    if want in ids:
        return want
    suf = [i for i in ids if i.endswith("." + want)]
    if len(suf) == 1:
        return suf[0]
    if len(suf) > 1:
        raise SystemExit(f"--session {want} is ambiguous: {suf}. Give the full id.")
    try:
        num = [i for i in ids if abs(float(i) - float(want)) < 1e-12]
        if len(num) == 1:
            return num[0]
    except ValueError:
        pass
    return None


def _choose_session(path, session, notes):
    tab = session_table(path)
    if tab.empty:
        return None
    if session is None:
        sid = tab.imu_rows.idxmax()
        notes.append(f"no --session given; using {sid!r} (most IMU rows). Sessions found:\n{tab.to_string()}")
        return sid
    sid = _resolve_session(tab, session)
    if sid is None:
        raise SystemExit(f"session {session!r} not found in imu.csv. Sessions:\n{tab.to_string()}")
    return sid


def _read_csv(path, sid=None, chunk=500_000):
    """Read one session of a csv, in chunks, skipping the *_raw and flags columns (the big, unused ones)."""
    hdr = list(pd.read_csv(path, nrows=0).columns)
    use = [c for c in hdr if "_raw" not in c.lower() and c.lower() != "flags"]
    dt = {"session": str}
    if sid is None or "session" not in hdr:
        return pd.read_csv(path, usecols=use, dtype=dt)
    parts = [c[c["session"] == sid] for c in pd.read_csv(path, usecols=use, dtype=dt, chunksize=chunk)]
    parts = [p for p in parts if len(p)]
    return pd.concat(parts, ignore_index=True) if parts else pd.DataFrame(columns=use)


def load_flight(path, cmap=None, session=None, lg_lsb=LG_G_PER_LSB, hg_lsb=HG_G_PER_LSB,
                gyro_lsb=GYRO_DPS_PER_LSB, launch_g=2.5, hg_scale=1.0, accel_src="merged"):
    cmap = cmap or {}
    d = Path(path)
    notes = []
    if not (d / "imu.csv").exists():
        raise SystemExit(f"no imu.csv in {d}")
    sid = _choose_session(d / "imu.csv", session, notes)
    dfs = {n: _read_csv(d / f"{n}.csv", sid) for n in ("imu", "baro", "gps", "status") if (d / f"{n}.csv").exists()}
    if len(dfs["imu"]) < 100:
        raise SystemExit(f"session {sid!r} has only {len(dfs['imu'])} IMU rows; not a flight.")
    notes.append(f"loaded session {sid!r}: {len(dfs['imu'])} IMU rows")

    # --- IMU
    raw = dfs["imu"]
    imu = pd.DataFrame({"t_abs": _time_s(raw, "imu_t", cmap, 480, notes, "imu")})
    for a in AX:
        c = _pick(raw, f"lg{a}", cmap)
        if c is None:
            raise SystemExit(f"[imu] no low-g accel '{a}' column. Columns: {list(raw.columns)}")
        imu[f"lg{a}"], how = _to_unit(raw[c].to_numpy(float), c, lg_lsb, "g")
        notes.append(f"imu low-g {a}: '{c}' -> g ({how})")
    have_hg = True
    for a in AX:
        c = _pick(raw, f"hg{a}", cmap)
        if c is None:
            have_hg = False
            break
        imu[f"hg{a}"], how = _to_unit(raw[c].to_numpy(float), c, hg_lsb, "g")
        imu[f"hg{a}"] *= hg_scale
        notes.append(f"imu high-g {a}: '{c}' -> g ({how}){f' x {hg_scale:g} (--hg-scale)' if hg_scale != 1 else ''}"
                     " [factor unverified, see hgcal]")
    if not have_hg:
        notes.append("imu: no high-g columns found; accel above +-16 g will be clipped")
    for a in AX:
        c = _pick(raw, f"gy{a}", cmap)
        if c is None:
            raise SystemExit(f"[imu] no gyro '{a}' column. Columns: {list(raw.columns)}")
        imu[f"g{a}"], _ = _to_unit(raw[c].to_numpy(float), c, gyro_lsb, "dps")
    imu["have_hg"] = have_hg

    # merged accel: low-g unless near its full scale, then high-g
    use_hg = np.zeros(len(imu), dtype=bool)
    for a in AX:
        lg = imu[f"lg{a}"].to_numpy()
        if have_hg and accel_src != "lg":
            hg = imu[f"hg{a}"].to_numpy()
            sw = np.abs(lg) > 0.9 * LG_FULL_SCALE_G if accel_src == "merged" else np.ones(len(lg), dtype=bool)
            imu[f"a{a}"] = np.where(sw, hg, lg)
            use_hg |= sw
        else:
            imu[f"a{a}"] = lg
    imu["use_hg"] = use_hg
    notes.append(f"accel source = {accel_src}" + (" (low-g, switching to high-g per axis when |low-g| > %.1f g)"
                 % (0.9 * LG_FULL_SCALE_G) if accel_src == "merged" else ""))
    imu["amag"] = np.sqrt(imu.ax**2 + imu.ay**2 + imu.az**2)

    # --- baro
    baro = pd.DataFrame()
    if "baro" in dfs:
        raw = dfs["baro"]
        pc = _pick(raw, "p", cmap)
        if pc is not None:
            baro["t_abs"] = _time_s(raw, "baro_t", cmap, 25, notes, "baro")
            v = raw[pc].to_numpy(float)
            m = np.nanmedian(v)
            if m > 1e6:
                v, how = v / 4096.0 * 100.0, "raw LPS22DF counts / 4096 -> hPa -> Pa"
            elif m > 2e4:
                how = "Pa"
            else:
                v, how = v * 100.0, "hPa -> Pa"
            notes.append(f"baro: '{pc}' interpreted as {how} (median {np.nanmedian(v):.0f} Pa)")
            baro["p"] = v

    # --- GPS
    gps = pd.DataFrame()
    if "gps" in dfs and len(dfs["gps"]):
        raw = dfs["gps"]
        gps["t_abs"] = _time_s(raw, "gps_t", cmap, 1 / 0.056, notes, "gps")
        for key in ("lat", "lon"):
            c = _pick(raw, key, cmap)
            if c is not None:
                v = raw[c].to_numpy(float)
                if np.nanmax(np.abs(v)) > 1000:
                    v = v * 1e-7
                    notes.append(f"gps {key}: '{c}' scaled by 1e-7 (UBX deg*1e-7)")
                gps[key] = v
        for key, out in (("alt", "alt"), ("gspeed", "gspeed"), ("veld", "veld")):
            c = _pick(raw, key, cmap)
            if c is not None:
                v = raw[c].to_numpy(float)
                if _is_int_valued(v) and not c.lower().endswith(("_m", "_mps")):
                    v = v * 1e-3
                    notes.append(f"gps {out}: '{c}' integer -> assumed UBX mm (mm/s) -> m (m/s). Check!")
                gps[out] = v
        for key in ("fix", "nsv"):
            c = _pick(raw, key, cmap)
            if c is not None:
                gps[key] = raw[c].to_numpy(float)

    # --- logger health
    status = pd.DataFrame()
    if "status" in dfs and len(dfs["status"]):
        raw = dfs["status"]
        status = raw.copy()
        status["t_abs"] = _time_s(raw, "imu_t", cmap, 1, notes, "status")

    # --- events + launch-relative time
    ev = detect_events(imu, baro, launch_g)
    t0 = ev.get("launch", {"t_abs": float(imu.t_abs.iloc[0])})["t_abs"] if ev else float(imu.t_abs.iloc[0])
    for df in (imu, baro, gps, status):
        if len(df):
            df["t"] = df["t_abs"] - t0
    if len(baro):
        pad = baro[baro.t < -0.5].p
        p0 = pad.median() if len(pad) > 10 else baro.p.iloc[:25].median()
        baro["alt"] = 44330.0 * (1.0 - (baro.p / p0) ** 0.190295)
        notes.append(f"baro altitude is relative to pad pressure {p0:.0f} Pa (ISA, no temperature correction)")
        if "apogee" not in ev:
            i = baro.alt.rolling(13, center=True, min_periods=1).median().idxmax()
            ev["apogee"] = {"t_abs": float(baro.t_abs[i]), "note": f"baro max {baro.alt[i]:.0f} m"}
    if len(gps) and "alt" in gps and len(baro):
        gps["alt_rel"] = gps["alt"] - gps[gps.t < -0.5].alt.median() if (gps.t < -0.5).any() else gps["alt"] - gps.alt.iloc[0]
    for k, e in ev.items():
        e["t"] = e["t_abs"] - t0
    return Flight(imu=imu, baro=baro, gps=gps, status=status, events=ev, notes=notes, t_launch_abs=t0)


def detect_events(imu, baro, launch_g=2.5):
    """Crude, threshold-based events. Good enough to line plots up; the EKF should do its own detection."""
    t = imu.t_abs.to_numpy()
    fs = 1.0 / np.nanmedian(np.diff(t))
    am = pd.Series(imu.amag.to_numpy()).rolling(max(int(fs * 0.02), 1), center=True, min_periods=1).mean().to_numpy()
    ev = {}
    n = max(int(fs * 0.1), 1)                       # must stay above threshold for 0.1 s
    above = pd.Series(am > launch_g).rolling(n).sum().to_numpy() >= n
    if not above.any():
        return ev
    i0 = int(np.argmax(above)) - n + 1
    ev["launch"] = {"t_abs": float(t[i0]), "note": f"|a| > {launch_g} g"}
    peak = i0 + int(np.argmax(am[i0:i0 + int(fs * 10)]))
    # burnout = steepest sustained drop in |a| after the boost peak (drag keeps |a| > 0 at high speed,
    # so an absolute threshold finds the wrong place)
    sm = pd.Series(am).rolling(max(int(fs * 0.05), 1), center=True, min_periods=1).mean().to_numpy()
    k = max(int(fs * 0.1), 1)
    seg_end = min(peak + int(fs * 15), len(sm) - k)
    if seg_end > peak:
        drop = sm[peak:seg_end] - sm[peak + k:seg_end + k]
        j = int(np.argmax(drop))
        if drop[j] > 0.3 * am[peak]:
            ev["burnout"] = {"t_abs": float(t[peak + j + k // 2]),
                             "note": f"steepest |a| drop ({drop[j]:.1f} g in 0.1 s) after peak {am[peak]:.1f} g"}
    return ev


# ---- high-g calibration -----------------------------------------------------------------------
def hg_cal(f: Flight):
    """Compare high-g to low-g where low-g is valid. A ratio != 1 means the high-g g/LSB factor is wrong."""
    if not f.imu.have_hg.iloc[0]:
        print("no high-g columns")
        return
    print("Per axis, using samples where |low-g| is between 3 g and %.1f g (valid, off the pad's 1 g):" % (0.9 * LG_FULL_SCALE_G))
    print("axis   n_pts   lg range     median hg/lg   fit slope   -> multiply high-g by")
    any_ok = False
    for a in AX:
        lg, hg = f.imu[f"lg{a}"].to_numpy(), f.imu[f"hg{a}"].to_numpy()
        m = (np.abs(lg) > 3) & (np.abs(lg) < 0.9 * LG_FULL_SCALE_G)
        if m.sum() < 50:
            print(f"  {a}    {m.sum():6d}   (not enough samples in that band on this axis)")
            continue
        r = np.median(hg[m] / lg[m])
        slope = float(np.dot(lg[m], hg[m]) / np.dot(lg[m], lg[m]))
        print(f"  {a}    {m.sum():6d}   {lg[m].min():5.1f}..{lg[m].max():5.1f}   {r:10.4f}   {slope:9.4f}   {1/slope:10.4f}")
        any_ok = True
    peak = f.imu[["lgx", "lgy", "lgz"]].abs().max().max()
    print(f"peak |low-g| seen: {peak:.1f} g (low-g full scale {LG_FULL_SCALE_G:g} g)")
    if not any_ok:
        print("No axis reached 3 g inside the low-g range, so the scale cannot be checked from this flight; "
              "bench-test it instead (see notes).")
    print("Trust only axes with real signal (the boost axis). A hg/lg of ~1.00 = factor OK; if it is ~2, 3, 0.5 ... "
          "use --hg-scale <1/slope> and fix the factor in atlas_parse.py.")


def _decimate_df(df, cols, max_points):
    """Keep the min and max of each column in every time bin (a common index set), so spikes survive."""
    n = len(df)
    if not max_points or n <= max_points:
        return df
    nb = max(max_points // (2 * len(cols)), 1)
    edges = np.linspace(0, n, nb + 1).astype(int)
    arrs = [df[c].to_numpy() for c in cols]
    idx = set()
    for a, b in zip(edges[:-1], edges[1:]):
        if b <= a:
            continue
        idx.add(a)
        for arr in arrs:
            seg = arr[a:b]
            idx.add(a + int(np.argmax(seg)))
            idx.add(a + int(np.argmin(seg)))
    return df.iloc[sorted(idx)]


# ---- plotting -----------------------------------------------------------------------------------
def plot_flight(f: Flight, out="flight.html", overlays=None, title="Atlas flight", cdn=False,
                max_points=40000, t_range=None):
    # NB: decimation keeps min/max of the merged accel + gyro; low/high-g overlays ride on the same samples
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    overlays = overlays or {}
    rows = ["Accel (g)", "Gyro (dps)", "Altitude (m)", "Vertical speed (m/s)", "GPS", "Logger health"]
    fig = make_subplots(rows=len(rows), cols=1, shared_xaxes=True, vertical_spacing=0.035,
                        subplot_titles=rows, specs=[[{}]] * 4 + [[{"secondary_y": True}], [{}]])
    col = {"x": "#1f77b4", "y": "#2ca02c", "z": "#d62728"}
    im = f.imu
    if t_range:
        im = im[(im.t >= t_range[0]) & (im.t <= t_range[1])]
    im = _decimate_df(im, ["ax", "ay", "az", "gx", "gy", "gz"], max_points)
    for a in AX:
        fig.add_trace(go.Scattergl(x=im.t, y=im[f"a{a}"], name=f"a{a}", line=dict(color=col[a], width=1),
                                   legendgroup="acc"), row=1, col=1)
    for a in AX:
        fig.add_trace(go.Scattergl(x=im.t, y=im[f"lg{a}"], name=f"low-g {a}", visible="legendonly",
                                   line=dict(color=col[a], width=1, dash="dot"), legendgroup="lg"), row=1, col=1)
        if "hg" + a in im:
            fig.add_trace(go.Scattergl(x=im.t, y=im[f"hg{a}"], name=f"high-g {a}", visible="legendonly",
                                       line=dict(color=col[a], width=1, dash="dash"), legendgroup="hg"), row=1, col=1)
    fig.add_trace(go.Scattergl(x=im.t, y=im.use_hg.astype(float) * LG_FULL_SCALE_G, name="high-g in use (axis any)",
                               visible="legendonly", line=dict(color="gray", width=1)), row=1, col=1)
    fig.add_trace(go.Scattergl(x=im.t, y=im.amag, name="|a|", line=dict(color="black", width=1), visible="legendonly",
                               legendgroup="acc"), row=1, col=1)
    for a in AX:
        fig.add_trace(go.Scattergl(x=im.t, y=im[f"g{a}"], name=f"g{a}", line=dict(color=col[a], width=1),
                                   legendgroup="gyro"), row=2, col=1)
    if len(f.baro):
        b = f.baro
        if t_range:
            b = b[(b.t >= t_range[0]) & (b.t <= t_range[1])]
        fig.add_trace(go.Scattergl(x=b.t, y=b.alt, name="baro alt", line=dict(color="#9467bd")), row=3, col=1)
        w = 13
        vel = np.gradient(b.alt.rolling(w, center=True, min_periods=1).mean().to_numpy(), b.t.to_numpy())
        fig.add_trace(go.Scattergl(x=b.t, y=vel, name="baro vel (smoothed d/dt)", line=dict(color="#9467bd")), row=4, col=1)
    g = f.gps
    if len(g) and "alt_rel" in g:
        fig.add_trace(go.Scattergl(x=g.t, y=g.alt_rel, name="GPS alt", mode="lines+markers", marker=dict(size=3),
                                   line=dict(color="#ff7f0e")), row=3, col=1)
    if len(g) and "veld" in g:
        fig.add_trace(go.Scattergl(x=g.t, y=-g.veld, name="GPS vel up (-velD)", mode="lines+markers",
                                   marker=dict(size=3), line=dict(color="#ff7f0e")), row=4, col=1)
    if len(g) and "gspeed" in g:
        fig.add_trace(go.Scattergl(x=g.t, y=g.gspeed, name="GPS ground speed", line=dict(color="#8c564b")), row=5, col=1)
    if len(g) and "nsv" in g:
        fig.add_trace(go.Scattergl(x=g.t, y=g.nsv, name="sats", line=dict(color="gray", dash="dot")), row=5, col=1,
                      secondary_y=True)
    if len(g) and "fix" in g:
        fig.add_trace(go.Scattergl(x=g.t, y=g.fix, name="fix type", line=dict(color="red", dash="dot")), row=5, col=1,
                      secondary_y=True)
    st = f.status
    for c_, colr in (("blocks_dropped", "red"), ("sd_errors", "black"), ("queue", "gray")):
        if len(st) and c_ in st:
            fig.add_trace(go.Scattergl(x=st.t, y=st[c_], name=c_, line=dict(color=colr)), row=6, col=1)
    # estimator overlays: {"EKF": {"alt": (t, y), "vel": (t, y), "<anything>": (t, y)}}
    palette = ["#e377c2", "#17becf", "#bcbd22", "#7f7f7f"]
    for k, (name, ser) in enumerate(overlays.items()):
        c = palette[k % len(palette)]
        for key, rown in (("alt", 3), ("vel", 4)):
            if key in ser:
                fig.add_trace(go.Scattergl(x=ser[key][0], y=ser[key][1], name=f"{name} {key}",
                                           line=dict(color=c, width=2)), row=rown, col=1)
    for k, e in f.events.items():
        fig.add_vline(x=e["t"], line=dict(color="gray", dash="dash", width=1))
        fig.add_annotation(x=e["t"], y=1.0, yref="paper", text=k, showarrow=False, yanchor="bottom",
                           font=dict(size=11))
    fig.update_layout(height=1250, title=title, hovermode="x unified", template="plotly_white",
                      legend=dict(orientation="h", y=-0.04))
    fig.update_xaxes(title_text="time since launch (s)", row=len(rows), col=1)
    fig.write_html(out, include_plotlyjs="cdn" if cdn else True)
    return out


def load_overlay(path):
    df = pd.read_csv(path)
    ser = {}
    for key in ("alt", "vel"):
        if key in df:
            ser[key] = (df["t"].to_numpy(), df[key].to_numpy())
    return ser


# ---- CLI ----------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["cols", "sessions", "plot", "hgcal"])
    ap.add_argument("csvdir")
    ap.add_argument("-o", "--out", default="flight.html")
    ap.add_argument("--map", help="JSON file {key: column_name} overriding the column guesses")
    ap.add_argument("--session", help="session id as it appears in the session column")
    ap.add_argument("--lg-lsb", type=float, default=LG_G_PER_LSB)
    ap.add_argument("--hg-lsb", type=float, default=HG_G_PER_LSB, help="g per LSB for the high-g accel")
    ap.add_argument("--accel", choices=["merged", "lg", "hg"], default="merged",
                    help="accel used for the main trace/events: merged (default), low-g only, or high-g only")
    ap.add_argument("--hg-scale", type=float, default=1.0,
                    help="multiplier applied to the high-g accel (use the hgcal result)")
    ap.add_argument("--gyro-lsb", type=float, default=GYRO_DPS_PER_LSB)
    ap.add_argument("--launch-g", type=float, default=2.5)
    ap.add_argument("--overlay", action="append", default=[], metavar="NAME=FILE.csv",
                    help="estimator output csv with columns t [launch-relative s], alt, vel")
    ap.add_argument("--max-points", type=int, default=40000,
                    help="max IMU points drawn (min/max kept per bin so spikes survive); 0 = draw everything")
    ap.add_argument("--t-range", type=float, nargs=2, metavar=("T0", "T1"),
                    help="only plot this launch-relative window, seconds")
    ap.add_argument("--cdn", action="store_true", help="load plotly.js from CDN (smaller file, needs internet)")
    a = ap.parse_args()
    cmap = json.load(open(a.map)) if a.map else {}

    if a.cmd == "cols":
        for n in ("imu", "baro", "gps", "status"):
            p = Path(a.csvdir) / f"{n}.csv"
            if p.exists():
                df = pd.read_csv(p, nrows=3)
                print(f"{n}.csv: {list(df.columns)}")
        return

    if a.cmd == "sessions":
        print(session_table(Path(a.csvdir) / "imu.csv").to_string())
        return

    f = load_flight(a.csvdir, cmap, a.session, a.lg_lsb, a.hg_lsb, a.gyro_lsb, a.launch_g, a.hg_scale, a.accel)
    print("\n".join("  - " + n for n in f.notes))
    print("events:", {k: f"{e['t']:+.2f} s ({e['note']})" for k, e in f.events.items()})
    if a.cmd == "hgcal":
        hg_cal(f)
    else:
        ov = {}
        for s in a.overlay:
            name, _, fn = s.partition("=")
            ov[name] = load_overlay(fn)
        print("wrote", plot_flight(f, a.out, ov, cdn=a.cdn, max_points=a.max_points, t_range=a.t_range))


if __name__ == "__main__":
    sys.exit(main())
