#!/usr/bin/env python3
"""Run the Atlas ESKF over logged flight CSVs.

  python3 run_offline.py ~/Documents/arc/atlas-v4-code/flight-1_csv
  python3 run_offline.py DIR --gps off --pre 90 --post 220 --out ekf_out

Writes  OUT/ekf_states.csv, ekf_updates.csv, ekf_meta.json, ekf_overview.png
"""
import argparse, json, os, sys, time
import numpy as np
import pandas as pd
from eskf import FlightFilter, Config, Params, G0
from flightdata import load_flight, find_launch


def run(d, out, pre=60.0, post=200.0, t0=None, t1=None, launch_t=None,
        gps="off", decim=9, yaw_deg=0.0, verbose=True):
    t_load = time.time()
    D = load_flight(d)
    if verbose:
        print(f"loaded {len(D['imu_t']):,} IMU / {len(D['baro_t']):,} baro / "
              f"{len(D['gps_t']):,} GPS samples in {time.time()-t_load:.1f} s")
    if launch_t is None and (t0 is None or t1 is None):
        launch_t = find_launch(D["imu_t"], D["imu_f"])
        if launch_t is None:
            sys.exit("could not auto-detect launch; pass --t0/--t1")
        if verbose:
            print(f"coarse launch detect: t = {launch_t:.2f} s")
    if t0 is None: t0 = launch_t - pre
    if t1 is None: t1 = launch_t + post
    t0 = max(t0, D["imu_t"][0]); t1 = min(t1, D["imu_t"][-1])

    i0, i1 = np.searchsorted(D["imu_t"], [t0, t1])
    b0, b1 = np.searchsorted(D["baro_t"], [t0, t1])
    g0, g1 = np.searchsorted(D["gps_t"], [t0, t1])
    ti, fi, wi = D["imu_t"][i0:i1], D["imu_f"][i0:i1], D["imu_w"][i0:i1]
    rx, rg = D["imu_rail_xl"][i0:i1], D["imu_rail_gy"][i0:i1]
    tb, pb = D["baro_t"][b0:b1], D["baro_p"][b0:b1]
    tg = D["gps_t"][g0:g1]
    gf, gn = D["gps_fix"][g0:g1], D["gps_nsv"][g0:g1]
    gh, gv = D["gps_h"][g0:g1], D["gps_vd"][g0:g1]
    ga, gs = D["gps_vacc"][g0:g1], D["gps_sacc"][g0:g1]

    ff = FlightFilter(Config(gps_mode=gps, yaw_deg=yaw_deg), Params())
    rows = []
    jb = jg = 0
    nb, ng = len(tb), len(tg)
    t_run = time.time()
    for k in range(len(ti)):
        t = ti[k]
        while jb < nb and tb[jb] <= t:
            ff.feed_baro(tb[jb], pb[jb]); jb += 1
        while jg < ng and tg[jg] <= t:
            ff.feed_gps(tg[jg], gf[jg], gn[jg], gh[jg], gv[jg], ga[jg], gs[jg]); jg += 1
        ff.feed_imu(t, fi[k], wi[k], bool(rx[k]), bool(rg[k]))
        if ff.phase != "ALIGN" and k % decim == 0:
            rows.append(ff.snapshot())
    if verbose:
        print(f"filter ran {len(ti):,} IMU steps in {time.time()-t_run:.1f} s "
              f"({(time.time()-t_run)/len(ti)*1e6:.0f} us/step)")

    os.makedirs(out, exist_ok=True)
    st = pd.DataFrame(rows)
    st.to_csv(os.path.join(out, "ekf_states.csv"), index=False, float_format="%.6g")
    up = pd.DataFrame(ff.updates, columns=["t", "kind", "nis", "accepted"])
    up.to_csv(os.path.join(out, "ekf_updates.csv"), index=False, float_format="%.5g")

    # reference signals for the plots (not used by the filter)
    base_alt = 44330.0 * (1 - (pb / ff.p0) ** 0.190294957)
    meta = dict(source=os.path.abspath(d), window=[float(t0), float(t1)],
                nose_body=[float(x) for x in ff.nose_b], gps_mode=gps,
                p0_pa=ff.p0, h0_gps=ff.h0,
                events=[dict(t=float(e[0]), name=e[1], info=e[2]) for e in ff.events])
    with open(os.path.join(out, "ekf_meta.json"), "w") as fh:
        json.dump(meta, fh, indent=1)

    if verbose:
        print("\nevents:")
        for e in ff.events:
            print(f"  t={e[0]:9.3f}  {e[1]:8s} {e[2]}")
        n = up.groupby(["kind", "accepted"]).size().unstack(fill_value=0)
        print("\naiding updates (accepted 1 / rejected 0):\n" + n.to_string())
    return dict(states=st, updates=up, meta=meta, ff=ff,
                ref=dict(baro_t=tb, baro_alt=base_alt, gps_t=tg, gps_fix=gf, gps_h=gh,
                         gps_vd=gv, gps_nsv=gn, h0=ff.h0))


def overview_plot(res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    st, ref, meta = res["states"], res["ref"], res["meta"]
    t = st["t"].to_numpy()
    ev = meta["events"]
    fig, ax = plt.subplots(5, 1, figsize=(11, 13), sharex=True)
    ax[0].plot(t, st["alt_m"], label="EKF", lw=1.6)
    ax[0].plot(ref["baro_t"], ref["baro_alt"], ".", ms=2, label="baro (ISA, raw)", alpha=.6)
    good = ref["gps_fix"] == 3
    if ref["h0"] is not None:
        ax[0].plot(ref["gps_t"][good], (ref["gps_h"] - ref["h0"])[good], ".", ms=2,
                   label="GPS height (raw)", alpha=.6)
    ax[0].set_ylabel("altitude AGL (m)"); ax[0].legend(loc="upper right")
    ax[1].plot(t, -st["vd"], label="EKF v_up")
    ax[1].plot(ref["gps_t"][good], -ref["gps_vd"][good], ".", ms=2, label="GPS v_up (raw)", alpha=.6)
    ax[1].set_ylabel("vertical speed (m/s)"); ax[1].legend(loc="upper right")
    ax[2].plot(t, st["f_nose_g"], lw=.8)
    ax[2].set_ylabel("accel along nose (g)"); ax[2].set_ylim(-3, 16)
    ax[3].plot(t, st["tilt_deg"], label="tilt from vertical")
    ax3 = ax[3].twinx(); ax3.plot(t, st["spin_dps"], c="C1", lw=.8, label="roll rate (dps)")
    ax[3].set_ylabel("tilt (deg)"); ax3.set_ylabel("spin about nose (dps)")
    ax[4].plot(t, st["bgx_dps"], label="gyro bias x"); ax[4].plot(t, st["bgy_dps"], label="y")
    ax[4].plot(t, st["bgz_dps"], label="z"); ax[4].set_ylabel("gyro bias (dps)")
    ax[4].legend(loc="upper right"); ax[4].set_xlabel("t (s)")
    for a in ax:
        for e in ev:
            if e["name"] in ("launch", "burnout", "apogee", "landed", "shock"):
                a.axvline(e["t"], c="k", ls=":", lw=.7)
        a.grid(alpha=.3)
    for e in ev:
        if e["name"] in ("launch", "burnout", "apogee", "landed"):
            ax[0].text(e["t"], ax[0].get_ylim()[1]*0.02, e["name"], rotation=90, va="bottom", fontsize=8)
    fig.suptitle(f"Atlas ESKF  ({os.path.basename(meta['source'])},  GPS: {meta['gps_mode']})")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_dir", help="directory with imu.csv baro.csv gps.csv")
    ap.add_argument("--out", default=None, help="output dir (default: <csv_dir>/ekf_out)")
    ap.add_argument("--pre", type=float, default=60.0, help="s of pad data before launch")
    ap.add_argument("--post", type=float, default=200.0, help="s after launch")
    ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
    ap.add_argument("--launch-t", type=float, help="skip auto launch detect")
    ap.add_argument("--gps", choices=["off", "vertical"], default="off")
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--decim", type=int, default=9, help="log every Nth IMU step (~50 Hz)")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    out = a.out or os.path.join(a.csv_dir, "ekf_out")
    res = run(a.csv_dir, out, a.pre, a.post, a.t0, a.t1, a.launch_t, a.gps, a.decim, a.yaw_deg)
    if not a.no_plot:
        p = os.path.join(out, "ekf_overview.png")
        overview_plot(res, p)
        print("\nplot:", p)
    print("states:", os.path.join(out, "ekf_states.csv"))


if __name__ == "__main__":
    main()
