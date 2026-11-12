#!/usr/bin/env python3
"""Atlas orientation viewer (matplotlib): OFFLINE playback of an EKF run.

For LIVE use atlas_live.py (tabs, raw plots, GPS/altitude, reads the new text stream). The
--port / --replay options below are the older single-window live path and use an obsolete CSV-ish
line format; keep them only for quick matplotlib-only tests.


OFFLINE (after run_offline.py):
    python3 viewer.py --offline flight-1_csv/ekf_out
LIVE over USB serial (needs pyserial and firmware that prints I/B/G lines, see README):
    python3 viewer.py --port /dev/serial/by-id/usb-STMicroelectronics_...
    python3 viewer.py --port /dev/ttyACM0 --flight        # flight logic on (launch detect etc.)
LIVE-style test without hardware: replay a flight CSV in real time through the same code path
    python3 viewer.py --replay flight-1_csv --start 1740 --speed 1
Still image / headless (no window):
    python3 viewer.py --offline DIR --at 1760 --snapshot frame.png

Keys (offline): space play/pause, left/right +-1 s (shift: +-10 s), up/down speed x2, e next event
Keys (live):    r re-align (hold the board still)
"""
import argparse, glob, json, os, queue, sys, threading, time
import numpy as np

from eskf import FlightFilter, Config, Params, q2R, G0

# display frame: x = East, y = North, z = Up  (from nav NED)
T_DISP = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


# ------------------------------------------------------------------ geometry
def rocket_mesh(L=1.0, r=0.075, n=14):
    """Polygons in a local frame with +z = nose. Returns (list of (k,3) arrays, colours)."""
    th = np.linspace(0, 2*np.pi, n, endpoint=False)
    ring = lambda z, rr=r: np.stack([rr*np.cos(th), rr*np.sin(th), np.full(n, z)], 1)
    z_tail, z_sh, z_tip = -0.5*L, 0.5*L*0.45, 0.5*L
    a, b = ring(z_tail), ring(z_sh)
    polys, cols = [], []
    for i in range(n):
        j = (i+1) % n
        polys.append(np.array([a[i], a[j], b[j], b[i]])); cols.append("#c8ccd2")
        polys.append(np.array([b[i], b[j], [0, 0, z_tip]])); cols.append("#d9534f")
    for k in range(4):
        c, s = np.cos(k*np.pi/2), np.sin(k*np.pi/2)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        fin = np.array([[r, 0, z_tail+0.28], [r, 0, z_tail], [r+0.17, 0, z_tail]])
        polys.append((Rz @ fin.T).T); cols.append("#3b6fb6")
    return polys, cols


def frame_from_nose(nose_b):
    """C: local (z=nose) -> body."""
    z = np.asarray(nose_b, float); z /= np.linalg.norm(z)
    t = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = t - (t @ z)*z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)


# --------------------------------------------------------------- common view
class OrientationView:
    def __init__(self, nose_b, title, live_panels=False):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig = plt.figure(figsize=(12.5, 7))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.35, 1], height_ratios=[1, 1],
                                   left=0.02, right=0.90, top=0.95, bottom=0.12,
                                   wspace=0.12, hspace=0.28)
        self.ax = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax1 = self.fig.add_subplot(gs[0, 1])
        self.ax2 = self.fig.add_subplot(gs[1, 1])
        self.fig.suptitle(title, fontsize=11)
        ax = self.ax
        ax.set_xlim(-0.9, 0.9); ax.set_ylim(-0.9, 0.9); ax.set_zlim(-0.9, 0.9)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=18, azim=-58)
        ax.set_xlabel("E"); ax.set_ylabel("N"); ax.set_zlabel("Up")
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        self.polys0, self.cols = rocket_mesh()
        self.C = frame_from_nose(nose_b)
        self.coll = Poly3DCollection(self.polys0, facecolors=self.cols, edgecolors="k", linewidths=0.3)
        ax.add_collection3d(self.coll)
        for g in np.linspace(-0.8, 0.8, 5):          # ground grid
            ax.plot([g, g], [-0.8, 0.8], [-0.85, -0.85], c="#bbbbbb", lw=0.5)
            ax.plot([-0.8, 0.8], [g, g], [-0.85, -0.85], c="#bbbbbb", lw=0.5)
        ax.text(0, 0.95, -0.85, "N (arbitrary)", ha="center", fontsize=8, color="#666666")
        ax.text(0.95, 0, -0.85, "E", ha="center", fontsize=8, color="#666666")
        self.axl = [ax.plot([0, 0], [0, 0], [0, 0], c=c, lw=2)[0] for c in ("r", "g", "b")]
        for c, n in zip(("r", "g", "b"), ("x", "y", "z")):
            ax.plot([], [], c=c, lw=2, label=f"body {n}")
        ax.legend(loc="lower left", fontsize=8, frameon=False)
        self.info = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, va="top",
                              family="monospace", fontsize=9)
        self.banner = ax.text2D(0.5, 0.5, "", transform=ax.transAxes, ha="center",
                                fontsize=16, color="#d9534f")

    def set_attitude(self, R_nb):
        M = T_DISP @ R_nb @ self.C
        self.coll.set_verts([(M @ p.T).T for p in self.polys0])
        self.coll.set_facecolor(self.cols)
        for i, l in enumerate(self.axl):
            e = T_DISP @ R_nb[:, i] * 0.6
            l.set_data_3d([0, e[0]], [0, e[1]], [0, e[2]])


# ----------------------------------------------------------------- offline
class OfflinePlayer:
    def __init__(self, d):
        import pandas as pd
        self.st = pd.read_csv(os.path.join(d, "ekf_states.csv"))
        self.meta = json.load(open(os.path.join(d, "ekf_meta.json")))
        s = self.st
        self.t = s["t"].to_numpy()
        self.q = s[["qw", "qx", "qy", "qz"]].to_numpy()
        self.events = [e for e in self.meta["events"] if e["name"] != "shock"]
        self.view = OrientationView(self.meta["nose_body"],
                                    f"Atlas ESKF playback: {os.path.basename(self.meta['source'])}")
        v = self.view
        v.ax1.plot(self.t, s["alt_m"], c="C0"); v.ax1.set_ylabel("altitude (m)", color="C0")
        a2 = v.ax1.twinx(); a2.plot(self.t, -s["vd"], c="C1", lw=.8); a2.set_ylabel("v up (m/s)", color="C1")
        v.ax2.plot(self.t, s["tilt_deg"], c="C2"); v.ax2.set_ylabel("tilt (deg)", color="C2")
        a3 = v.ax2.twinx(); a3.plot(self.t, s["f_nose_g"], c="C3", lw=.6); a3.set_ylabel("accel along nose (g)", color="C3")
        a3.set_ylim(-3, 16)
        v.ax2.set_xlabel("t (s)")
        for a in (v.ax1, v.ax2):
            a.grid(alpha=.3)
            for e in self.events:
                a.axvline(e["t"], c="k", ls=":", lw=.7)
        self.cur = [a.axvline(self.t[0], c="r", lw=1.2) for a in (v.ax1, v.ax2)]
        from matplotlib.widgets import Slider
        sa = v.fig.add_axes([0.08, 0.04, 0.6, 0.03])
        self.slider = Slider(sa, "t (s)", self.t[0], self.t[-1], valinit=self.t[0])
        self.slider.on_changed(lambda val: self.seek(val, from_slider=True))
        self.sim_t = self.t[0]; self.speed = 1.0; self.playing = False
        self._wall = time.monotonic()
        v.fig.canvas.mpl_connect("key_press_event", self.on_key)
        v.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.draw()

    def seek(self, t, from_slider=False):
        self.sim_t = float(np.clip(t, self.t[0], self.t[-1]))
        if not from_slider:
            self.slider.eventson = False; self.slider.set_val(self.sim_t); self.slider.eventson = True
        self.draw()

    def on_click(self, ev):
        if ev.inaxes in (self.view.ax1, self.view.ax2) and ev.xdata is not None:
            self.seek(ev.xdata)

    def on_key(self, ev):
        k = ev.key
        if k == " ": self.playing = not self.playing
        elif k == "right": self.seek(self.sim_t + 1)
        elif k == "left": self.seek(self.sim_t - 1)
        elif k == "shift+right": self.seek(self.sim_t + 10)
        elif k == "shift+left": self.seek(self.sim_t - 10)
        elif k == "up": self.speed = min(self.speed*2, 64)
        elif k == "down": self.speed = max(self.speed/2, 1/16)
        elif k == "e":
            nxt = [e["t"] for e in self.events if e["t"] > self.sim_t + 0.01]
            self.seek(nxt[0] if nxt else self.events[0]["t"])

    def step(self):
        now = time.monotonic(); dt = now - self._wall; self._wall = now
        if self.playing:
            self.sim_t += dt*self.speed
            if self.sim_t >= self.t[-1]:
                self.sim_t = self.t[-1]; self.playing = False
            self.slider.eventson = False; self.slider.set_val(self.sim_t); self.slider.eventson = True
            self.draw()

    def draw(self):
        i = int(np.clip(np.searchsorted(self.t, self.sim_t), 0, len(self.t)-1))
        r = self.st.iloc[i]
        self.view.set_attitude(q2R(self.q[i]))
        self.view.info.set_text(
            f"t={r.t:9.2f} s   {r.phase}\n"
            f"alt {r.alt_m:7.1f} m   v_up {-r.vd:7.1f} m/s\n"
            f"tilt {r.tilt_deg:6.1f} deg   spin {r.spin_dps:7.0f} dps\n"
            f"a_nose {r.f_nose_g:6.2f} g   att sigma {r.sig_att_deg:4.1f} deg\n"
            f"speed x{self.speed:g}  {'PLAY' if self.playing else 'PAUSE'}")
        for c in self.cur:
            c.set_xdata([r.t, r.t])
        self.view.fig.canvas.draw_idle()


# -------------------------------------------------------------------- live
def parse_line(line):
    """I,t,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps | B,t,pressure_pa | G,t,fix,nsv,hmsl,vd,vacc,sacc"""
    p = line.strip().split(",")
    try:
        k = p[0]
        if k == "I" and len(p) >= 8:
            t = float(p[1]); a = [float(x)/1000 for x in p[2:5]]; w = [float(x)/1000 for x in p[5:8]]
            return ("I", t, a, w, max(abs(x) for x in a) >= 15.98, max(abs(x) for x in w) >= 2290.0)
        if k == "B" and len(p) >= 3:
            return ("B", float(p[1]), float(p[2]))
        if k == "G" and len(p) >= 8:
            return ("G", float(p[1]), int(p[2]), int(p[3]), float(p[4]), float(p[5]), float(p[6]), float(p[7]))
    except ValueError:
        pass
    return None


def serial_thread(port, q, stop):
    try:
        import serial
    except ImportError:
        sys.exit("pip install pyserial")
    s = serial.Serial(port, 115200, timeout=0.2)
    buf = b""
    while not stop.is_set():
        buf += s.read(4096)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            ev = parse_line(ln.decode("ascii", "ignore"))
            if ev:
                try: q.put_nowait(ev)
                except queue.Full: pass
    s.close()


def replay_thread(d, start, speed, q, stop):
    from flightdata import load_flight
    D = load_flight(d)
    i = int(np.searchsorted(D["imu_t"], start)); b = int(np.searchsorted(D["baro_t"], start))
    g = int(np.searchsorted(D["gps_t"], start))
    wall0 = time.monotonic(); n = len(D["imu_t"])
    while not stop.is_set() and i < n:
        tgt = start + (time.monotonic() - wall0)*speed
        while i < n and D["imu_t"][i] <= tgt:
            while b < len(D["baro_t"]) and D["baro_t"][b] <= D["imu_t"][i]:
                q.put(("B", D["baro_t"][b], D["baro_p"][b])); b += 1
            while g < len(D["gps_t"]) and D["gps_t"][g] <= D["imu_t"][i]:
                q.put(("G", D["gps_t"][g], D["gps_fix"][g], D["gps_nsv"][g], D["gps_h"][g],
                       D["gps_vd"][g], D["gps_vacc"][g], D["gps_sacc"][g])); g += 1
            q.put(("I", D["imu_t"][i], D["imu_f"][i], D["imu_w"][i],
                   bool(D["imu_rail_xl"][i]), bool(D["imu_rail_gy"][i]))); i += 1
        time.sleep(0.005)


class LivePlayer:
    def __init__(self, source_desc, q, cfg):
        self.q = q; self.cfg = cfg; self.src = source_desc
        self.ff = FlightFilter(cfg)
        self.view = None
        self.hist = {k: [] for k in ("t", "f", "w", "tilt")}
        self.n_in = 0; self.t_wall0 = time.monotonic(); self.rate = 0.0
        self._last_t = None; self._rate_n = 0; self._rate_t = time.monotonic()
        self.view = OrientationView(self.ff.nose_b, f"Atlas LIVE: {source_desc}")
        v = self.view
        v.ax1.set_ylabel("|accel| (g)"); v.ax2.set_ylabel("|gyro| (dps)"); v.ax2.set_xlabel("t (s, last 30 s)")
        self.l1, = v.ax1.plot([], [], c="C0", lw=.8); self.l2, = v.ax2.plot([], [], c="C1", lw=.8)
        for a in (v.ax1, v.ax2): a.grid(alpha=.3)
        v.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def on_key(self, ev):
        if ev.key == "r":
            self.reset()

    def reset(self):
        self.ff = FlightFilter(self.cfg)
        for k in self.hist: self.hist[k].clear()
        self._last_t = None

    def _feed(self, ev):
        k = ev[0]
        if k == "I":
            _, t, f, w, rx, rg = ev
            if self._last_t is not None and t < self._last_t - 1.0:
                self.reset()                      # MCU reset
            self._last_t = t
            self.ff.feed_imu(t, f, w, rx, rg)
            self._rate_n += 1
            if self.ff.phase != "ALIGN" and self._rate_n % 5 == 0:
                h = self.hist
                h["t"].append(t); h["f"].append(float(np.linalg.norm(f))); h["w"].append(float(np.linalg.norm(w)))
        elif k == "B":
            self.ff.feed_baro(ev[1], ev[2])
        elif k == "G":
            self.ff.feed_gps(*ev[1:])

    def step(self):
        n = 0
        while n < 3000:
            try: ev = self.q.get_nowait()
            except queue.Empty: break
            self._feed(ev); n += 1
        now = time.monotonic()
        if now - self._rate_t > 1.0:
            self.rate = self._rate_n/(now - self._rate_t); self._rate_n = 0; self._rate_t = now
        ff = self.ff; v = self.view
        if ff.phase == "ALIGN":
            left = max(0.0, ff.cfg.align_s - (ff.t - (ff._t_start or ff.t)))
            v.banner.set_text(f"aligning - hold still ({left:.1f} s)" if ff._t_start is not None
                              else "waiting for data...")
            v.info.set_text(f"{self.src}\nIMU {self.rate:5.0f} Hz")
        else:
            v.banner.set_text("")
            v.C = frame_from_nose(ff.nose_b)
            v.set_attitude(ff.kf.R)
            s = ff.snapshot()
            nn = ff.kf.R @ ff.nose_b
            az = np.degrees(np.arctan2(nn[1], nn[0]))
            v.info.set_text(
                f"t={s['t']:9.2f} s   {s['phase']}{'  (bench)' if ff.cfg.bench else ''}\n"
                f"nose heading* {az:6.1f} deg   (* arbitrary, no mag)\n"
                f"tilt {s['tilt_deg']:6.1f} deg   spin {s['spin_dps']:7.0f} dps\n"
                f"alt {s['alt_m']:7.1f} m   v_up {-s['vd']:6.1f} m/s\n"
                f"IMU {self.rate:5.0f} Hz   att sigma {s['sig_att_deg']:4.1f} deg   (r = re-align)")
            h = self.hist
            if len(h["t"]) > 3:
                t = np.array(h["t"]); m = t > t[-1] - 30
                if m.sum() > 3:
                    tt = t[m]; self.l1.set_data(tt, np.array(h["f"])[m]); self.l2.set_data(tt, np.array(h["w"])[m])
                    for a, key in ((v.ax1, "f"), (v.ax2, "w")):
                        a.set_xlim(tt[0], tt[-1] + 0.1)
                        y = np.array(h[key])[m]; a.set_ylim(0, max(y.max()*1.1, 1.2 if key == "f" else 10))
        v.fig.canvas.draw_idle()


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--offline", metavar="EKF_OUT_DIR")
    g.add_argument("--port", metavar="SERIAL")
    g.add_argument("--replay", metavar="CSV_DIR", help="replay imu/baro/gps csv in real time as a fake live stream")
    ap.add_argument("--start", type=float, default=None, help="replay start time (s)")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed")
    ap.add_argument("--flight", action="store_true", help="live: enable launch/burnout logic (default is bench mode)")
    ap.add_argument("--at", type=float, help="offline: jump to time")
    ap.add_argument("--snapshot", metavar="PNG", help="save a frame and exit (headless)")
    ap.add_argument("--headless-seconds", type=float, default=0.0, help="live: run N s without a window, then --snapshot")
    a = ap.parse_args()

    import matplotlib
    if a.snapshot or a.headless_seconds:
        matplotlib.use("Agg")

    if a.offline:
        p = OfflinePlayer(a.offline)
        if a.at is not None: p.seek(a.at)
        if a.snapshot:
            p.view.fig.savefig(a.snapshot, dpi=100); print("saved", a.snapshot); return
        anim = matplotlib.animation.FuncAnimation(p.view.fig, lambda i: p.step(), interval=33,
                                                   cache_frame_data=False)
        p.view.plt.show(); return

    q = queue.Queue(maxsize=200000); stop = threading.Event()
    if a.port:
        port = a.port
        if port == "auto":
            c = sorted(glob.glob("/dev/serial/by-id/*STM*") + glob.glob("/dev/ttyACM*"))
            if not c: sys.exit("no serial port found; pass --port")
            port = c[0]
        th = threading.Thread(target=serial_thread, args=(port, q, stop), daemon=True)
        cfg = Config(bench=not a.flight); desc = port
    else:
        start = a.start if a.start is not None else 0.0
        th = threading.Thread(target=replay_thread, args=(a.replay, start, a.speed, q, stop), daemon=True)
        cfg = Config(bench=not a.flight, align_s=5.0); desc = f"replay {os.path.basename(a.replay)} @{a.speed:g}x"
    th.start()
    p = LivePlayer(desc, q, cfg)
    if a.headless_seconds:
        t_end = time.monotonic() + a.headless_seconds
        while time.monotonic() < t_end:
            p.step(); time.sleep(0.03)
        stop.set()
        if a.snapshot:
            p.view.fig.savefig(a.snapshot, dpi=100); print("saved", a.snapshot)
        return
    anim = matplotlib.animation.FuncAnimation(p.view.fig, lambda i: p.step(), interval=33,
                                               cache_frame_data=False)
    p.view.plt.show(); stop.set()


if __name__ == "__main__":
    import matplotlib.animation  # noqa
    main()
