#!/usr/bin/env python3
"""Atlas live viewer (Qt): 3D attitude, raw sensor plots, GPS + altitude, log.

  python3 atlas_live.py --port auto                 # first STM32 CDC port, bench mode
  python3 atlas_live.py --port /dev/ttyACM0 --flight
  python3 atlas_live.py --replay flight-1_csv --start 1745 --speed 1 --flight [--fake-mag]
  python3 atlas_live.py --replay DIR --start 1745 --seconds 12 --snapshot-dir shots   # headless test

Reads the timestamped IMU/MAG/BARO/GPS lines (see stream.py / firmware/usb_stream.c).  Your current
single status line also works (host timestamps, lower quality) and is always shown in the status bar.
"""
import os, sys
if any(a.startswith("--snapshot-dir") for a in sys.argv):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import argparse, glob, html, math, queue, threading, time
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from eskf import FlightFilter, Config
from stream import StreamParser, serial_thread, replay_thread, file_thread
from viewer import rocket_mesh, frame_from_nose, T_DISP

pg.setConfigOptions(antialias=False, background="w", foreground="#222222")
AXC = ["#d62728", "#2ca02c", "#1f77b4"]          # x, y, z
M_PER_DEG_LAT = 111132.92


class Ring:
    """Fixed-size circular buffer of float rows; column 0 is time (monotonic)."""
    def __init__(self, ncols, cap):
        self.a = np.full((cap, ncols), np.nan); self.cap = cap; self.n = 0; self.i = 0; self.ncols = ncols
    def push(self, row):
        self.a[self.i] = row; self.i = (self.i + 1) % self.cap; self.n = min(self.n + 1, self.cap)
    def clear(self): self.n = 0; self.i = 0
    def data(self):
        if self.n < self.cap: return self.a[:self.n]
        return np.concatenate((self.a[self.i:], self.a[:self.i]))
    def since(self, tmin):
        d = self.data()
        if not len(d): return d
        return d[np.searchsorted(d[:, 0], tmin):]


class RocketCanvas(FigureCanvasQTAgg):
    def __init__(self):
        self.fig = Figure(figsize=(6, 5))
        super().__init__(self.fig)
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        ax = self.ax = self.fig.add_subplot(111, projection="3d")
        ax.set_xlim(-0.9, 0.9); ax.set_ylim(-0.9, 0.9); ax.set_zlim(-0.9, 0.9)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=18, azim=-58)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        self.polys0, self.cols = rocket_mesh()
        self.C = frame_from_nose([0, 0, 1])
        self.coll = Poly3DCollection(self.polys0, facecolors=self.cols, edgecolors="k", linewidths=0.3)
        ax.add_collection3d(self.coll)
        for g in np.linspace(-0.8, 0.8, 5):
            ax.plot([g, g], [-0.8, 0.8], [-0.85, -0.85], c="#bbbbbb", lw=0.5)
            ax.plot([-0.8, 0.8], [g, g], [-0.85, -0.85], c="#bbbbbb", lw=0.5)
        ax.text(0, 0.95, -0.85, "N (arbitrary)", ha="center", fontsize=8, color="#666666")
        ax.text(0.95, 0, -0.85, "E", ha="center", fontsize=8, color="#666666")
        self.axl = [ax.plot([0, 0], [0, 0], [0, 0], c=c, lw=2)[0] for c in ("r", "g", "b")]
        for c, n in zip(("r", "g", "b"), "xyz"):
            ax.plot([], [], c=c, lw=2, label=f"body {n}")
        ax.legend(loc="lower left", fontsize=8, frameon=False)
        self.info = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9)
        self.banner = ax.text2D(0.5, 0.5, "", transform=ax.transAxes, ha="center", fontsize=16, color="#d9534f")
        self.set_attitude(np.eye(3))

    def set_attitude(self, R_nb):
        M = T_DISP @ R_nb @ self.C
        self.coll.set_verts([(M @ p.T).T for p in self.polys0]); self.coll.set_facecolor(self.cols)
        for i, l in enumerate(self.axl):
            e = T_DISP @ R_nb[:, i] * 0.6
            l.set_data_3d([0, e[0]], [0, e[1]], [0, e[2]])


def make_plot(title, ylabel, last=False):
    p = pg.PlotItem(title=title)
    p.showGrid(x=True, y=True, alpha=0.3)
    p.setLabel("left", ylabel); p.getAxis("left").enableAutoSIPrefix(False)
    if last: p.setLabel("bottom", "time relative to now (s)")
    p.getAxis("bottom").enableAutoSIPrefix(False)
    return p


class LiveWindow(QtWidgets.QMainWindow):
    def __init__(self, q, desc, cfg, parser, stop):
        super().__init__()
        self.q, self.desc, self.cfg, self.parser, self.stop = q, desc, cfg, parser, stop
        self.setWindowTitle(f"Atlas live - {desc}"); self.resize(1320, 820)
        self.window_s = 30.0; self.paused = False; self.tick_n = 0
        self.ff = FlightFilter(cfg)
        self.imu = Ring(7, 150000); self.mag = Ring(4, 60000)
        self.baro = Ring(3, 40000); self.gps = Ring(5, 80000)
        self.t_latest = 0.0
        self.p0 = None; self.last_p = []; self.origin = None
        self.gps_last = None; self.n_imu = 0; self.n_gps = 0; self.rate_imu = 0.0; self.rate_gps = 0.0
        self.rate_t = time.monotonic(); self.err_msg = ""
        self._build()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(30)

    # ------------------------------------------------------------------ UI
    def _build(self):
        c = QtWidgets.QWidget(); self.setCentralWidget(c)
        v = QtWidgets.QVBoxLayout(c); v.setContentsMargins(4, 4, 4, 0)
        bar = QtWidgets.QHBoxLayout(); v.addLayout(bar)
        bar.addWidget(QtWidgets.QLabel(f"<b>{html.escape(self.desc)}</b>"))
        bar.addStretch(1)
        self.lbl_rate = QtWidgets.QLabel(""); self.lbl_rate.setStyleSheet("font-family: monospace;"); bar.addWidget(self.lbl_rate)
        self.btn_pause = QtWidgets.QPushButton("Pause"); self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(lambda s: setattr(self, "paused", s)); bar.addWidget(self.btn_pause)
        bar.addWidget(QtWidgets.QLabel("Window:"))
        self.cmb = QtWidgets.QComboBox(); self.cmb.addItems(["10 s", "30 s", "60 s", "120 s"]); self.cmb.setCurrentIndex(1)
        self.cmb.currentIndexChanged.connect(lambda i: setattr(self, "window_s", [10, 30, 60, 120][i])); bar.addWidget(self.cmb)
        b = QtWidgets.QPushButton("Re-align attitude"); b.clicked.connect(self.realign); bar.addWidget(b)
        b = QtWidgets.QPushButton("Re-zero alt / GPS"); b.clicked.connect(self.rezero); bar.addWidget(b)
        b = QtWidgets.QPushButton("Clear plots"); b.clicked.connect(self.clear_plots); bar.addWidget(b)
        QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self, activated=self.btn_pause.toggle)

        self.tabs = QtWidgets.QTabWidget(); v.addWidget(self.tabs, 1)
        # --- tab 0: attitude
        self.canvas = RocketCanvas(); self.tabs.addTab(self.canvas, "Attitude")
        # --- tab 1: raw sensors
        gl = pg.GraphicsLayoutWidget(); self.tabs.addTab(gl, "Raw sensors")
        self.p_acc = make_plot("Accelerometer (LSM6DSV80X)", "acceleration [g]")
        self.p_gyr = make_plot("Gyroscope", "angular rate [deg/s]")
        self.p_mag = make_plot("Magnetometer (LIS2MDL)", "magnetic field [µT]", last=True)
        for i, p in enumerate((self.p_acc, self.p_gyr, self.p_mag)):
            gl.addItem(p, row=i, col=0)
            if i: p.setXLink(self.p_acc)
        self.c_acc, self.c_gyr, self.c_mag = [], [], []
        for p, cl in ((self.p_acc, self.c_acc), (self.p_gyr, self.c_gyr), (self.p_mag, self.c_mag)):
            p.addLegend(offset=(-5, 5))
            for col, n in zip(AXC, "xyz"):
                cu = p.plot(pen=pg.mkPen(col, width=1), name=n)
                cu.setDownsampling(auto=True, method="peak"); cu.setClipToView(True); cl.append(cu)
        # --- tab 2: GPS + altitude
        w = QtWidgets.QWidget(); self.tabs.addTab(w, "GPS + altitude")
        h = QtWidgets.QHBoxLayout(w)
        left = QtWidgets.QVBoxLayout(); h.addLayout(left, 1)
        self.gps_info = QtWidgets.QLabel("no GPS data yet"); self.gps_info.setStyleSheet("font-family: monospace;")
        left.addWidget(self.gps_info)
        self.gps_plot = pg.PlotWidget(title="GPS position (metres from first fix)")
        self.gps_plot.setAspectLocked(True); self.gps_plot.showGrid(x=True, y=True, alpha=0.3)
        self.gps_plot.setLabel("bottom", "East [m]"); self.gps_plot.setLabel("left", "North [m]")
        for ax in ("bottom", "left"): self.gps_plot.getAxis(ax).enableAutoSIPrefix(False)
        self.gps_trail = self.gps_plot.plot(pen=pg.mkPen("#1f77b4", width=1))
        self.gps_cur = pg.ScatterPlotItem(size=12, brush=pg.mkBrush("#d62728"), pen=pg.mkPen("k"))
        self.gps_org = pg.ScatterPlotItem(size=14, symbol="star", brush=pg.mkBrush("#2ca02c"), pen=pg.mkPen("k"))
        self.gps_plot.addItem(self.gps_org); self.gps_plot.addItem(self.gps_cur)
        left.addWidget(self.gps_plot, 1)
        self.chk_fit = QtWidgets.QCheckBox("auto-fit GPS view"); self.chk_fit.setChecked(True); left.addWidget(self.chk_fit)
        self.alt_plot = pg.PlotWidget(title="Altitude above start")
        self.alt_plot.showGrid(x=True, y=True, alpha=0.3); self.alt_plot.addLegend(offset=(-5, 5))
        self.alt_plot.setLabel("left", "altitude [m]"); self.alt_plot.setLabel("bottom", "time relative to now (s)")
        for ax in ("bottom", "left"): self.alt_plot.getAxis(ax).enableAutoSIPrefix(False)
        self.c_balt = self.alt_plot.plot(pen=pg.mkPen("#1f77b4", width=2), name="baro (ISA, from start pressure)")
        self.c_galt = self.alt_plot.plot(pen=pg.mkPen("#ff7f0e", width=2), name="GPS height (MSL, from first fix)")
        h.addWidget(self.alt_plot, 1)
        # --- tab 3: log
        self.log = QtWidgets.QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(500)
        self.log.setStyleSheet("font-family: monospace;"); self.tabs.addTab(self.log, "Log")
        # --- status bar
        self.lbl_stat = QtWidgets.QLabel("waiting for data..."); self.lbl_stat.setTextFormat(QtCore.Qt.RichText)
        self.lbl_stat.setStyleSheet("font-family: monospace;")
        self.statusBar().addWidget(self.lbl_stat, 1)

    # ------------------------------------------------------------- actions
    def realign(self): self.ff = FlightFilter(self.cfg)

    def rezero(self):
        self.baro.clear(); self.gps.clear(); self.origin = None
        self.p0 = float(np.mean(self.last_p)) if self.last_p else None
        self.gps_trail.setData([], []); self.gps_org.setData([], []); self.gps_cur.setData([], [])

    def clear_plots(self):
        for r in (self.imu, self.mag, self.baro, self.gps): r.clear()
        self.origin = None; self.gps_trail.setData([], [])

    def reset_all(self, why):
        self.clear_plots(); self.ff = FlightFilter(self.cfg); self.p0 = None; self.last_p = []
        self.t_latest = 0.0; self.log.appendPlainText(f"[viewer] reset: {why}")

    # --------------------------------------------------------------- events
    def handle(self, ev):
        k = ev[0]
        if k == "I":
            _, t, f, w, rx, rg = ev
            self.t_latest = max(self.t_latest, t); self.n_imu += 1
            self.imu.push([t, f[0], f[1], f[2], w[0], w[1], w[2]])
            self.ff.feed_imu(t, f, w, rx, rg)
        elif k == "M":
            self.t_latest = max(self.t_latest, ev[1]); self.mag.push([ev[1], *ev[2]])
        elif k == "B":
            _, t, p = ev
            self.t_latest = max(self.t_latest, t)
            self.last_p.append(p); self.last_p = self.last_p[-10:]
            if self.p0 is None and len(self.last_p) >= 10: self.p0 = float(np.mean(self.last_p))
            if self.p0:
                alt = 44330.0 * (1.0 - (p / self.p0) ** 0.190294957)
                self.baro.push([t, alt, p])
            self.ff.feed_baro(t, p)
        elif k == "G":
            _, t, fix, sv, lat, lon, hmsl, vd, hacc, vacc, sacc = ev
            self.t_latest = max(self.t_latest, t); self.n_gps += 1
            self.gps_last = (t, fix, sv, lat, lon, hmsl, hacc)
            if fix >= 3 and math.isfinite(lat) and math.isfinite(lon) and (lat != 0.0 or lon != 0.0):
                if self.origin is None:
                    self.origin = (lat, lon, hmsl if math.isfinite(hmsl) else 0.0)
                    self.gps_org.setData([0.0], [0.0])
                la0, lo0, h0 = self.origin
                E = (lon - lo0) * M_PER_DEG_LAT * math.cos(math.radians(la0)); N = (lat - la0) * M_PER_DEG_LAT
                self.gps.push([t, E, N, hmsl - h0 if math.isfinite(hmsl) else np.nan, sv])
            if all(math.isfinite(x) for x in (hmsl, vd, vacc)):      # firmware does not send sAcc: use 0
                self.ff.feed_gps(t, fix, sv, hmsl, vd, vacc, sacc if math.isfinite(sacc) else 0.0)
        elif k == "S":
            self.set_status(ev[1])
            now = time.monotonic()
            if now - getattr(self, "_last_log_t", 0) > 5: self._last_log_t = now; self.log.appendPlainText(ev[2])
        elif k == "L":
            self.log.appendPlainText(ev[1])
        elif k == "RESET":
            self.reset_all(ev[1])
        elif k == "ERR":
            self.err_msg = ev[1]; self.log.appendPlainText(f"[viewer] {ev[1]}")

    def set_status(self, st):
        parts = []
        for name in ("IMU", "BARO", "SD", "GPS", "RADIO", "USB"):
            seg = st.get(name)
            if seg is None: continue
            ok = seg.startswith(f"{name}:OK")
            parts.append(f"<span style='color:{'#1a7f2e' if ok else '#c0392b'}'>{html.escape(seg)}</span>")
        self.lbl_stat.setText(" | ".join(parts))

    # --------------------------------------------------------------- refresh
    def tick(self):
        n = 0
        while n < 4000:
            try: ev = self.q.get_nowait()
            except queue.Empty: break
            self.handle(ev); n += 1
        now = time.monotonic()
        if now - self.rate_t >= 1.0:
            dt = now - self.rate_t
            self.rate_imu = self.n_imu / dt; self.rate_gps = self.n_gps / dt
            self.n_imu = self.n_gps = 0; self.rate_t = now
            mode = ("timestamped" if self.parser.timestamped else
                    "legacy status line (host time)" if self.parser.legacy_samples else "no samples yet")
            legacy = self.parser.legacy_samples and not self.parser.timestamped
            self.cfg.static_updates_every = 1 if legacy else 10      # slow stream: aid every sample
            self.lbl_rate.setText(f"IMU {self.rate_imu:5.0f} Hz  GPS {self.rate_gps:4.0f} Hz  "
                                  f"parse err {self.parser.errors}  [{mode}]")
        self.tick_n += 1
        if self.paused: return
        i = self.tabs.currentIndex()
        if i == 0: self.refresh_att()
        elif i == 1 and self.tick_n % 3 == 0: self.refresh_raw()
        elif i == 2 and self.tick_n % 4 == 0: self.refresh_gps()

    def refresh_att(self):
        ff, cv = self.ff, self.canvas
        if ff.phase == "ALIGN":
            started = ff._t_start is not None
            left = max(0.0, ff.cfg.align_s - (ff.t - ff._t_start)) if started else 0
            cv.banner.set_text(f"aligning - hold still ({left:.1f} s)" if started else "waiting for data...")
            cv.info.set_text("")
        else:
            cv.banner.set_text("")
            cv.C = frame_from_nose(ff.nose_b); cv.set_attitude(ff.kf.R)
            s = ff.snapshot(); nn = ff.kf.R @ ff.nose_b
            az = np.degrees(np.arctan2(nn[1], nn[0]))
            warn = "\nLEGACY LINE: host time, low rate => approximate" if (self.parser.legacy_samples and not self.parser.timestamped) else ""
            cv.info.set_text(
                f"t={s['t']:9.2f} s   {s['phase']}{'  (bench)' if ff.cfg.bench else ''}\n"
                f"tilt {s['tilt_deg']:6.1f} deg   spin {s['spin_dps']:7.0f} dps\n"
                f"nose heading* {az:6.1f} deg   (* arbitrary, no mag)\n"
                + ("" if ff.cfg.bench else f"alt {s['alt_m']:7.1f} m   v_up {-s['vd']:6.1f} m/s\n")
                + f"att sigma {s['sig_att_deg']:4.1f} deg{warn}")
        cv.draw_idle()

    def _xrel(self, d, tn): return d[:, 0] - tn

    def refresh_raw(self):
        tn, w = self.t_latest, self.window_s
        d = self.imu.since(tn - w)
        if len(d) > 1:
            x = d[:, 0] - tn
            for j in range(3):
                self.c_acc[j].setData(x, d[:, 1 + j]); self.c_gyr[j].setData(x, d[:, 4 + j])
        m = self.mag.since(tn - w)
        if len(m) > 1:
            for j in range(3): self.c_mag[j].setData(m[:, 0] - tn, m[:, 1 + j])
            self.p_mag.setTitle("Magnetometer (LIS2MDL)")
        else:
            self.p_mag.setTitle("Magnetometer (LIS2MDL) - no data (not fitted / not streamed)")
        self.p_acc.setXRange(-w, 0, padding=0)

    def refresh_gps(self):
        tn, w = self.t_latest, self.window_s
        g = self.gps.data()
        if len(g):
            self.gps_trail.setData(g[:, 1], g[:, 2]); self.gps_cur.setData([g[-1, 1]], [g[-1, 2]])
            if self.chk_fit.isChecked():
                cx, cy = (g[:, 1].min() + g[:, 1].max()) / 2, (g[:, 2].min() + g[:, 2].max()) / 2
                hs = max(5.0, 0.6 * max(np.ptp(g[:, 1]), np.ptp(g[:, 2])))
                self.gps_plot.setRange(xRange=(cx - hs, cx + hs), yRange=(cy - hs, cy + hs), padding=0)
        if self.gps_last:
            t, fix, sv, lat, lon, hmsl, hacc = self.gps_last
            pos = f"lat {lat:.7f}  lon {lon:.7f}" if (math.isfinite(lat) and (lat or lon)) else "no position in stream"
            hc = f"  hAcc {hacc:.1f} m" if math.isfinite(hacc) and hacc < 1e5 else ""
            self.gps_info.setText(f"fix={fix}  sv={sv}{hc}\n{pos}")
        b = self.baro.since(tn - w); gg = self.gps.since(tn - w)
        if len(b) > 1: self.c_balt.setData(b[:, 0] - tn, b[:, 1])
        if len(gg) > 1: self.c_galt.setData(gg[:, 0] - tn, gg[:, 3])
        self.alt_plot.setXRange(-w, 0, padding=0)

    def closeEvent(self, e):
        self.stop.set(); super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--port", help="serial device or 'auto'")
    g.add_argument("--replay", metavar="CSV_DIR", help="replay flight CSVs in real time as the text stream")
    g.add_argument("--text-file", metavar="FILE", help="replay a captured text stream (e.g. cat /dev/ttyACM0 > cap.txt)")
    ap.add_argument("--line-rate", type=float, default=0.0,
                    help="text-file: lines/s (0 = use t= stamps when present, else 10 lines/s)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--start", type=float, default=0.0); ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--fake-mag", action="store_true", help="replay only: SYNTHETIC mag lines to exercise the plot")
    ap.add_argument("--flight", action="store_true", help="enable launch/burnout logic (default: bench mode)")
    ap.add_argument("--snapshot-dir", help="headless: run --seconds, save one PNG per tab, exit")
    ap.add_argument("--seconds", type=float, default=10.0)
    a = ap.parse_args()

    q = queue.Queue(maxsize=300000); stop = threading.Event()
    parser = StreamParser(); cfg = Config(bench=not a.flight, max_dt=0.5)
    if a.port:
        port = a.port
        if port == "auto":
            c = sorted(glob.glob("/dev/serial/by-id/*STM*") + glob.glob("/dev/ttyACM*"))
            if not c: sys.exit("no serial port found; pass --port")
            port = c[0]
        th = threading.Thread(target=serial_thread, args=(port, a.baud, q, stop, parser), daemon=True); desc = port
    elif a.text_file:
        th = threading.Thread(target=file_thread, args=(a.text_file, a.line_rate, q, stop, parser), daemon=True)
        desc = f"file {os.path.basename(a.text_file)}"
    else:
        th = threading.Thread(target=replay_thread, args=(a.replay, a.start, a.speed, q, stop, a.fake_mag, parser),
                              daemon=True)
        desc = f"replay {os.path.basename(os.path.normpath(a.replay))} x{a.speed:g}" + (" (+synthetic mag)" if a.fake_mag else "")
    th.start()
    app = QtWidgets.QApplication(sys.argv)
    win = LiveWindow(q, desc, cfg, parser, stop); win.show()
    if a.snapshot_dir:
        os.makedirs(a.snapshot_dir, exist_ok=True)
        def finish():
            names = ["attitude", "raw", "gps_alt", "log"]
            for i, n in enumerate(names):
                win.tabs.setCurrentIndex(i); win.tick(); app.processEvents()
                (win.refresh_att, win.refresh_raw, win.refresh_gps, lambda: None)[i]()
                app.processEvents()
                if i == 0: win.canvas.draw()
                win.grab().save(os.path.join(a.snapshot_dir, f"tab_{n}.png"))
            stop.set(); app.quit()
        QtCore.QTimer.singleShot(int(a.seconds * 1000), finish)
    app.exec_(); stop.set()


if __name__ == "__main__":
    main()
