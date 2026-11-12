"""Atlas v4 error-state EKF (ESKF) + flight-phase logic.

Frames
  body : IMU sensor frame (x,y,z as printed on the LSM6DSV80X)
  nav  : local NED, origin at the launch pad, yaw is *arbitrary* (no magnetometer)
         unless you pass yaw_deg.  Horizontal GPS is therefore NOT used by default.

State (nominal)   q (body->nav, Hamilton, [w,x,y,z]), v, p (NED, m), ba, bg
Error state (15)  [dtheta(body), dv, dp, dba, dbg]

Units inside this file are SI (m/s^2, rad/s, m).  FlightFilter takes g and dps.
Pure numpy; no HAL, no globals, so the same code can later be ported to C.
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
import numpy as np

G0 = 9.80665
D2R = np.pi / 180.0
R2D = 180.0 / np.pi
# 99.9 % chi-square gates by degrees of freedom
CHI2 = {1: 10.83, 2: 13.82, 3: 16.27, 6: 22.46}


# ---------------------------------------------------------------- math helpers
def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def qexp(rv):
    a = np.sqrt(rv @ rv)
    if a < 1e-9:
        q = np.array([1.0, 0.5*rv[0], 0.5*rv[1], 0.5*rv[2]])
        return q / np.sqrt(q @ q)
    s = np.sin(0.5*a) / a
    return np.array([np.cos(0.5*a), rv[0]*s, rv[1]*s, rv[2]*s])


def q2R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = 2*np.sqrt(t+1.0)
        q = [0.25*s, (R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2*np.sqrt(1.0+R[0, 0]-R[1, 1]-R[2, 2])
        q = [(R[2, 1]-R[1, 2])/s, 0.25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s]
    elif R[1, 1] > R[2, 2]:
        s = 2*np.sqrt(1.0+R[1, 1]-R[0, 0]-R[2, 2])
        q = [(R[0, 2]-R[2, 0])/s, (R[0, 1]+R[1, 0])/s, 0.25*s, (R[1, 2]+R[2, 1])/s]
    else:
        s = 2*np.sqrt(1.0+R[2, 2]-R[0, 0]-R[1, 1])
        q = [(R[1, 0]-R[0, 1])/s, (R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, 0.25*s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def euler_zyx(R):
    """roll, pitch, yaw (deg) of body in nav, aerospace 3-2-1."""
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll*R2D, pitch*R2D, yaw*R2D


# ------------------------------------------------------------------- ESKF core
@dataclass
class Params:
    g: float = G0
    # Process noise.  Measured on flight-1 pad data (t=1690-1745 s, 455 Hz):
    #   gyro  3.4-4.9 mdps/rtHz  = 6-9e-5 rad/s/rtHz   -> 1.5e-4 (about 2x margin)
    #   accel 0.6-2.1e-3 m/s^2/rtHz                     -> 5e-3 (margin for flight vibration)
    # Gyro bias moved < 2 mdps over 55 s on the pad.  Tune again from flight residuals.
    gyro_noise: float = 1.5e-4      # rad/s/sqrt(Hz)
    accel_noise: float = 5e-3       # m/s^2/sqrt(Hz)
    gyro_bias_rw: float = 5e-6      # rad/s^2/sqrt(Hz)
    accel_bias_rw: float = 1e-4     # m/s^3/sqrt(Hz)
    # Initial 1-sigma
    p0_att_deg: float = 2.0
    p0_vel: float = 0.1
    p0_pos: float = 1.0
    p0_ba: float = 0.2              # m/s^2
    p0_bg_dps: float = 0.5


class ESKF:
    def __init__(self, prm: Params | None = None):
        self.prm = prm or Params()
        self.q = np.array([1.0, 0, 0, 0])
        self.v = np.zeros(3)
        self.p = np.zeros(3)
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.P = np.eye(15)
        self.gvec = np.array([0.0, 0.0, self.prm.g])
        self._I15 = np.eye(15)

    # ---- initialisation from a stationary pad window
    def init_from_pad(self, f_mean, w_mean, yaw_deg=0.0):
        """f_mean: mean specific force (m/s^2, body); w_mean: mean gyro (rad/s)."""
        zb = -f_mean / np.linalg.norm(f_mean)                 # nav-down in body
        ref = np.zeros(3)
        ref[np.argmin(np.abs(zb))] = 1.0                      # body axis most horizontal
        h = ref - (ref @ zb) * zb
        h /= np.linalg.norm(h)
        k = np.cross(zb, h)
        psi = yaw_deg * D2R
        xn = np.cos(psi)*h - np.sin(psi)*k
        yn = np.sin(psi)*h + np.cos(psi)*k
        M = np.vstack([xn, yn, zb])                           # rows = nav axes in body = R_nb
        self.q = R2q(M)
        self.v[:] = 0
        self.p[:] = 0
        self.ba[:] = 0
        self.bg = np.array(w_mean, float)
        pr = self.prm
        self.P = np.zeros((15, 15))
        self.P[0:3, 0:3] = np.eye(3) * (pr.p0_att_deg*D2R)**2
        self.P[3:6, 3:6] = np.eye(3) * pr.p0_vel**2
        self.P[6:9, 6:9] = np.eye(3) * pr.p0_pos**2
        self.P[9:12, 9:12] = np.eye(3) * pr.p0_ba**2
        self.P[12:15, 12:15] = np.eye(3) * (pr.p0_bg_dps*D2R)**2

    # ---- time update
    def predict(self, f_m, w_m, dt, accel_noise_scale=1.0):
        pr = self.prm
        w = w_m - self.bg
        f = f_m - self.ba
        R = q2R(self.q)
        a_n = R @ f + self.gvec
        self.p += self.v*dt + 0.5*a_n*dt*dt
        self.v += a_n*dt
        q = qmul(self.q, qexp(w*dt))
        self.q = q / np.sqrt(q @ q)

        F = self._I15.copy()
        F[0:3, 0:3] -= skew(w)*dt
        F[0:3, 12:15] = -np.eye(3)*dt
        F[3:6, 0:3] = -R @ skew(f)*dt
        F[3:6, 9:12] = -R*dt
        F[6:9, 3:6] = np.eye(3)*dt
        Q = np.zeros(15)
        Q[0:3] = (pr.gyro_noise**2)*dt
        Q[3:6] = (pr.accel_noise*accel_noise_scale)**2*dt
        Q[9:12] = (pr.accel_bias_rw**2)*dt
        Q[12:15] = (pr.gyro_bias_rw**2)*dt
        self.P = F @ self.P @ F.T
        self.P[np.diag_indices(15)] += Q

    # ---- generic measurement update (Joseph form, NIS gate, error reset)
    def _update(self, y, H, Rm, gate=True, gate_scale=1.0):
        S = H @ self.P @ H.T + Rm
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False, np.nan
        nis = float(y @ Sinv @ y)
        if gate and nis > CHI2.get(len(y), 22.46) * gate_scale:
            return False, nis
        K = self.P @ H.T @ Sinv
        dx = K @ y
        IKH = self._I15 - K @ H
        P = IKH @ self.P @ IKH.T + K @ Rm @ K.T
        # inject + reset
        self.q = qmul(self.q, qexp(dx[0:3]))
        self.q /= np.sqrt(self.q @ self.q)
        self.v += dx[3:6]
        self.p += dx[6:9]
        self.ba += dx[9:12]
        self.bg += dx[12:15]
        G = self._I15.copy()
        G[0:3, 0:3] -= 0.5*skew(dx[0:3])
        P = G @ P @ G.T
        self.P = 0.5*(P + P.T)
        return True, nis

    # ---- measurements
    def update_baro(self, alt_up, sigma, gate_scale=1.0):
        """alt_up: metres above pad.  h(x) = -p_d."""
        H = np.zeros((1, 15)); H[0, 8] = -1.0
        y = np.array([alt_up + self.p[2]])
        return self._update(y, H, np.array([[sigma**2]]), gate_scale=gate_scale)

    def update_gps_vertical(self, pd_meas, vd_meas, sig_p, sig_v, gate_scale=1.0):
        H = np.zeros((2, 15)); H[0, 8] = 1.0; H[1, 5] = 1.0
        y = np.array([pd_meas - self.p[2], vd_meas - self.v[2]])
        return self._update(y, H, np.diag([sig_p**2, sig_v**2]), gate_scale=gate_scale)

    def update_zupt(self, sig_v=0.02, sig_p=0.5):
        H = np.zeros((6, 15)); H[0:3, 3:6] = np.eye(3); H[3:6, 6:9] = np.eye(3)
        y = np.concatenate([-self.v, -self.p])
        Rm = np.diag([sig_v**2]*3 + [sig_p**2]*3)
        return self._update(y, H, Rm, gate=False)

    def update_tilt(self, f_m, sigma=0.03):
        """Accelerometer gravity-direction pseudo-measurement (roll/pitch only)."""
        f = f_m - self.ba
        n = np.linalg.norm(f)
        if n < 1e-3:
            return False, np.nan
        u_meas = f / n
        u_est = -(q2R(self.q).T @ np.array([0.0, 0.0, 1.0]))
        H = np.zeros((3, 15)); H[:, 0:3] = skew(u_est)
        return self._update(u_meas - u_est, H, np.eye(3)*sigma**2, gate=False)

    def update_gyro_static(self, w_m, sigma=2e-3):
        """At rest the gyro output is its bias."""
        H = np.zeros((3, 15)); H[:, 12:15] = np.eye(3)
        return self._update(w_m - self.bg, H, np.eye(3)*sigma**2, gate=False)

    # ---- convenience
    @property
    def R(self):
        return q2R(self.q)

    def sigma(self):
        return np.sqrt(np.maximum(np.diag(self.P), 0.0))


# --------------------------------------------------------------- flight logic
@dataclass
class Config:
    align_s: float = 5.0            # stationary window used for initial alignment
    yaw_deg: float = 0.0            # heading of the reference body axis (arbitrary)
    launch_g: float = 2.5           # sustained specific force along nose
    launch_n: int = 60              # samples averaged for detection (~0.13 s)
    burnout_g: float = 0.3
    baro_sigma: float = 2.0         # m
    baro_lag_s: float = 1.5         # pressure-port lag: sigma_eff^2 = sigma^2 + (lag * |v|)^2
                                    # (flight-1: baro read ~100+ m low at 130 m/s just after burnout)
    baro_lock_after_burnout: float = 1.0   # s (pressure spike during burn)
    gps_mode: str = "off"           # "off" | "vertical"  (GPS is unreliable in flight, see README)
    gps_pos_sigma: float = 8.0      # m   (GPS quality in flight is poor, see README)
    gps_vel_sigma: float = 1.5      # m/s
    gps_min_sv: int = 6
    gps_lock_after_burnout: float = 2.0
    shock_s: float = 0.5            # distrust accel for this long after a rail hit
    shock_accel_scale: float = 100.0
    boost_accel_scale: float = 20.0   # Q inflation while thrusting: scale-factor / misalignment
                                      # error grows with acceleration and is not in the noise model
    regate_rate: float = 0.25         # gate grows as 1 + regate_rate * n_consecutive_rejects**2
    descent_tilt_sigma: float = 0.15  # chute phase: gravity direction, noisy from swinging
    static_updates_every: int = 10  # PAD: aiding at ~ODR/10
    bench: bool = False             # never leave PAD (live viewer on a desk)
    tilt_sigma: float = 0.03
    max_dt: float = 0.05            # longest IMU gap integrated; raise for slow text streams


class FlightFilter:
    PHASES = ("ALIGN", "PAD", "BOOST", "COAST", "DESCENT", "LANDED")

    def __init__(self, cfg: Config | None = None, prm: Params | None = None):
        self.cfg = cfg or Config()
        self.kf = ESKF(prm)
        self.phase = "ALIGN"
        self.events: list[tuple[float, str, str]] = []
        self.updates: list[tuple[float, str, float, int]] = []
        self.t = 0.0
        self.nose_b = np.array([0.0, 0.0, 1.0])
        self.p0 = None
        self.h0 = None
        self._al_f, self._al_w, self._al_p, self._al_h = [], [], [], []
        self._t_start = None
        self._t_prev = None
        self._last_f = None
        self._last_w = None
        self._shock_until = -1.0
        self._baro_lock_until = -1.0
        self._gps_lock_until = -1.0
        self._up_win = deque(maxlen=self.cfg.launch_n)
        self._land_win = deque(maxlen=455)
        self._k = 0
        self._nrej = {"baro": 0, "gps": 0}
        self.t_launch = self.t_burnout = self.t_apogee = None
        self.alt_max = 0.0
        self.f_last = np.zeros(3)
        self.w_last = np.zeros(3)

    # --- logging
    def _ev(self, t, name, info=""):
        self.events.append((t, name, info))

    # --- IMU: g and dps in, body frame
    def feed_imu(self, t, f_g, w_dps, rail_xl=False, rail_gy=False):
        c = self.cfg
        f = np.asarray(f_g, float) * G0
        w = np.asarray(w_dps, float) * D2R
        if rail_xl and self._last_f is not None:
            f = self._last_f
            self._shock_until = t + c.shock_s
            self._ev_once(t, "shock", "accel at full-scale rail")
        elif not rail_xl:
            self._last_f = f
        if rail_gy and self._last_w is not None:
            w = self._last_w
            self._shock_until = t + c.shock_s
            self._ev_once(t, "shock", "gyro at int16 rail")
        elif not rail_gy:
            self._last_w = w
        self.t = t
        self.f_last, self.w_last = f, w

        if self.phase == "ALIGN":
            if self._t_start is None:
                self._t_start = t
            self._al_f.append(f); self._al_w.append(w)
            if t - self._t_start >= c.align_s:
                self._do_align(t)
            return

        dt = t - self._t_prev
        self._t_prev = t
        if not (1e-5 < dt < c.max_dt):
            dt = min(max(dt, 1e-4), c.max_dt)
        scale = c.shock_accel_scale if t < self._shock_until else 1.0
        if self.phase == "BOOST" or (self.phase == "COAST" and t < self._baro_lock_until):
            scale = max(scale, c.boost_accel_scale)
        self.kf.predict(f, w, dt, scale)
        self._k += 1
        fa = float(np.linalg.norm(f))

        # specific force along the nose axis, in g
        up_g = float(f @ self.nose_b) / G0
        self._up_win.append(up_g)
        mean_up = sum(self._up_win) / len(self._up_win)

        if self.phase == "PAD":
            if self._k % c.static_updates_every == 0:
                self._pad_aiding(t, f, w, fa)
            if (not c.bench and len(self._up_win) == c.launch_n
                    and mean_up > c.launch_g):
                self.phase = "BOOST"
                self.t_launch = t
                self._baro_lock_until = 1e9
                self._gps_lock_until = 1e9
                self._ev(t, "launch", f"{mean_up:.1f} g along nose")
        elif self.phase == "BOOST":
            if t - self.t_launch > 0.5 and mean_up < c.burnout_g:
                self.phase = "COAST"
                self.t_burnout = t
                self._baro_lock_until = t + c.baro_lock_after_burnout
                self._gps_lock_until = t + c.gps_lock_after_burnout
                self._ev(t, "burnout", f"{t - self.t_launch:.2f} s burn")
        elif self.phase == "COAST":
            alt = -self.kf.p[2]
            self.alt_max = max(self.alt_max, alt)
            if t - self.t_burnout > 1.0 and self.kf.v[2] >= 0.0:
                self.phase = "DESCENT"
                self.t_apogee = t
                self._ev(t, "apogee", f"{self.alt_max:.0f} m max altitude")
        elif self.phase == "DESCENT":
            if self._k % c.static_updates_every == 0 and abs(fa/G0 - 1.0) < 0.2:
                self.kf.update_tilt(f, c.descent_tilt_sigma)
            self._land_win.append((fa / G0, float(np.linalg.norm(w))))
            if (t - self.t_apogee > 10.0 and len(self._land_win) == 455
                    and self._k % 50 == 0):
                a = np.array(self._land_win)
                if (a[:, 0].std() < 0.005 and abs(a[:, 0].mean() - 1.0) < 0.05
                        and a[:, 1].mean() < 2*D2R):
                    self.phase = "LANDED"
                    self._ev(t, "landed", f"alt at touchdown {-self.kf.p[2]:+.1f} m (closure error)")
        elif self.phase == "LANDED":
            if self._k % c.static_updates_every == 0:
                self._pad_aiding(t, f, w, fa)

    def _ev_once(self, t, name, info):
        if not self.events or self.events[-1][1] != name or t - self.events[-1][0] > 2.0:
            self._ev(t, name, info)

    def _do_align(self, t):
        f0 = np.mean(self._al_f, axis=0)
        w0 = np.mean(self._al_w, axis=0)
        self.kf.init_from_pad(f0, w0, self.cfg.yaw_deg)
        self.nose_b = f0 / np.linalg.norm(f0)
        if self._al_p:
            self.p0 = float(np.mean(self._al_p))
        if self._al_h:
            self.h0 = float(np.mean(self._al_h))
        self.phase = "PAD"
        self._t_prev = t
        self._ev(t, "aligned",
                 f"|f|={np.linalg.norm(f0)/G0:.4f} g, nose_b="
                 f"[{self.nose_b[0]:+.2f},{self.nose_b[1]:+.2f},{self.nose_b[2]:+.2f}]")

    def _pad_aiding(self, t, f, w, fa):
        k = self.kf
        c = self.cfg
        near_1g = abs(fa/G0 - 1.0) < 0.1
        static = near_1g and abs(fa/G0 - 1.0) < 0.05 and np.linalg.norm(w - k.bg) < 3*D2R
        if near_1g:
            ok, nis = k.update_tilt(f, c.tilt_sigma)
        if static or c.bench:             # bench: the board is on a desk, never translating
            k.update_zupt()
        if static:
            k.update_gyro_static(w)

    # --- barometer: pressure in Pa
    def feed_baro(self, t, p_pa):
        if self.phase == "ALIGN":
            if self._t_start is not None:
                self._al_p.append(p_pa)
            return
        if self.p0 is None:
            self.p0 = p_pa
        alt = 44330.0 * (1.0 - (p_pa / self.p0) ** 0.190294957)
        self.last_baro_alt = alt
        if t < self._baro_lock_until or t < self._shock_until + 0.5:
            return
        gs = 1.0 + self.cfg.regate_rate * self._nrej["baro"] ** 2
        c = self.cfg
        sig = float(np.sqrt(c.baro_sigma**2 + (c.baro_lag_s * abs(self.kf.v[2]))**2))
        ok, nis = self.kf.update_baro(alt, sig, gs)
        self._nrej["baro"] = 0 if ok else self._nrej["baro"] + 1
        self.updates.append((t, "baro", nis, int(ok)))

    # --- GPS: only vertical channel, see README
    def feed_gps(self, t, fix, nsv, hmsl, vel_d, vacc, sacc):
        good = fix == 3 and nsv >= self.cfg.gps_min_sv
        if self.phase == "ALIGN":
            if self._t_start is not None and good:
                self._al_h.append(hmsl)
            return
        if self.h0 is None and good:
            self.h0 = hmsl
        if (self.cfg.gps_mode != "vertical" or not good or self.h0 is None
                or t < self._gps_lock_until or t < self._shock_until + 0.5):
            return
        sp = max(self.cfg.gps_pos_sigma, vacc)
        sv = max(self.cfg.gps_vel_sigma, sacc)
        gs = 1.0 + self.cfg.regate_rate * self._nrej["gps"] ** 2
        ok, nis = self.kf.update_gps_vertical(-(hmsl - self.h0), vel_d, sp, sv, gs)
        self._nrej["gps"] = 0 if ok else self._nrej["gps"] + 1
        self.updates.append((t, "gps", nis, int(ok)))

    # --- output
    def snapshot(self):
        k = self.kf
        R = k.R
        roll, pitch, yaw = euler_zyx(R)
        nose_n = R @ self.nose_b
        tilt = np.degrees(np.arccos(np.clip(-nose_n[2], -1, 1)))
        spin = float((self.w_last - k.bg) @ self.nose_b) * R2D
        s = k.sigma()
        return dict(t=self.t, phase=self.phase,
                    qw=k.q[0], qx=k.q[1], qy=k.q[2], qz=k.q[3],
                    roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw,
                    tilt_deg=tilt, spin_dps=spin,
                    vn=k.v[0], ve=k.v[1], vd=k.v[2],
                    pn=k.p[0], pe=k.p[1], pd=k.p[2], alt_m=-k.p[2],
                    speed=float(np.linalg.norm(k.v)),
                    bax=k.ba[0], bay=k.ba[1], baz=k.ba[2],
                    bgx_dps=k.bg[0]*R2D, bgy_dps=k.bg[1]*R2D, bgz_dps=k.bg[2]*R2D,
                    sig_att_deg=float(np.linalg.norm(s[0:3])*R2D),
                    sig_alt_m=float(s[8]), sig_vz=float(s[5]),
                    f_nose_g=float(self.f_last @ self.nose_b) / G0)
