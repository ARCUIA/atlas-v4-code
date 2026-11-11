"""Sanity tests for eskf.py.  Run:  python3 test_eskf.py"""
import numpy as np
from eskf import ESKF, FlightFilter, Config, G0, q2R, qexp, qmul, D2R

rng = np.random.default_rng(1)

def test_rotation():
    k = ESKF(); k.bg[:] = 0
    dt = 1/455
    for _ in range(455):
        k.predict(np.array([0, 0, G0]) * 0 + np.array([0.0, 0.0, -G0]), np.array([0, 0, 90*D2R]), dt)
    R = k.R   # 90 deg about z (body z == nav z here)
    assert abs(R[0, 1] + 1) < 2e-3 and abs(R[1, 0] - 1) < 2e-3, R
    print("rotation integration ok")

def test_align():
    k = ESKF()
    f = np.array([-1.003*G0, -0.025*G0, 0.079*G0])    # your pad reading: nose = -X
    k.init_from_pad(f, np.zeros(3))
    up_nav = k.R @ (f/np.linalg.norm(f))
    assert np.allclose(up_nav, [0, 0, -1], atol=1e-9), up_nav
    print("alignment ok (specific force maps to nav-up)")

def synth(a_burn_g=8.0, burn=2.0, pad=20.0, T=40.0, noise=True):
    """vertical rocket, nose = -X.  returns arrays of t, f_g(3), w_dps, true alt, true v_up"""
    dt = 1/455; t = np.arange(0, pad+T, dt)
    up = np.zeros_like(t); a_net = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < pad: a_net[i] = 0.0
        elif ti < pad+burn: a_net[i] = a_burn_g*G0
        else: a_net[i] = -G0
    v = np.cumsum(a_net)*dt; v[t < pad] = 0
    # clamp ground
    alt = np.cumsum(v)*dt
    ground = (t >= pad+burn)&(alt < 0); 
    spec_up = (a_net + G0)/G0              # specific force along up, g
    spec_up[(t < pad)] = 1.0
    f = np.zeros((len(t), 3)); f[:, 0] = -spec_up   # nose = -X
    w = np.zeros((len(t), 3))
    if noise:
        f += rng.normal(0, 0.003, f.shape); w += rng.normal(0, 0.08, w.shape)
        w += np.array([0.3, -0.2, 0.1])            # constant gyro bias (dps)
    return t, f, w, alt, v

def test_flight():
    t, f, w, alt, v = synth()
    ff = FlightFilter(Config())
    p0 = 99500.0
    ib = 0
    tb = np.arange(0, t[-1], 1/25)
    for i, ti in enumerate(t):
        while ib < len(tb) and tb[ib] <= ti:
            j = min(int(tb[ib]*455), len(alt)-1)
            p = p0*(1 - alt[j]/44330.0)**(1/0.190294957)
            ff.feed_baro(tb[ib], p + rng.normal(0, 8)); ib += 1
        ff.feed_imu(ti, f[i], w[i])
    ev = {e[1]: e[0] for e in ff.events}
    print("events:", {k: round(v_, 2) for k, v_ in ev.items()})
    assert "launch" in ev and abs(ev["launch"] - 20.0) < 0.2
    assert "burnout" in ev and abs(ev["burnout"] - 22.0) < 0.2
    i_ap = np.argmax(alt); t_ap = t[i_ap]
    assert "apogee" in ev and abs(ev["apogee"] - t_ap) < 1.5, (ev, t_ap)
    s = ff.snapshot()
    print(f"true apogee {alt.max():.1f} m @ {t_ap:.2f} s ; filter alt_max {ff.alt_max:.1f} m")
    assert abs(ff.alt_max - alt.max()) < 5.0
    print(f"tilt at end {s['tilt_deg']:.2f} deg (true 0), bg_x {s['bgx_dps']:.2f} dps (true 0.3)")
    assert s["tilt_deg"] < 2.0
    print("synthetic flight ok")

def test_static_hold():
    t, f, w, alt, v = synth(T=0.0, pad=60.0)
    ff = FlightFilter(Config())
    for i, ti in enumerate(t):
        ff.feed_imu(ti, f[i], w[i])
    s = ff.snapshot()
    print(f"60 s static: tilt {s['tilt_deg']:.3f} deg, |v| {s['speed']:.4f} m/s, bg {s['bgx_dps']:.3f},{s['bgy_dps']:.3f},{s['bgz_dps']:.3f} dps (true 0.3,-0.2,0.1)")
    assert s["tilt_deg"] < 0.3 and s["speed"] < 0.05
    print("static hold ok")

if __name__ == "__main__":
    test_rotation(); test_align(); test_static_hold(); test_flight()
    print("ALL OK")
