"""Load Atlas flight CSVs (atlas_parse.py output) into numpy arrays."""
from __future__ import annotations
import os
import numpy as np
import pandas as pd

XL_RAIL_RAW = 32760     # raw int16 counts treated as "at the rail"
GY_RAIL_RAW = 32760


def load_flight(d: str):
    imu = pd.read_csv(os.path.join(d, "imu.csv"), usecols=[
        "t_s", "xl_x_mg", "xl_y_mg", "xl_z_mg", "gy_x_mdps", "gy_y_mdps", "gy_z_mdps",
        "xl_raw_x", "xl_raw_y", "xl_raw_z", "gy_raw_x", "gy_raw_y", "gy_raw_z"])
    out = {}
    out["imu_t"] = imu["t_s"].to_numpy()
    out["imu_f"] = imu[["xl_x_mg", "xl_y_mg", "xl_z_mg"]].to_numpy() / 1000.0   # g
    out["imu_w"] = imu[["gy_x_mdps", "gy_y_mdps", "gy_z_mdps"]].to_numpy() / 1000.0  # dps
    out["imu_rail_xl"] = (imu[["xl_raw_x", "xl_raw_y", "xl_raw_z"]].abs().to_numpy()
                          >= XL_RAIL_RAW).any(axis=1)
    out["imu_rail_gy"] = (imu[["gy_raw_x", "gy_raw_y", "gy_raw_z"]].abs().to_numpy()
                          >= GY_RAIL_RAW).any(axis=1)
    b = pd.read_csv(os.path.join(d, "baro.csv"), usecols=["t_s", "pressure_pa"])
    out["baro_t"] = b["t_s"].to_numpy()
    out["baro_p"] = b["pressure_pa"].to_numpy()
    g = pd.read_csv(os.path.join(d, "gps.csv"), usecols=[
        "t_s", "fix_type", "num_sv", "hmsl_m", "vel_d_mps", "vacc_m", "sacc_mps",
        "lat_deg", "lon_deg", "hacc_m"])
    out["gps_t"] = g["t_s"].to_numpy()
    out["gps_fix"] = g["fix_type"].to_numpy()
    out["gps_nsv"] = g["num_sv"].to_numpy()
    out["gps_h"] = g["hmsl_m"].to_numpy()
    out["gps_vd"] = g["vel_d_mps"].to_numpy()
    out["gps_vacc"] = g["vacc_m"].to_numpy()
    out["gps_sacc"] = g["sacc_mps"].to_numpy()
    out["gps_lat"] = g["lat_deg"].to_numpy()
    out["gps_lon"] = g["lon_deg"].to_numpy()
    out["gps_hacc"] = g["hacc_m"].to_numpy()
    return out


def find_launch(imu_t, imu_f, thresh_g=3.0, n=100):
    """First time the n-sample (0.22 s) mean |f| exceeds thresh_g (coarse, offline only).
    Short handling spikes (121 s, 1290 s in flight-1) are shorter than the window."""
    a = np.linalg.norm(imu_f, axis=1)
    k = np.convolve(a, np.ones(n) / n, mode="same")
    idx = np.where(k > thresh_g)[0]
    return float(imu_t[idx[0]]) if len(idx) else None
