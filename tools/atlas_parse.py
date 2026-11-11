#!/usr/bin/env python3
"""Decode an Atlas log dump (raw 512-byte sectors) into CSV files and a summary.

    python3 tools/atlas_parse.py flight.bin --outdir flight_csv

Block format: see app/core/logblk.h. Every block is checked (magic + CRC-16); damaged blocks are
counted and skipped. Unit conversions use the same factors as ST's driver for the full scales
recorded in each session header.
"""
import argparse, csv, os, struct, sys

BLK, HDR = 512, 32
REC_SESSION, REC_IMU, REC_BARO, REC_STATUS, REC_GPS, REC_GPSCFG = 1, 2, 3, 4, 5, 6

# mg per LSB / mdps per LSB, copied from ST's lsm6dsv80x_reg.c conversion functions
XL_MG = {2: 0.061, 4: 0.122, 8: 0.244, 16: 0.488}
HG_MG = {32: 0.976, 64: 1.952, 80: 3.904}
GY_MDPS = {125: 4.375, 250: 8.75, 500: 17.5, 1000: 35.0, 2000: 70.0, 4000: 140.0}


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def check_block(blk):
    """Returns header dict, or None if the block is not an intact log block."""
    if len(blk) != BLK or blk[0:4] != b"ATLB":
        return None
    gen, session, seq, t0, nrec, used, crc = struct.unpack_from("<IIIIHHH", blk, 4)
    if used > BLK - HDR:
        return None
    if crc16(blk[HDR:HDR + used], crc16(blk[0:24])) != crc:
        return None
    return dict(gen=gen, session=session, seq=seq, t0=t0, nrec=nrec, used=used)


def records(blk, used):
    pos, end = HDR, HDR + used
    while pos + 2 <= end:
        rtype, rlen = blk[pos], blk[pos + 1]
        if pos + 2 + rlen > end:
            return
        yield rtype, blk[pos + 2:pos + 2 + rlen]
        pos += 2 + rlen


class Session:
    def __init__(self, key):
        self.key = key
        self.cfg = dict(cpu_mhz=0, xl_fs_g=16, hg_fs_g=80, gy_fs_dps=2000,
                        xl_odr=480, hg_odr=480, gy_odr=480, baro_odr_x100=2500)
        self.blocks = 0
        self.last_seq = None
        self.seq_gaps = 0
        self.t_last = None
        self.epoch = 0
        self.imu_n = [0, 0, 0]
        self.baro_n = 0
        self.t_first = None
        self.t_end = None
        self.dropped = 0
        self.gps_n = 0
        self.gps_fix_n = 0
        self.gps_cfg = None

    def unwrap(self, t32):
        """Timestamps are the low 32 bits of microseconds (wrap every ~71.6 min): make them monotonic."""
        if self.t_last is not None and t32 < self.t_last and self.t_last - t32 > 2 ** 31:
            self.epoch += 2 ** 32
        self.t_last = t32
        t = (t32 + self.epoch) / 1e6
        if self.t_first is None:
            self.t_first = t
        self.t_end = t
        return t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("--outdir", default=None, help="where to write CSVs (default: <dump>_csv)")
    args = ap.parse_args()

    data = open(args.dump, "rb").read()
    outdir = args.outdir or os.path.splitext(args.dump)[0] + "_csv"
    os.makedirs(outdir, exist_ok=True)

    imu_f = open(os.path.join(outdir, "imu.csv"), "w", newline="")
    baro_f = open(os.path.join(outdir, "baro.csv"), "w", newline="")
    status_f = open(os.path.join(outdir, "status.csv"), "w", newline="")
    gps_f = open(os.path.join(outdir, "gps.csv"), "w", newline="")
    imu_w, baro_w, status_w, gps_w = csv.writer(imu_f), csv.writer(baro_f), csv.writer(status_f), csv.writer(gps_f)
    imu_w.writerow(["session", "t_s", "flags", "xl_x_mg", "xl_y_mg", "xl_z_mg", "hg_x_mg", "hg_y_mg", "hg_z_mg",
                    "gy_x_mdps", "gy_y_mdps", "gy_z_mdps",
                    "xl_raw_x", "xl_raw_y", "xl_raw_z", "hg_raw_x", "hg_raw_y", "hg_raw_z",
                    "gy_raw_x", "gy_raw_y", "gy_raw_z"])
    baro_w.writerow(["session", "t_s", "pressure_hpa", "pressure_pa", "temp_c", "pressure_raw", "temp_raw"])
    status_w.writerow(["session", "t_s", "blocks_written", "blocks_dropped", "queue", "sd_errors"])
    gps_w.writerow(["session", "t_s", "itow_ms", "utc", "fix_type", "gnss_fix_ok", "num_sv", "lat_deg", "lon_deg",
                    "height_m", "hmsl_m", "hacc_m", "vacc_m", "vel_n_mps", "vel_e_mps", "vel_d_mps", "gspeed_mps",
                    "heading_deg", "sacc_mps", "pdop"])

    sessions, bad_blocks, foreign_blocks = {}, 0, 0
    for off in range(0, len(data) - len(data) % BLK, BLK):
        blk = data[off:off + BLK]
        h = check_block(blk)
        if h is None:
            if blk[0:4] == b"ATLB":
                bad_blocks += 1          # looks like a log block but failed the CRC
            else:
                foreign_blocks += 1      # card header or unused sector
            continue
        s = sessions.setdefault((h["gen"], h["session"]), Session((h["gen"], h["session"])))
        s.blocks += 1
        if s.last_seq is not None and h["seq"] > s.last_seq + 1:
            s.seq_gaps += h["seq"] - s.last_seq - 1
        s.last_seq = h["seq"]
        label = f'{h["gen"]}.{h["session"]}'
        for rtype, d in records(blk, h["used"]):
            if rtype == REC_SESSION and len(d) >= 22:
                (_t, ver, _r, mhz, xl, hg, gy, xo, ho, go, bo) = struct.unpack_from("<IBBHHHHHHHH", d)
                s.cfg.update(cpu_mhz=mhz, xl_fs_g=xl, hg_fs_g=hg, gy_fs_dps=gy,
                             xl_odr=xo, hg_odr=ho, gy_odr=go, baro_odr_x100=bo)
            elif rtype == REC_IMU and len(d) >= 23:
                t32, flags = struct.unpack_from("<IB", d)
                raw = struct.unpack_from("<9h", d, 5)
                t = s.unwrap(t32)
                kx, kh, kg = (XL_MG.get(s.cfg["xl_fs_g"], 0), HG_MG.get(s.cfg["hg_fs_g"], 0),
                              GY_MDPS.get(s.cfg["gy_fs_dps"], 0))
                conv = [v * kx for v in raw[0:3]] + [v * kh for v in raw[3:6]] + [v * kg for v in raw[6:9]]
                imu_w.writerow([label, f"{t:.6f}", flags] + [f"{v:.3f}" for v in conv] + list(raw))
                for i in range(3):
                    s.imu_n[i] += (flags >> i) & 1
            elif rtype == REC_BARO and len(d) >= 10:
                t32, praw, traw = struct.unpack_from("<Iih", d)
                t = s.unwrap(t32)
                hpa = praw / 1048576.0          # ST: lps22df_from_lsb_to_hPa
                baro_w.writerow([label, f"{t:.6f}", f"{hpa:.4f}", f"{hpa * 100:.2f}", f"{traw / 100.0:.2f}", praw, traw])
                s.baro_n += 1
            elif rtype == REC_GPS and len(d) >= 96:
                t32 = struct.unpack_from("<I", d)[0]
                pv = d[4:96]                       # the UBX-NAV-PVT payload, exactly as the receiver sent it
                t = s.unwrap(t32)
                itow, year = struct.unpack_from("<IH", pv, 0)
                mon, day, hr, mn, sec, valid = struct.unpack_from("<6B", pv, 6)
                fix, flags = pv[20], pv[21]
                nsv = pv[23]
                lon, lat, hgt, hmsl = struct.unpack_from("<4i", pv, 24)
                hacc, vacc = struct.unpack_from("<2I", pv, 40)
                vn, ve, vd, gs, hm = struct.unpack_from("<5i", pv, 48)
                sacc = struct.unpack_from("<I", pv, 68)[0]
                pdop = struct.unpack_from("<H", pv, 76)[0]
                gps_w.writerow([label, f"{t:.6f}", itow, f"{year:04d}-{mon:02d}-{day:02d}T{hr:02d}:{mn:02d}:{sec:02d}",
                                fix, flags & 1, nsv, f"{lat / 1e7:.7f}", f"{lon / 1e7:.7f}", f"{hgt / 1000:.3f}",
                                f"{hmsl / 1000:.3f}", f"{hacc / 1000:.3f}", f"{vacc / 1000:.3f}",
                                f"{vn / 1000:.3f}", f"{ve / 1000:.3f}", f"{vd / 1000:.3f}", f"{gs / 1000:.3f}",
                                f"{hm / 1e5:.5f}", f"{sacc / 1000:.3f}", f"{pdop / 100:.2f}"])
                s.gps_n += 1
                s.gps_fix_n += 1 if (fix >= 2 and flags & 1) else 0
            elif rtype == REC_GPSCFG and len(d) >= 17:
                _t, bf, bn, ms, code, dyn, only = struct.unpack_from("<IIIHbBB", d)
                s.gps_cfg = dict(baud_found=bf, baud_final=bn, meas_ms=ms, code=code, dynmodel=dyn, gps_only=only)
            elif rtype == REC_STATUS and len(d) >= 16:
                t32, wr, dr, q, err = struct.unpack_from("<IIIHH", d)
                t = s.unwrap(t32)
                status_w.writerow([label, f"{t:.6f}", wr, dr, q, err])
                s.dropped = max(s.dropped, dr)
    for f in (imu_f, baro_f, status_f, gps_f):
        f.close()

    print(f"{len(data) // BLK} sectors read: {sum(s.blocks for s in sessions.values())} good log blocks, "
          f"{bad_blocks} damaged, {foreign_blocks} not log blocks")
    for key in sorted(sessions):
        s = sessions[key]
        dur = (s.t_end - s.t_first) if s.t_first is not None and s.t_end > s.t_first else 0
        rate = lambda n: f"{n / dur:.0f} Hz" if dur else "n/a"
        print(f"session {key[0]}.{key[1]}: {s.blocks} blocks, {dur:.1f} s | "
              f"IMU xl/hg/gy = {s.imu_n[0]}/{s.imu_n[1]}/{s.imu_n[2]} samples ({rate(s.imu_n[0])}/{rate(s.imu_n[1])}/{rate(s.imu_n[2])}) | "
              f"baro {s.baro_n} ({rate(s.baro_n)}) | missing blocks {s.seq_gaps}, dropped (reported) {s.dropped}")
        if s.gps_cfg is not None or s.gps_n:
            c = s.gps_cfg
            cfg_txt = (f"config code {c['code']} (0 = OK), receiver found at {c['baud_found']} baud, now {c['baud_final']}, "
                       f"{c['meas_ms']} ms period, {'GPS only' if c['gps_only'] else 'all constellations'}") if c else "no config record"
            print(f"    GPS: {s.gps_n} solutions ({rate(s.gps_n)}), {s.gps_fix_n} with a valid fix | {cfg_txt}")
    print(f"CSV files in {outdir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
