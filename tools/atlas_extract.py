#!/usr/bin/env python3
"""
atlas_extract.py - cut ONE session out of a parsed Atlas log directory.

The output has the same files, same names, same headers and the exact same row text as the input
(rows are copied byte-for-byte, nothing is re-parsed or re-formatted), just with one session in it.
So anything that reads the full directory (atlas_view.py, your EKF code) reads the extract unchanged.

    python3 tools/atlas_extract.py flight-1_csv --list
    python3 tools/atlas_extract.py flight-1_csv --session 32 -o flight-1_s32
    python3 tools/atlas_extract.py flight-1_csv --session 1.32 -o flight-1_s32 --force

--session accepts the full id ("1.32"), or just the part after the dot ("32") if that is unique.
Streams line by line, so a 900 MB imu.csv is fine (memory use is tiny).
"""
import argparse
import csv
import shutil
import sys
from collections import OrderedDict
from pathlib import Path


def _header(path):
    with open(path, "rb") as f:
        line = f.readline()
    return line, line.decode("utf-8-sig").strip().split(",")


def _key_fn(cols):
    """Return a function line_bytes -> session id (str), or None if the file has no session column."""
    if "session" not in cols:
        return None
    idx = cols.index("session")
    if idx == 0:                                    # fast path (your files): session is the first column
        return lambda line: line.split(b",", 1)[0].strip().strip(b'"').decode()
    return lambda line: next(csv.reader([line.decode()]))[idx].strip()


def list_sessions(path):
    """OrderedDict session -> row count for one csv file."""
    _, cols = _header(path)
    key = _key_fn(cols)
    counts = OrderedDict()
    if key is None:
        return counts
    with open(path, "rb") as f:
        f.readline()
        for line in f:
            if line.strip():
                k = key(line)
                counts[k] = counts.get(k, 0) + 1
    return counts


def resolve(ids, want):
    want = str(want).strip()
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


def extract_file(src, dst, sid):
    hdr_line, cols = _header(src)
    key = _key_fn(cols)
    dst.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with open(src, "rb") as fi, open(dst, "wb") as fo:
        fi.readline()
        fo.write(hdr_line)
        for line in fi:
            if line.strip() and key(line) == sid:
                fo.write(line)
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("indir", help="parsed log directory (contains imu.csv, baro.csv, ...)")
    ap.add_argument("--session", help="session id, e.g. 32 or 1.32")
    ap.add_argument("-o", "--out", help="output directory (default: <indir>_s<session>)")
    ap.add_argument("--list", action="store_true", help="list sessions (rows per file) and exit")
    ap.add_argument("--force", action="store_true", help="allow writing into an existing non-empty directory")
    a = ap.parse_args()

    src = Path(a.indir)
    files = sorted(p for p in src.rglob("*.csv") if p.is_file())
    if not files:
        raise SystemExit(f"no .csv files under {src}")

    # session ids come from the first file that has a session column (imu.csv preferred)
    files.sort(key=lambda p: (p.name != "imu.csv", str(p)))
    per_file = OrderedDict()
    for p in files:
        per_file[p] = list_sessions(p)
    ids = list(next((c for c in per_file.values() if c), {}).keys())
    allids = list(OrderedDict.fromkeys(i for c in per_file.values() for i in c))

    if a.list or not a.session:
        print(f"{'session':>10}  " + "  ".join(f"{p.relative_to(src)!s:>12}" for p in per_file))
        for i in allids:
            print(f"{i:>10}  " + "  ".join(f"{per_file[p].get(i, 0):12d}" for p in per_file))
        if not a.session and not a.list:
            print("\nGive --session ID (and -o DIR) to extract one.")
        return

    sid = resolve(allids, a.session)
    if sid is None:
        raise SystemExit(f"session {a.session!r} not found. Sessions: {allids}")

    out = Path(a.out) if a.out else Path(f"{str(src).rstrip('/')}_s{sid}")
    if out.resolve() == src.resolve():
        raise SystemExit("output dir must differ from the input dir")
    if out.exists() and any(out.iterdir()) and not a.force:
        raise SystemExit(f"{out} exists and is not empty (use --force to write into it)")
    out.mkdir(parents=True, exist_ok=True)

    print(f"extracting session {sid!r} -> {out}/")
    for p in files:
        rel = p.relative_to(src)
        if not per_file[p]:                         # csv without a session column: copy as is
            shutil.copy2(p, out / rel)
            print(f"  {str(rel):20s} no session column, copied unchanged")
            continue
        n = extract_file(p, out / rel, sid)
        note = "" if n else "   (no rows for this session in this file; header only)"
        print(f"  {str(rel):20s} {n:9d} rows{note}")


if __name__ == "__main__":
    sys.exit(main())
