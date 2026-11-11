#!/usr/bin/env python3
"""Copy the flight log off the board over USB.

    python3 tools/atlas_dump.py /dev/ttyACM0 -o flight.bin          # dump everything that was logged
    python3 tools/atlas_dump.py /dev/ttyACM0 --info                  # just show how much is stored
    python3 tools/atlas_dump.py /dev/ttyACM0 --erase                 # forget the stored log (asks first)

The first command pauses logging on the board until it is reset. Needs pyserial (pip install pyserial).
"""
import argparse, subprocess, sys, time


class SerialTransport:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=5)
    def write(self, b): self.s.write(b); self.s.flush()
    def readline(self): return self.s.readline()
    def read(self, n): return self.s.read(n)
    def drain(self): self.s.reset_input_buffer()


class PipeTransport:
    """Talks to a program over stdin/stdout instead of a serial port (used by the host tests)."""
    def __init__(self, cmd):
        self.p = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    def write(self, b): self.p.stdin.write(b); self.p.stdin.flush()
    def readline(self): return self.p.stdout.readline()
    def read(self, n): return self.p.stdout.read(n)
    def drain(self): pass


def expect_line(t, prefix, tries=200):
    for _ in range(tries):
        line = t.readline()
        if not line:
            break
        if line.startswith(prefix):
            return line.decode(errors="replace").strip()
    raise SystemExit(f"no '{prefix.decode()}' reply from the board (is it running the logger firmware?)")


def get_info(t):
    t.drain()
    t.write(b"i\n")
    reply = expect_line(t, b"ATLAS used=")
    kv = dict(p.split("=") for p in reply.split()[1:])
    return {k: int(v) for k, v in kv.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. /dev/ttyACM0")
    ap.add_argument("-o", "--output", default="flight.bin")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--erase", action="store_true")
    ap.add_argument("--start", type=int, default=1, help="first sector (default 1)")
    ap.add_argument("--count", type=int, default=None, help="number of sectors (default: everything logged)")
    ap.add_argument("--exec", action="store_true", help="treat PORT as a command to run (for tests)")
    args = ap.parse_args()

    t = PipeTransport(args.port) if args.exec else SerialTransport(args.port)
    info = get_info(t)
    print(f"card: {info['used']} log blocks used of {info['total']} sectors, generation {info['gen']}, last session {info['session']}")
    if args.info:
        return 0
    if args.erase:
        if input("Erase the stored log? Type 'erase' to confirm: ").strip() != "erase":
            print("cancelled"); return 1
        t.write(b"x!\n")
        print(expect_line(t, b"ATLAS erase"))
        return 0

    count = args.count if args.count is not None else info["used"] - (args.start - 1)
    if count <= 0:
        print("nothing to dump"); return 0
    t.write(f"r {args.start} {count}\n".encode())
    head = expect_line(t, b"ATLD ").split()
    start, n = int(head[1]), int(head[2])
    total = n * 512
    got, t0 = 0, time.time()
    with open(args.output, "wb") as f:
        while got < total:
            chunk = t.read(min(65536, total - got))
            if not chunk:
                raise SystemExit(f"\ntimed out after {got} of {total} bytes (the board stopped sending)")
            f.write(chunk)
            got += len(chunk)
            print(f"\r{got // 512}/{n} sectors ({got / 1e6 / max(time.time() - t0, 1e-3):.2f} MB/s)", end="", flush=True)
    print(f"\nsaved {n} sectors (from sector {start}) to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
