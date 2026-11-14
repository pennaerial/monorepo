#!/usr/bin/env python3
"""RTK base station: configure a ZED-F9P as a survey-in base and serve its RTCM3 over TCP.

Every rover (local or on another computer) connects to this TCP port, receives the raw
RTCM3 byte stream, and writes it into its own receiver. See README.md.

    python3 rtk_gps/rtk_base.py                     # defaults below
    python3 rtk_gps/rtk_base.py --port /dev/ttyACM1 --listen 0.0.0.0:2101
"""

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ubx  # noqa: E402

DEFAULT_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0"

# MSM4 for every constellation the F9P tracks + base position + GLONASS biases.
# This is u-blox's recommended base set; MSM7 variants are turned off to save bandwidth.
RTCM_ON = ["1005", "1074", "1084", "1094", "1124", "1230"]
RTCM_OFF = ["1077", "1087", "1097", "1127"]


class Client:
    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = f"{addr[0]}:{addr[1]}"
        self.pending = b""
        self.sock.setblocking(False)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def send(self, data):
        """Queue + flush. Returns False if the client should be dropped."""
        self.pending += data
        if len(self.pending) > 256 * 1024:
            return False  # stalled client
        try:
            n = self.sock.send(self.pending)
            self.pending = self.pending[n:]
        except BlockingIOError:
            pass
        except OSError:
            return False
        return True


def configure(dev, args):
    items = [
        ("USBOUTPROT_UBX", 1),
        ("USBOUTPROT_NMEA", 0),
        ("USBOUTPROT_RTCM3X", 1),
        ("RATE_MEAS", 1000),
        ("TMODE_SVIN_MIN_DUR", args.svin_dur),
        ("TMODE_SVIN_ACC_LIMIT", int(args.svin_acc * 1e4)),
    ]
    items += [(f"MSGOUT_RTCM_{t}_USB", 1) for t in RTCM_ON]
    items += [(f"MSGOUT_RTCM_{t}_USB", 0) for t in RTCM_OFF]
    bad = dev.valset(items)
    # Toggle mode off->survey-in so a fresh survey starts with the new limits.
    bad += dev.valset([("TMODE_MODE", 0)])
    time.sleep(0.2)
    bad += dev.valset([("TMODE_MODE", 1)])
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT, help="base receiver serial device")
    ap.add_argument("--listen", default="0.0.0.0:2101", help="TCP host:port to serve RTCM3 on")
    ap.add_argument("--svin-dur", type=int, default=60, help="survey-in minimum duration, s")
    ap.add_argument(
        "--svin-acc",
        type=float,
        default=2.0,
        help="survey-in accuracy limit, m. Only shifts the absolute frame; the base->rover "
        "vectors are cm-accurate regardless (error scales ~1e-7 x baseline per m).",
    )
    args = ap.parse_args()

    dev = ubx.Device(args.port)
    print(f"[base] {args.port}: {dev.mon_ver() or 'no MON-VER reply'}")
    bad = configure(dev, args)
    if bad:
        print(f"[base] WARNING receiver rejected config keys: {bad}")
    print(f"[base] survey-in: min {args.svin_dur}s, accuracy <= {args.svin_acc} m")

    host, port = args.listen.rsplit(":", 1)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, int(port)))
    srv.listen(8)
    srv.setblocking(False)
    print(f"[base] serving RTCM3 on tcp://{args.listen}")

    clients = []
    counts = {}
    svin = None
    announced = False
    last_poll = 0.0

    def handle(m):
        nonlocal svin
        if m[0] == "rtcm":
            counts[m[1]] = counts.get(m[1], 0) + 1
            for c in list(clients):
                if not c.send(m[2]):
                    print(f"[base] client {c.addr} dropped")
                    c.sock.close()
                    clients.remove(c)
        elif m[0] == "ubx" and (m[1], m[2]) == (0x01, 0x3B):
            svin = ubx.parse_nav_svin(m[3])

    try:
        while True:
            try:
                s, addr = srv.accept()
                clients.append(Client(s, addr))
                print(f"[base] client connected: {clients[-1].addr}")
            except BlockingIOError:
                pass

            for m in dev.read_messages(0.05):
                handle(m)

            now = time.time()
            if now - last_poll >= 1.0:
                last_poll = now
                dev.write(ubx.ubx_frame(0x01, 0x3B))  # poll NAV-SVIN; reply lands in handle()
                if svin:
                    state = "VALID" if svin["valid"] else ("surveying" if svin["active"] else "idle")
                    rtcm = " ".join(f"{k}:{v}" for k, v in sorted(counts.items())) or "none"
                    print(
                        f"[base] svin {state:9s} {svin['dur']:4d}s acc={svin['acc']:7.2f}m "
                        f"obs={svin['obs']:4d} | clients={len(clients)} | rtcm/s {rtcm}",
                        flush=True,
                    )
                    if svin["valid"] and not announced:
                        announced = True
                        x, y, z = (v / 100 for v in svin["ecef_cm"])
                        print(f"[base] SURVEY COMPLETE. Base ECEF = ({x:.2f}, {y:.2f}, {z:.2f}) m; 1005 now broadcast")
                counts.clear()
    except KeyboardInterrupt:
        pass
    finally:
        for c in clients:
            c.sock.close()
        srv.close()
        dev.close()


if __name__ == "__main__":
    main()
