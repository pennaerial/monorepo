#!/usr/bin/env python3
"""RTK rover: feed base RTCM3 (from rtk_base.py over TCP) into a ZED-F9P and log the
rover's position relative to the base (UBX-NAV-RELPOSNED, north/east/down in metres).

Same script on every rover computer; only --port and --base change.

    # on the base computer (rover plugged in next to the base)
    python3 rtk_gps/rtk_rover.py --name rover1 --port /dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0

    # on the other computer
    python3 rtk_gps/rtk_rover.py --name rover2 --port /dev/ttyACM0 --base <BASE_IP>:2101

CSV log: /tmp/rtk_<name>_<YYYYmmdd_HHMMSS>.csv (one row per navigation epoch).
"""

import argparse
import csv
import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ubx  # noqa: E402

CSV_FIELDS = [
    "wall_time", "itow_ms", "carr_soln", "rel_valid", "n_m", "e_m", "d_m", "dist_m",
    "acc_n_m", "acc_e_m", "acc_d_m", "num_sv", "fix_type", "lat_deg", "lon_deg", "h_ell_m",
    "hacc_m", "corr_age_s",
]


class BaseLink:
    """TCP client to rtk_base.py with automatic reconnect."""

    def __init__(self, hostport):
        host, port = hostport.rsplit(":", 1)
        self.addr = (host, int(port))
        self.sock = None
        self.next_try = 0.0
        self.last_rx = None
        self.bytes_rx = 0

    def fileno(self):
        return self.sock.fileno() if self.sock else None

    def poll_connect(self):
        if self.sock or time.time() < self.next_try:
            return
        try:
            s = socket.create_connection(self.addr, timeout=2.0)
            s.setblocking(False)
            self.sock = s
            print(f"[rover] connected to base {self.addr[0]}:{self.addr[1]}", flush=True)
        except OSError as e:
            print(f"[rover] base {self.addr[0]}:{self.addr[1]} unreachable ({e}); retrying", flush=True)
            self.next_try = time.time() + 3.0

    def read(self):
        try:
            data = self.sock.recv(65536)
        except BlockingIOError:
            return b""
        except OSError:
            data = b""
        if not data:
            print("[rover] base connection lost; reconnecting", flush=True)
            self.sock.close()
            self.sock = None
            self.next_try = time.time() + 1.0
            return b""
        self.last_rx = time.time()
        self.bytes_rx += len(data)
        return data


def configure(dev, rate_hz):
    return dev.valset([
        ("TMODE_MODE", 0),  # make sure it's not acting as a base
        ("USBINPROT_UBX", 1),
        ("USBINPROT_RTCM3X", 1),
        ("USBOUTPROT_UBX", 1),
        ("USBOUTPROT_NMEA", 0),
        ("USBOUTPROT_RTCM3X", 0),
        ("RATE_MEAS", int(1000 / rate_hz)),
        ("MSGOUT_UBX_NAV_PVT_USB", 1),
        ("MSGOUT_UBX_NAV_RELPOSNED_USB", 1),
    ])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="rover receiver serial device")
    ap.add_argument("--base", default="127.0.0.1:2101", help="rtk_base.py TCP host:port")
    ap.add_argument("--name", default=socket.gethostname(), help="label used in the log filename")
    ap.add_argument("--rate", type=float, default=5.0, help="navigation rate, Hz")
    ap.add_argument("--log-dir", default="/tmp")
    args = ap.parse_args()

    dev = ubx.Device(args.port)
    print(f"[rover] {args.port}: {dev.mon_ver() or 'no MON-VER reply'}")
    bad = configure(dev, args.rate)
    if bad:
        print(f"[rover] WARNING receiver rejected config keys: {bad}")

    log_path = os.path.join(args.log_dir, f"rtk_{args.name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    log_f = open(log_path, "w", newline="")
    log = csv.DictWriter(log_f, fieldnames=CSV_FIELDS)
    log.writeheader()
    print(f"[rover] logging to {log_path}")

    link = BaseLink(args.base)
    rtcm_parser = ubx.StreamParser()
    rtcm_types = set()
    pvt = None
    last_print = 0.0

    try:
        while True:
            link.poll_connect()
            fds = [dev.fd] + ([link.sock] if link.sock else [])
            r, _, _ = select.select(fds, [], [], 0.2)

            if link.sock and link.sock in r:
                data = link.read()
                if data:
                    # Forward only CRC-valid RTCM frames to the receiver.
                    for m in rtcm_parser.feed(data):
                        if m[0] == "rtcm":
                            rtcm_types.add(m[1])
                            dev.write(m[2])

            if dev.fd in r:
                for m in dev.read_messages(0):
                    if m[0] != "ubx" or m[1] != 0x01:
                        continue
                    if m[2] == 0x07:
                        pvt = ubx.parse_nav_pvt(m[3])
                    elif m[2] == 0x3C:
                        rel = ubx.parse_nav_relposned(m[3])
                        if not rel:
                            continue
                        age = time.time() - link.last_rx if link.last_rx else None
                        p = pvt if pvt and pvt["itow"] == rel["itow"] else None
                        log.writerow({
                            "wall_time": f"{time.time():.3f}",
                            "itow_ms": rel["itow"],
                            "carr_soln": ubx.CARR[rel["carr"]],
                            "rel_valid": int(rel["valid"]),
                            "n_m": f"{rel['n']:.4f}",
                            "e_m": f"{rel['e']:.4f}",
                            "d_m": f"{rel['d']:.4f}",
                            "dist_m": f"{rel['len']:.4f}",
                            "acc_n_m": f"{rel['acc_n']:.4f}",
                            "acc_e_m": f"{rel['acc_e']:.4f}",
                            "acc_d_m": f"{rel['acc_d']:.4f}",
                            "num_sv": p["nsv"] if p else "",
                            "fix_type": ubx.FIX.get(p["fix"], p["fix"]) if p else "",
                            "lat_deg": f"{p['lat']:.8f}" if p else "",
                            "lon_deg": f"{p['lon']:.8f}" if p else "",
                            "h_ell_m": f"{p['h']:.3f}" if p else "",
                            "hacc_m": f"{p['hacc']:.3f}" if p else "",
                            "corr_age_s": f"{age:.1f}" if age is not None else "",
                        })
                        log_f.flush()

                        now = time.time()
                        if now - last_print >= 1.0:
                            last_print = now
                            nsv = p["nsv"] if p else (pvt["nsv"] if pvt else 0)
                            fix = ubx.FIX.get(pvt["fix"], "?") if pvt else "?"
                            agestr = f"{age:4.1f}s" if age is not None else "  - "
                            if rel["valid"]:
                                body = (f"N={rel['n']:+9.3f} E={rel['e']:+9.3f} D={rel['d']:+8.3f} m "
                                        f"|{rel['len']:8.3f}| acc NED=({rel['acc_n']:.3f},"
                                        f"{rel['acc_e']:.3f},{rel['acc_d']:.3f})")
                            else:
                                why = []
                                if not link.sock:
                                    why.append("no base link")
                                elif rel["ref_pos_miss"] or 1005 not in rtcm_types:
                                    why.append("no base position (1005) yet - base still surveying?")
                                if rel["ref_obs_miss"]:
                                    why.append("no base observations")
                                if not rel["gnss_ok"]:
                                    why.append("rover has no fix")
                                body = "relpos invalid: " + (", ".join(why) or "waiting")
                            print(f"[rover] {ubx.CARR[rel['carr']]:5s} {fix:5s} sv={nsv:2d} "
                                  f"corr_age={agestr} | {body}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        log_f.close()
        dev.close()
        print(f"[rover] log saved: {log_path}")


if __name__ == "__main__":
    main()
