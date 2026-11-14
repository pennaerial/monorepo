"""Minimal u-blox ZED-F9P helpers (stdlib only): serial, UBX framing, stream parser, VALSET.

Shared by rtk_base.py and rtk_rover.py. No third-party dependencies, so the same files run
on any Linux machine with python3.
"""

import os
import select
import struct
import termios
import time
import tty

# --- config keys (u-blox F9 HPG 1.13 interface description) --------------------------------
# Size of the value is encoded in bits 28..30 of the key id.
KEYS = {
    # Ports / protocols on USB
    "USBINPROT_UBX": 0x10770001,
    "USBINPROT_NMEA": 0x10770002,
    "USBINPROT_RTCM3X": 0x10770004,
    "USBOUTPROT_UBX": 0x10780001,
    "USBOUTPROT_NMEA": 0x10780002,
    "USBOUTPROT_RTCM3X": 0x10780004,
    # Time mode (base station)
    "TMODE_MODE": 0x20030001,  # 0 disabled, 1 survey-in, 2 fixed
    "TMODE_SVIN_MIN_DUR": 0x40030010,  # s
    "TMODE_SVIN_ACC_LIMIT": 0x40030011,  # 0.1 mm
    # Navigation rate
    "RATE_MEAS": 0x30210001,  # ms
    # UBX messages on USB
    "MSGOUT_UBX_NAV_PVT_USB": 0x20910009,
    "MSGOUT_UBX_NAV_RELPOSNED_USB": 0x20910090,
    # RTCM output on USB
    "MSGOUT_RTCM_1005_USB": 0x209102C0,
    "MSGOUT_RTCM_1074_USB": 0x20910361,
    "MSGOUT_RTCM_1077_USB": 0x209102CF,
    "MSGOUT_RTCM_1084_USB": 0x20910366,
    "MSGOUT_RTCM_1087_USB": 0x209102D4,
    "MSGOUT_RTCM_1094_USB": 0x2091036B,
    "MSGOUT_RTCM_1097_USB": 0x2091031B,
    "MSGOUT_RTCM_1124_USB": 0x20910370,
    "MSGOUT_RTCM_1127_USB": 0x209102D9,
    "MSGOUT_RTCM_1230_USB": 0x20910306,
}

_SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}


def open_serial(path):
    """Open a USB-CDC serial port in raw, non-blocking mode.

    Raw mode matters: the default tty line discipline translates CR->NL and eats bytes,
    which corrupts binary UBX/RTCM frames.
    """
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[4] = attrs[5] = termios.B115200  # ignored by USB-CDC but harmless
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def write_all(fd, data, timeout=2.0):
    """Write all bytes to a non-blocking fd, waiting for room as needed."""
    end = time.time() + timeout
    while data:
        if time.time() > end:
            raise TimeoutError("serial write timed out")
        select.select([], [fd], [], 0.1)
        try:
            n = os.write(fd, data)
            data = data[n:]
        except BlockingIOError:
            pass


def ubx_checksum(body):
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])


def ubx_frame(cls, mid, payload=b""):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + ubx_checksum(body)


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class StreamParser:
    """Splits a mixed UBX / RTCM3 / NMEA byte stream into validated messages.

    feed() returns a list of tuples:
      ("ubx", cls, id, payload)
      ("rtcm", msg_type, raw_frame_bytes)
      ("nmea", line_str)
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        b = self.buf
        i = 0
        n = len(b)
        while i < n:
            c = b[i]
            if c == 0xB5:
                if n - i < 8:
                    break
                if b[i + 1] != 0x62:
                    i += 1
                    continue
                length = b[i + 4] | (b[i + 5] << 8)
                if length > 8192:
                    i += 1
                    continue
                end = i + 8 + length
                if end > n:
                    break
                if ubx_checksum(b[i + 2 : end - 2]) == bytes(b[end - 2 : end]):
                    out.append(("ubx", b[i + 2], b[i + 3], bytes(b[i + 6 : end - 2])))
                    i = end
                else:
                    i += 1
            elif c == 0xD3:
                if n - i < 6:
                    break
                if b[i + 1] & 0xFC:
                    i += 1
                    continue
                length = ((b[i + 1] & 0x03) << 8) | b[i + 2]
                end = i + 6 + length
                if end > n:
                    break
                frame = bytes(b[i:end])
                if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                    mtype = (frame[3] << 4) | (frame[4] >> 4) if length >= 2 else 0
                    out.append(("rtcm", mtype, frame))
                    i = end
                else:
                    i += 1
            elif c == 0x24:  # '$'
                j = b.find(b"\n", i)
                if j < 0:
                    if n - i > 200:
                        i += 1
                        continue
                    break
                out.append(("nmea", bytes(b[i:j]).decode("ascii", "replace").strip()))
                i = j + 1
            else:
                i += 1
        del self.buf[:i]
        return out


def _encode_kv(name, value):
    key = KEYS[name]
    size = _SIZE[(key >> 28) & 0x7]
    return struct.pack("<I", key) + int(value).to_bytes(size, "little", signed=False)


class Device:
    """A u-blox receiver on a serial port: read loop + request/response helpers."""

    def __init__(self, path):
        self.path = path
        self.fd = open_serial(path)
        self.parser = StreamParser()

    def close(self):
        os.close(self.fd)

    def read_messages(self, timeout=0.0):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return []
        try:
            data = os.read(self.fd, 65536)
        except BlockingIOError:
            return []
        if not data:
            raise OSError(f"{self.path}: device disconnected")
        return self.parser.feed(data)

    def write(self, data):
        write_all(self.fd, data)

    def request(self, cls, mid, payload=b"", reply=None, timeout=1.5, sink=None):
        """Send a UBX frame and wait for a reply.

        reply=(cls,id) waits for that message and returns its payload.
        reply=None waits for ACK-ACK/ACK-NAK on (cls,id) and returns True/False.
        Messages seen while waiting are passed to sink (if given) so nothing is lost.
        """
        self.write(ubx_frame(cls, mid, payload))
        end = time.time() + timeout
        while time.time() < end:
            for m in self.read_messages(0.05):
                if m[0] == "ubx":
                    _, c, i, p = m
                    if reply is None and c == 0x05 and len(p) >= 2 and p[0] == cls and p[1] == mid:
                        return i == 0x01
                    if reply is not None and (c, i) == reply:
                        return p
                if sink:
                    sink(m)
        return None

    def valset(self, items, layers=0x01, sink=None):
        """Set config items in RAM (layers=1). Falls back to one-at-a-time on NAK.

        Returns a list of key names the receiver rejected. RAM-only means a power cycle
        restores the module's saved config; the scripts re-apply settings on every start.
        """
        payload = bytes([0x00, layers, 0, 0]) + b"".join(_encode_kv(k, v) for k, v in items)
        if self.request(0x06, 0x8A, payload, sink=sink):
            return []
        bad = []
        for k, v in items:
            p = bytes([0x00, layers, 0, 0]) + _encode_kv(k, v)
            if not self.request(0x06, 0x8A, p, sink=sink):
                bad.append(k)
        return bad

    def mon_ver(self):
        p = self.request(0x0A, 0x04, reply=(0x0A, 0x04))
        if not p:
            return None
        fields = [p[k : k + 30].split(b"\0")[0].decode(errors="replace") for k in range(40, len(p), 30)]
        mod = next((f[4:] for f in fields if f.startswith("MOD=")), "?")
        fw = next((f[6:] for f in fields if f.startswith("FWVER=")), "?")
        return f"{mod} fw {fw}"


# --- message decoders ----------------------------------------------------------------------

CARR = {0: "none", 1: "FLOAT", 2: "FIXED"}
FIX = {0: "nofix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "time"}


def parse_nav_pvt(p):
    if len(p) < 92:
        return None
    return {
        "itow": struct.unpack_from("<I", p, 0)[0],
        "fix": p[20],
        "gnss_ok": bool(p[21] & 0x01),
        "diff": bool(p[21] & 0x02),
        "carr": (p[21] >> 6) & 0x3,
        "nsv": p[23],
        "lon": struct.unpack_from("<i", p, 24)[0] * 1e-7,
        "lat": struct.unpack_from("<i", p, 28)[0] * 1e-7,
        "h": struct.unpack_from("<i", p, 32)[0] * 1e-3,
        "hacc": struct.unpack_from("<I", p, 40)[0] * 1e-3,
        "vacc": struct.unpack_from("<I", p, 44)[0] * 1e-3,
    }


def parse_nav_relposned(p):
    """UBX-NAV-RELPOSNED v1 (F9): vector from the base (reference station) to this rover."""
    if len(p) < 64 or p[0] != 0x01:
        return None
    n, e, d, length = struct.unpack_from("<iiii", p, 8)
    hpn, hpe, hpd, hpl = struct.unpack_from("<bbbb", p, 32)
    accn, acce, accd, accl = struct.unpack_from("<IIII", p, 36)
    flags = struct.unpack_from("<I", p, 60)[0]
    return {
        "ref_id": struct.unpack_from("<H", p, 2)[0],
        "itow": struct.unpack_from("<I", p, 4)[0],
        "n": n * 1e-2 + hpn * 1e-4,
        "e": e * 1e-2 + hpe * 1e-4,
        "d": d * 1e-2 + hpd * 1e-4,
        "len": length * 1e-2 + hpl * 1e-4,
        "acc_n": accn * 1e-4,
        "acc_e": acce * 1e-4,
        "acc_d": accd * 1e-4,
        "gnss_ok": bool(flags & 0x01),
        "diff": bool(flags & 0x02),
        "valid": bool(flags & 0x04),
        "carr": (flags >> 3) & 0x3,
        "ref_pos_miss": bool(flags & 0x40),
        "ref_obs_miss": bool(flags & 0x80),
    }


def parse_nav_svin(p):
    if len(p) < 40:
        return None
    mx, my, mz = struct.unpack_from("<iii", p, 12)
    return {
        "dur": struct.unpack_from("<I", p, 8)[0],
        "ecef_cm": (mx, my, mz),
        "acc": struct.unpack_from("<I", p, 28)[0] * 1e-4,
        "obs": struct.unpack_from("<I", p, 32)[0],
        "valid": bool(p[36]),
        "active": bool(p[37]),
    }
