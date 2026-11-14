# RTK GPS: two rovers relative to one base

Three u-blox ZED-F9P modules (firmware HPG 1.13):

| Unit   | Computer         | Script          | Output                                |
|--------|------------------|-----------------|---------------------------------------|
| Base   | `p1g5` (base PC) | `rtk_base.py`   | RTCM3 corrections on `tcp://<BASE_IP>:2101` |
| Rover1 | `p1g5` (base PC) | `rtk_rover.py`  | `/tmp/rtk_rover1_*.csv`               |
| Rover2 | other computer   | `rtk_rover.py`  | `/tmp/rtk_rover2_*.csv`               |

```
 base F9P ──USB──> rtk_base.py ──TCP :2101 (raw RTCM3)──┬──> rtk_rover.py (localhost) ──USB──> rover1 F9P
                                                        └──> rtk_rover.py (network)  ──USB──> rover2 F9P
```

Each rover F9P computes **UBX-NAV-RELPOSNED** on board: the vector **from the base antenna to
that rover's antenna** in north / east / down metres. With carrier solution `FIXED`, it's
accurate to about 1–2 cm. Both rovers use the same base, so their vectors share a frame, and the
rover1→rover2 vector is `rover2_NED − rover1_NED` (match rows on `itow_ms`).

The base's absolute survey-in position only sets where the NED frame is anchored. Being a few metres
off there changes the base→rover vectors by about 1e-7 × baseline (sub-mm), so a short survey is enough.

Everything uses the standard library only (python3 ≥ 3.8, no pip installs).

## Wire protocol (for anyone writing their own client)

Plain TCP, no NTRIP handshake. Connect to `<BASE_IP>:2101` and read the byte stream: it contains
only CRC-valid RTCM3 frames. Write those bytes unchanged into the rover F9P's USB port. The base sends
1005 (base position, only after the survey completes), 1074/1084/1094/1124 (GPS/GLO/GAL/BDS MSM4) and
1230, all at 1 Hz.

## Base computer (p1g5): run these

1. One time only: let the other computer reach port 2101 through ufw:
   ```bash
   sudo ufw allow 2101/tcp
   ```
2. Base (leave running):
   ```bash
   python3 rtk_gps/rtk_base.py
   ```
   Wait for `SURVEY COMPLETE` (≥60 s, ≤2 m accuracy; outdoors with open sky takes about 1–3 min). Rovers can
   connect before then. They just report `no base position (1005) yet` until it finishes.
3. Rover1 (second terminal):
   ```bash
   python3 rtk_gps/rtk_rover.py --name rover1 --port /dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0
   ```
4. Find the base IP to give the other computer:
   ```bash
   hostname -I
   ```

On this computer the base and rover enumerate with **identical** `/dev/serial/by-id` names (neither
unit has a USB serial number), so use the `by-path` names, which are tied to the physical USB port:
- port `usb-0:1` (`/dev/ttyACM1` today) = **base**
- port `usb-0:2` (`/dev/ttyACM0` today) = **rover1**

If you swap USB ports, swap the paths. `rtk_base.py --port ...` / `rtk_rover.py --port ...`.

## Other computer (rover2): run these

1. Get this `rtk_gps/` folder (`git pull`, or copy the 3 `.py` files).
2. Find the rover's serial device:
   ```bash
   ls -l /dev/serial/by-id/
   ```
   The u-blox entry points to e.g. `/dev/ttyACM0`. The user needs to be in the `dialout` group.
3. Check that the base is reachable (both computers on the same network/hotspot):
   ```bash
   nc -vz <BASE_IP> 2101
   ```
4. Run:
   ```bash
   python3 rtk_gps/rtk_rover.py --name rover2 --port /dev/ttyACM0 --base <BASE_IP>:2101
   ```

## Reading the rover output

```
[rover] FIXED 3D    sv=24 corr_age= 0.4s | N=  +12.345 E=   -3.210 D=  +0.052 m |  12.756| acc NED=(0.010,0.009,0.015)
```
- 1st field = carrier solution: `none` (code DGNSS, metres), `FLOAT` (decimetres), `FIXED` (cm). Use `FIXED` rows for cm work.
- `corr_age`: seconds since the last bytes from the base. It should stay below about 2 s.
- `|...|`: straight-line distance base→rover.

CSV columns: `wall_time, itow_ms, carr_soln, rel_valid, n_m, e_m, d_m, dist_m, acc_n_m, acc_e_m,
acc_d_m, num_sv, fix_type, lat_deg, lon_deg, h_ell_m, hacc_m, corr_age_s`. `itow_ms` is GPS time of
week. Both rovers run on GPS time, so use it to line up rover1 and rover2 rows (default 5 Hz → 200 ms steps).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `base ... unreachable` | Wrong IP, different network, or ufw on the base PC (step 1). Test with `nc -vz`. |
| `no base position (1005) yet` | Base still surveying. Check the base terminal. |
| Stays `none`/`FLOAT` for minutes | Sky view. Antennas need open sky, away from buildings and people; give FIXED 30–120 s after the base finishes. |
| Garbled data in other tools | Port not in raw mode. These scripts set raw mode themselves; other tools need 8N1, raw, no flow control. |
| `receiver rejected config keys` | Firmware mismatch. Report the key names; the rest of the config is still applied. |

Settings are written to receiver **RAM only**. A power cycle restores each module's saved config,
and the scripts reapply everything on every start.

## Verified on 2026-11-14 (indoors)

All config keys were accepted by both F9Ps. The base streamed 1005/1074/1084/1094/1124/1230 over TCP,
and rover1 received them and output valid RELPOSNED at 5 Hz, code-level only (indoors, about 3 satellites at
the base). Carrier-phase FIXED needs open sky and has not been tested yet.
