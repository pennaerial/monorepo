# Ethernet, Mesh, and SSH

Every Pi's addresses end in its number. For example, `air-09` is `192.168.50.9`
on Ethernet and `10.10.0.9` on the mesh. A laptop is `.254` on both.

## Ethernet

Connect the Pi to your laptop with an Ethernet cable, then connect using your
Pi's name (e.g. `air-09`):

```bash
ssh penn@air-09.local    # or ssh penn@192.168.50.9
```

The password is `123`. The Pi gives your laptop `192.168.50.254` when you plug in.

**Only plug a Pi's Ethernet into a laptop or a switch with nothing but Pis on it.**
The Pi hands out addresses on that port, which disrupts other networks.

If the laptop doesn't get an address, set it by hand.

Linux, replacing `<adapter>` with the wired interface from `ip -br link` (likely `eth0`):

```bash
sudo ip addr add 192.168.50.254/24 dev <adapter>
```

macOS: **System Settings → Network → your Ethernet adapter → Details → TCP/IP**.
Set **Configure IPv4** to **Manually**, IP address `192.168.50.254`, subnet mask
`255.255.255.0`, and leave Router blank.

## Mesh

A Pi joins the mesh by itself when its USB Wi-Fi adapter is plugged in.

A Linux laptop needs a mesh-capable USB Wi-Fi adapter (e.g. the MediaTek MT7612U
the Pis use) and `iw` (`sudo apt install iw`). Save this script as `~/mesh-up.sh`:

```bash
#!/bin/bash
# Joins the Pi mesh from a Linux laptop.
#   sudo bash ~/mesh-up.sh <wifi interface> [mesh ip]
# Without an IP, the laptop takes 10.10.0.254.
set -e

WIFI="${1:?Usage: mesh-up.sh <wifi interface> [mesh ip]}"
MESH_IP="${2:-10.10.0.254}"

# stop NetworkManager from taking the adapter back
nmcli device set "$WIFI" managed no
iw reg set US
ip link set "$WIFI" down
iw dev "$WIFI" set type mp
ip link set "$WIFI" mtu 1532
ip link set "$WIFI" up
iw dev "$WIFI" mesh join rosmesh freq 2437 HT20
iw dev "$WIFI" set mesh_param mesh_fwding 0

ip link show bat0 > /dev/null 2>&1 || ip link add bat0 type batadv
ip link set "$WIFI" master bat0
ip link set bat0 up
ip addr replace "$MESH_IP/24" dev bat0

echo "$WIFI joined the mesh as $MESH_IP"
```

Find the adapter's interface name with `iw dev` (e.g. `wlx00c0cabe2451`), then run:

```bash
sudo bash ~/mesh-up.sh wlx00c0cabe2451
```

If a second laptop joins, give it a different address (e.g. `10.10.0.253`):

```bash
sudo bash ~/mesh-up.sh wlx00c0cabe2451 10.10.0.253
```

To check the mesh, ping a Pi that's powered on with its adapter plugged in
(e.g. `air-09`):

```bash
ping -c 4 10.10.0.9
```

The laptop stays on the mesh until it reboots or the adapter is unplugged.

## Sharing the laptop's internet

A Pi uses its own Wi-Fi for internet if it has one. Otherwise it sends internet
traffic to the laptop at `.254`, over Ethernet first and then the mesh. To make
your Linux laptop pass that traffic on:

```bash
pennair gateway on
pennair gateway off
```
