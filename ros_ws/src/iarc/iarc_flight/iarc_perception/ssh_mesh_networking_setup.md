# Pi SSH Over Wireless Mesh

_Last updated: 2026-11-07_

SSH from a Mac into a Raspberry Pi over an 802.11s wireless mesh (with BATMAN-adv routing), using USB Wi-Fi adapters, with no ethernet cable, router or Wi-Fi access point.

Pis are named `air-##` (e.g. `air-01`, `air-02`) and every PennAiR Pi uses the login username **`penn`**.

## What you need

- A Pi (`air-##`) and a Mac, each with its **own USB Wi-Fi adapter that supports 802.11s mesh** ("mesh point" mode). Not every adapter does; check before you start (Part 1 step 3).
- An Ubuntu VM in UTM on the Mac. BATMAN-adv only runs on Linux, so every Mac step happens inside the VM, not in macOS.
- The Mac's USB Wi-Fi adapter connected **to the VM** through UTM's USB device menu. Being plugged into macOS is not enough.

## Mesh settings

Every node on the mesh must use the same settings:

| Setting | Value |
|---|---|
| Mesh ID | `rosmesh` |
| Frequency | `2437` MHz (channel 6) |
| Channel width | `HT20` |
| Country | `US` |
| Subnet | `10.10.0.0/24` |

Each node needs its **own** IP in `10.10.0.0/24`. Never give two nodes the same address. Pis can have different mesh IPs, so you find the Pi's address with `nmap` in Part 3 instead of assuming it.

## Part 1 — Set up each node (Pi and Ubuntu VM)

Do these steps on **both** the Pi and the Ubuntu VM.

1. **Install the tools.**

   ```bash
   sudo apt update
   sudo apt install -y iw batctl iproute2 kmod nmap
   ```

2. **Find the USB Wi-Fi adapter's interface name.** You'll need it for the rest of the guide, written as `<wifi-interface>`.

   ```bash
   iw dev
   ```

   Each wireless interface shows up as a block like this. The name is on the `Interface` line:

   ```text
   phy#1
       Interface wlx...
           ifindex 4
           addr xx:xx:xx:xx:xx:xx
           type managed
   ```

   - The USB adapter's name almost always starts with **`wlx`** followed by its MAC address (letters and numbers).
   - On the Pi you'll also see **`wlan0`**. That's the Pi's built-in Wi-Fi. Don't use it for the mesh; it can stay on its normal network.
   - In the Ubuntu VM, the USB adapter is usually the only wireless interface. If `iw dev` prints nothing, the adapter isn't connected to the VM. Connect it through UTM's USB device menu and run `iw dev` again.
   - Not sure which one is the adapter? Run `iw dev`, unplug the adapter, then run it again. The interface that disappeared is the adapter.

3. **Check the adapter supports mesh.** Look up which radio (`phy`) the adapter is on, then check that radio:

   ```bash
   iw dev <wifi-interface> info | grep wiphy
   ```

   This prints something like `wiphy 1`. Use that number here:

   ```bash
   iw phy phy1 info | grep -A 10 "Supported interface modes"
   ```

   It must list `* mesh point`. If it doesn't, that adapter won't work for the mesh. (The setup script also checks this and stops with an error if mesh isn't supported.)

4. **Create the setup script** at `~/setup_mesh.sh`:

   ```bash
   nano ~/setup_mesh.sh
   ```

   Paste this in, then save (`Ctrl+O`, `Enter`, `Ctrl+X`):

   ```bash
   #!/usr/bin/env bash
   # Usage: sudo bash setup_mesh.sh <wifi-interface> <mesh-ip/24>
   set -euo pipefail
   export LC_ALL=C

   MESH_ID="rosmesh"
   FREQUENCY=2437
   WIDTH="HT20"
   COUNTRY="US"

   die() { echo "ERROR: $*" >&2; exit 1; }

   (( EUID == 0 )) || die "Run with sudo"
   [[ $# -eq 2 ]] || die "Usage: sudo bash $0 <wifi-interface> <mesh-ip/24>"

   IFACE="$1"
   MESH_IP="$2"

   for cmd in iw ip modprobe awk grep; do
       command -v "$cmd" >/dev/null || die "Missing command: $cmd"
   done

   INFO=$(iw dev "$IFACE" info) || die "Wi-Fi interface not found: $IFACE"
   PHY=$(awk '$1 == "wiphy" {print "phy" $2}' <<< "$INFO")
   [[ -n "$PHY" ]] || die "Cannot identify Wi-Fi radio"

   iw phy "$PHY" info | grep -qE '^[[:space:]]*\* mesh point[[:space:]]*$' \
       || die "Adapter does not support mesh mode"

   if ip -4 addr show dev "$IFACE" | grep -q 'inet '; then
       die "$IFACE has an IPv4 address. Disconnect it from its current network first."
   fi

   modprobe batman-adv || die "BATMAN-adv kernel module unavailable"

   if ip link show dev bat0 >/dev/null 2>&1; then
       ip -d link show dev bat0 | grep -qw batadv \
           || die "bat0 exists but is not a BATMAN-adv interface"
   fi

   echo "Using interface: $IFACE"
   echo "Mesh network: $MESH_ID"

   iw reg set "$COUNTRY"

   # Stop NetworkManager from fighting over the mesh adapter
   command -v nmcli >/dev/null && nmcli device set "$IFACE" managed no

   ip link set dev "$IFACE" down
   ip link set dev "$IFACE" nomaster
   ip addr flush dev "$IFACE"
   iw dev "$IFACE" set type mp
   ip link set dev "$IFACE" mtu 1532
   ip link set dev "$IFACE" up
   iw dev "$IFACE" set power_save off || true
   iw dev "$IFACE" mesh join "$MESH_ID" freq "$FREQUENCY" "$WIDTH" mesh_fwding=0

   ip link show dev bat0 >/dev/null 2>&1 || ip link add name bat0 type batadv
   command -v nmcli >/dev/null && nmcli device set bat0 managed no

   ip link set dev "$IFACE" master bat0
   ip link set dev bat0 up
   ip addr replace "$MESH_IP" dev bat0

   echo
   echo "Mesh setup complete! Mesh IP: $MESH_IP"
   ip -br addr show bat0
   ```

5. **Run it** with your adapter name from step 2 and a mesh IP that no other node is using:

   ```bash
   sudo bash ~/setup_mesh.sh <wifi-interface> 10.10.0.<N>/24
   ```

   The last line should show `bat0` with your IP. `bat0` showing `UNKNOWN` is normal.

   This setup doesn't survive a reboot. Run the script again after every restart, or after reconnecting the adapter.

## Part 2 — Check the mesh link (in the Ubuntu VM)

Once both nodes have run the script, check each layer in order:

1. **Wireless link.** Look for `mesh plink: ESTAB`, which means the two adapters are paired:

   ```bash
   iw dev <wifi-interface> station dump
   ```

2. **BATMAN-adv neighbors.** The Pi's adapter should be listed:

   ```bash
   sudo batctl meshif bat0 neighbors
   ```

3. **BATMAN-adv ping** (optional). Use the MAC address from the `neighbors` output:

   ```bash
   sudo batctl meshif bat0 ping -c 4 <neighbor-mac>
   ```

If all three work but SSH still fails, the problem is the IP layer. Usually it's the wrong Pi IP, which Part 3 finds for you.

## Part 3 — Find the Pi's mesh IP and connect (in the Ubuntu VM)

1. **Scan the mesh subnet** to see every node that's up:

   ```bash
   sudo nmap -sn -e bat0 10.10.0.0/24
   ```

   Each `Nmap scan report for 10.10.0.X` line is a node. Ignore the VM's own mesh IP; the other one is the Pi. If several Pis are on the mesh, SSH into each one and run `hostname` to tell which `air-##` it is.

2. **Check the Pi is reachable and SSH is open:**

   ```bash
   ping -c 4 <pi-mesh-ip>
   nmap -p 22 <pi-mesh-ip>
   ```

   You want `22/tcp open ssh`.

3. **Connect:**

   ```bash
   ssh penn@<pi-mesh-ip>
   ```

   First time: check the fingerprint, type `yes`, then enter the `penn` password. Run `hostname` to confirm you're on the right `air-##`.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `No such device (-19)`, or the adapter vanishes from `iw dev` | USB adapter dropped out of the VM | Reconnect it to the VM in UTM's USB menu, check `iw dev`, rerun the script |
| `mesh point` missing from `iw list` | Adapter or driver doesn't support 802.11s | Use a different adapter |
| Script says the interface has an IPv4 address | Adapter is connected to a normal Wi-Fi network | Disconnect it from that network (or use a different adapter), then rerun |
| No stations in `station dump` | Nodes can't see each other | Check both use the same mesh ID, frequency and width, both scripts ran without errors, and the nodes are in range |
| BATMAN neighbors show up but `ping` to the Pi fails | Wrong Pi IP, or two nodes with the same IP | Rescan with `nmap` (Part 3 step 1); check `ip -br addr show bat0` on each node for duplicates |
| `ping` works but port 22 is `closed` | SSH isn't running on the Pi | On the Pi: `sudo systemctl enable --now ssh` |
| Port 22 is `filtered` | Firewall on the Pi | Check the Pi's firewall rules |
| Mesh gone after a reboot | The script's settings don't persist | Rerun the script on that node |

## Notes

- The mesh itself is **not encrypted**. SSH traffic is encrypted, but anything else sent over the mesh is not.
- The Pi's built-in Wi-Fi can stay connected to a normal network while the USB adapter handles the mesh.

## Useful commands

| Command | Purpose |
|---|---|
| `iw dev` | List wireless interfaces |
| `iw list` | Show adapter capabilities |
| `ip -br addr show bat0` | Show this node's mesh IP |
| `iw dev <wifi-interface> station dump` | Show 802.11s peers |
| `sudo batctl meshif bat0 neighbors` | Show BATMAN-adv neighbors |
| `sudo batctl meshif bat0 originators` | Show mesh routes |
| `sudo nmap -sn -e bat0 10.10.0.0/24` | Find every node on the mesh |
| `ip neigh show dev bat0` | Check ARP (`FAILED` = nothing answered at that IP) |
| `ssh penn@<pi-mesh-ip>` | Connect to the Pi |
