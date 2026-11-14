# Ethernet and SSH

Connect the Pi to your laptop with an Ethernet cable, then:

```bash
ssh penn@air-09.local    # or ssh penn@192.168.50.9
```

The password is `123`.

Each Pi has the address `192.168.50.<number>`: `air-09` is `192.168.50.9`. The Pi
gives your laptop `192.168.50.254` when you plug in.

**Only plug a Pi's Ethernet into a laptop or a switch with nothing but Pis on it.**
The Pi hands out addresses on that port, which disrupts other networks.

## If the laptop doesn't get an address

Set it by hand.

Linux, replacing `<adapter>` with the wired interface from `ip -br link`:

```bash
sudo ip addr add 192.168.50.254/24 dev <adapter>
```

macOS: **System Settings → Network → your Ethernet adapter → Details → TCP/IP**.
Set **Configure IPv4** to **Manually**, IP address `192.168.50.254`, subnet mask
`255.255.255.0`, and leave Router blank.
