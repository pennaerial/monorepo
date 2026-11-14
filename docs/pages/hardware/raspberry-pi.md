# Raspberry Pi Setup

You need:

- a Raspberry Pi 4 and a 32 GB or larger SD card
- a Linux laptop with a card reader and the `pennair` CLI installed (see `rust/cli/README.md`)
- an Ethernet cable
- a MediaTek MT7612U USB Wi-Fi adapter for the mesh

## Assigned hostnames

Update this table when you assign or retire an SD card. The hostname follows the
SD card when it moves to another Pi.

| Hostname | Mesh IP | Ethernet IP |
| --- | --- | --- |
| `air-04` | `10.10.0.4` | `192.168.50.4` |
| `air-05` | `10.10.0.5` | `192.168.50.5` |
| `air-06` | `10.10.0.6` | `192.168.50.6` |
| `air-07` | `10.10.0.7` | `192.168.50.7` |
| `air-08` | `10.10.0.8` | `192.168.50.8` |

## 1. Flash the SD card

Insert the card and find its device name:

```bash
lsblk -o NAME,SIZE,MODEL,MOUNTPOINTS
```

Replace `/dev/sdX` below with the card's device from that list (e.g. `/dev/sda`).
Use the whole card (`/dev/sda`), not a partition (`/dev/sda1`).

Flash it with an unused number from 1 to 99. For example, `9` makes `air-09`.
**Flashing erases the card.**

```bash
pennair flash 9 /dev/sdX
```

On WSL, attach the USB card reader first with `usbipd attach --wsl --busid <busid>`
in PowerShell, then run `sudo modprobe usb-storage` in WSL.

## 2. Boot and connect

Put the card in the Pi, connect Ethernet to your laptop, and power it on. The
first boot takes about two minutes. Then connect, using your Pi's name (e.g. `air-09`):

```bash
ssh penn@air-09.local
```

The password is `123`. See [Ethernet, Mesh, and SSH](ethernet.md) if it doesn't connect.

## 3. Push the code

Again with your Pi's name (e.g. `air-09`):

```bash
pennair push penn@air-09.local --no-restart
```

The first push to a new card takes 15 to 20 minutes, because the Pi builds
`px4_msgs`. Later pushes take under a minute.

## 4. Check the mesh

Plug in the USB Wi-Fi adapter. On the Pi, run:

```bash
ip -br address show bat0
```

The address ends in the Pi's number (e.g. `10.10.0.9/24` for `air-09`). Then ping
another Pi that's powered on and on the mesh (e.g. `air-06`):

```bash
ping -c 4 10.10.0.6
```

## Wi-Fi

The image has no saved Wi-Fi networks. To add one:

```bash
sudo nmcli device wifi connect "<network>" password "<password>"
```
