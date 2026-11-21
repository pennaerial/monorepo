# Raspberry Pi Setup

Use the club image, which already includes Ubuntu 24.04 and the Wi-Fi
mesh setup. Ask the software team for the image and login password.

The most recent image is `nov7.img` (November 7, 2026).

This setup was tested on a Raspberry Pi 4 with a MediaTek MT7612U USB Wi-Fi adapter.
You will also need a 32 GB or larger SD card, a card reader, and an Ethernet cable.

## Assigned hostnames

These names are already in use. Update this table when assigning or retiring an
SD card. The hostname follows the SD card when it is moved to another Pi.

| Hostname | Mesh IP | Notes |
| --- | --- | --- |
| `air-04` | `10.10.0.4` | |
| `air-05` | `10.10.0.5` | |
| `air-06` | `10.10.0.6` | Source of the current base image |
| `air-07` | `10.10.0.7` | |
| `air-08` | `10.10.0.8` | |

## 1. Flash the SD card

> **TODO:** Add macOS flashing instructions.

On your Linux laptop, open a terminal in the folder containing the image. The
commands below use `<image>.img`; replace it with your image's filename before
running them.

Insert the new SD card and identify it:

```bash
lsblk -o NAME,SIZE,MODEL,MOUNTPOINTS
```

The commands below assume the card is `/dev/mmcblk0`. If yours differs, use the
device and partition names shown by `lsblk`. **Flashing erases the selected card.**

Unmount the card's partitions:

```bash
sudo umount /dev/mmcblk0p*
```

"Not mounted" is fine. If a partition is busy, close windows or terminals using
the card and retry. Then write the image to the whole card:

```bash
sudo dd if="<image>.img" of=/dev/mmcblk0 bs=4M status=progress conv=fsync
```

Wait for the command to finish successfully, then verify the copy:

```bash
sudo cmp -n "$(stat -c %s "<image>.img")" "<image>.img" /dev/mmcblk0
```

Returning to the prompt without output means the copy matches. Safely eject the
card before removing it.

## 2. Boot and connect over Ethernet

Keep the other Pis powered off until setup is complete. Insert the card into the
new Pi and leave its USB Wi-Fi adapter unplugged.

Follow [Ethernet and SSH](ethernet.md) to connect from your laptop, then return
here. Run the next two steps inside the Pi SSH session.

## 3. Give the Pi a unique name

Choose an unused name with exactly two digits and add it to the table above.
Avoid `air-00` and `air-20`. The examples use `air-09`; replace it with your chosen
name. A fresh clone keeps the source Pi's hostname, so rename it before joining
the mesh.

```bash
sudo hostnamectl set-hostname air-09
sudo nano /etc/hosts
```

In `/etc/hosts`, replace the old name on the `127.0.1.1` line so it reads:

```text
127.0.1.1 air-09
```

Keep the other lines. Save with **Ctrl+O**, press **Enter**, and exit with **Ctrl+X**.
The mesh IP follows the name: `air-09` gets `10.10.0.9`.

Label the cloned SD card with its new hostname (for example, `air-09`) so you can
identify it later.

## 4. Reset the machine ID and reboot

Do this once for each new clone, then reboot immediately:

```bash
sudo truncate -s 0 /etc/machine-id
sudo reboot
```

Ubuntu generates a new machine ID at boot. This helps give each Pi a different
software-generated mesh MAC address. This procedure keeps the image's shared SSH
host keys.

## 5. Check the mesh

Reconnect to the Pi using [Ethernet and SSH](ethernet.md).

Plug in the USB Wi-Fi adapter. On the Pi, run:

```bash
hostnamectl --static
ip -br address show bat0
cat /sys/class/net/bat0/address
```

For `air-09`, expect the hostname `air-09` and address `10.10.0.9/24` on `bat0`.
Each Pi should have a different `bat0` MAC address.

Power on another configured Pi with its adapter connected. Ping its mesh IP from
the new Pi; for example, if the other Pi is `air-06`:

```bash
ping -I bat0 -c 4 10.10.0.6
```

You should receive replies. Also check that the mesh reconnects after an adapter
unplug/replug and after a reboot with the adapter attached.

## If the card's full capacity is not available

Ubuntu may expand the filesystem automatically. Check on the Pi:

```bash
df -h /
```

A 64 GB card should show about 58 GB under **Size**. If it still shows about 19 GB
on a larger card, find the root partition:

```bash
findmnt -no SOURCE,FSTYPE /
```

If it reports `/dev/mmcblk0p2 ext4`, run:

```bash
sudo growpart /dev/mmcblk0 2
sudo resize2fs /dev/mmcblk0p2
```

If it reports `/dev/sda2 ext4`, use these instead:

```bash
sudo growpart /dev/sda 2
sudo resize2fs /dev/sda2
```

Run only the pair matching your Pi's root partition. `NOCHANGE` or
`Nothing to do!` means that part is already expanded. Check `df -h /` again.
