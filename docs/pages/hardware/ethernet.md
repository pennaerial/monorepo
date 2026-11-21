# Ethernet and SSH

Connect your laptop directly to a Pi over Ethernet and SSH. Connect one Pi at a
time.

## 1. Connect the cable

Connect the Pi to your laptop's Ethernet port or a USB Ethernet adapter, then
power on the Pi.

## 2. Set up Ethernet and SSH on the Pi

> **Using the club image? Skip this section and continue to step 3.** Ethernet
> and SSH should already be configured.

These steps are for Ubuntu using [NetworkManager](https://networkmanager.dev/docs/api/latest/nm-settings-nmcli.html).
Run them on the Pi with a monitor and keyboard.

Find the connection name for the Pi's Ethernet port (`eth0`):

```bash
nmcli -f NAME,DEVICE connection show
```

Replace `Wired connection 1` below with that name. Set the Pi's address and save
it for future boots:

```bash
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual ipv4.addresses 192.168.50.2/24 \
  ipv4.gateway "" ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection up "Wired connection 1"
```

Enable SSH:

```bash
sudo systemctl enable --now ssh
```

If the `ssh` service is missing, install it with `sudo apt install openssh-server`
while the Pi has internet access, then retry the command above.

Check the address:

```bash
ip -4 addr show eth0
```

You should see `192.168.50.2/24`.

## 3. Set the laptop's address

### Linux

Find your wired interface:

```bash
ip -br link
```

A built-in Ethernet port usually has a name like `eno2` or `enp3s0`; a USB
Ethernet adapter often starts with `enx`. Use the interface connected to the Pi.

Replace `<adapter>` with that name:

```bash
sudo ip -4 addr flush dev <adapter>
sudo ip addr add 192.168.50.1/24 dev <adapter>
sudo ip link set <adapter> up
```

These settings are temporary. Repeat them after a reboot or if the connection
resets.

### macOS

Open **System Settings → Network → your Ethernet adapter → Details → TCP/IP**.
Set **Configure IPv4** to **Manually**, then enter:

- IP address: `192.168.50.1`
- Subnet mask: `255.255.255.0`
- Router: leave blank

Apply the settings.

## 4. Connect to the Pi

Run on your laptop:

```bash
ssh penn@192.168.50.2
```

If your Pi uses a different username, replace `penn` with it. Accept the
fingerprint prompt if shown, then enter your Pi account's password. For the club
image, ask the software team if you need the password.
