# Pi SSH Setup

_Last updated: 2026-11-07_

Connect a laptop to any Raspberry Pi over a direct ethernet cable and SSH in, so you can work from the laptop instead of the Pi's monitor.

Every PennAiR Pi uses the login username **`penn`**. Pis are named `air-##` (e.g. `air-01`, `air-02`). Replace `air-##` below with the one you're connecting to (check with `hostname` on the Pi).

The addresses below are the same for every Pi. That's fine because only one Pi is plugged into your laptop at a time.

## Addresses

| Device | Interface | Address | Notes |
|---|---|---|---|
| Pi (`air-##`) | `eth0` | `192.168.50.2/24` | user: `penn` |
| Laptop | built-in ethernet port or USB adapter | `192.168.50.1/24` | gateway: blank |

Nothing on the cable hands out addresses automatically, so both ends are set by hand.

## Part 1 — On the Pi (Pi's own terminal: monitor + keyboard)

1. **Make sure SSH is running.** The status should say `active (running)`.

   ```bash
   sudo systemctl enable --now ssh
   sudo systemctl status ssh
   ```

2. **Set the ethernet address permanently** (once per Pi; survives reboot).

   The connection is usually called `Wired connection 1`. If that name doesn't work, find the one on `eth0` with `nmcli con show` and use it instead.

   ```bash
   sudo nmcli con modify "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.2/24
   sudo nmcli con up "Wired connection 1"
   ```

3. **Quick fix if step 2 isn't done yet** (lost on reboot).

   ```bash
   sudo nmcli device disconnect eth0
   sudo ip addr flush dev eth0
   sudo ip addr add 192.168.50.2/24 dev eth0
   sudo ip link set eth0 up
   ```

4. **Check before connecting.** Look for `LOWER_UP` and exactly one address: `inet 192.168.50.2/24`.

   ```bash
   ip -4 addr show eth0
   ```

   If a wrong address is also listed, remove it:

   ```bash
   sudo ip addr del <wrong-address>/24 dev eth0
   ```

## Part 2 — On the laptop

Plug the ethernet cable from the Pi into the laptop, either straight into its ethernet port or through a USB ethernet adapter. Then give the laptop's **wired** adapter `192.168.50.1/24`.

### Windows (PowerShell → right-click → "Run as administrator")

1. Find the wired adapter's name. Its Status must be `Up`.

   ```powershell
   Get-NetAdapter
   ```

2. Set the address (replace `"Ethernet"` with the name from step 1):

   ```powershell
   New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.50.1 -PrefixLength 24
   ```

3. Check it:

   ```powershell
   ipconfig
   ```

   You should see `IPv4 Address: 192.168.50.1` under that adapter. If you see `169.254.x.x` instead, the setting didn't apply — redo step 2.

**GUI alternative:** Settings → Network & internet → Ethernet → IP assignment → Edit → Manual → IPv4 on. IP `192.168.50.1`, mask `255.255.255.0`, gateway blank.

### Linux

Wired adapters have different names depending on how you're plugged in:

- **Built-in ethernet port** (no adapter): a short name like `eno2`, `enp3s0` or `eth0`
- **USB ethernet adapter:** a long name starting with `enx`, like `enx00e04c3d8b48`

Find yours (it's the one that shows `UP` once the cable is plugged in):

```bash
ip -br link
```

Then (replace `<adapter>` with that name):

```bash
sudo ip addr flush dev <adapter>
sudo ip addr add 192.168.50.1/24 dev <adapter>
sudo ip link set <adapter> up
```

### Mac

System Settings → Network → ethernet/USB adapter → Details → TCP/IP → Configure IPv4: **Manually**. IP `192.168.50.1`, subnet mask `255.255.255.0`, router blank.

## Part 3 — Connect (run on the laptop, not the Pi)

```bash
ping 192.168.50.2
ssh penn@192.168.50.2
```

First time: type `yes` at the fingerprint prompt, then the `penn` password.

If you've connected to a different Pi at `192.168.50.2` before, SSH will warn that the host key changed. Clear the old entry, then connect again:

```bash
ssh-keygen -R 192.168.50.2
```

**Optional — short name:** add this to `~/.ssh/config` on the laptop (Windows: `C:\Users\<you>\.ssh\config`), then just run `ssh air-##`. Add one block per Pi you use:

```
Host air-##
    HostName 192.168.50.2
    User penn
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `nmcli device status` shows eth0 "unmanaged" | | Part 1 step 2, or `sudo nmcli device set eth0 managed yes` |
| eth0 stuck on "connecting (getting IP configuration)" | Waiting for an automatic address that nothing provides | Part 1 step 2 (manual address) |
| `ip link show eth0` says `NO-CARRIER` or `DOWN` | No live device on the other end of the cable | Reseat/swap cable or adapter; the Pi's port lights should blink |
| `ping` times out | Laptop address missing / on the wrong adapter, or a stray address on the Pi | Laptop: `ipconfig`; Pi: `ip -4 addr show eth0`. Each side should show exactly one `192.168.50.x` address |
| `Permission denied (publickey)` | Password login disabled on the Pi | `sudo nano /etc/ssh/sshd_config.d/50-cloud-init.conf`, set `PasswordAuthentication yes`, then `sudo systemctl restart ssh` |
| Address gone after a reboot | Set with `ip addr add`, which isn't saved | Part 1 step 2 |
