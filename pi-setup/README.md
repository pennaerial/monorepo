# Set up a new Pi

The cloned SD card must already contain the required software and `~/monorepo`.

1. **Flash the club’s base image and insert the card.**

   The base image should use hostname `air-unassigned`, so a new Pi does not automatically join the mesh using another Pi’s address. Boot using a keyboard and monitor.

2. **Choose an unused hostname.**

   Use `air-01` through `air-09`. Label the Pi and its SD card with that number.

   TODO: Maintain a club-wide list of assigned hostnames.

3. **Set the hostname.**

   Replace `air-07` below with your chosen name:

   ```bash
   sudo hostnamectl set-hostname air-07
   sudo nano /etc/hosts
   ```

   Replace the old hostname in `/etc/hosts` with the same name. Keep the `localhost` entries unchanged.

4. **Install the mesh setup and reboot.**

   ```bash
   cd ~/monorepo
   sudo bash pi-setup/install.sh
   sudo reboot
   ```

5. **Verify after reboot.**

   ```bash
   hostnamectl --static
   systemctl status pi-mesh.service --no-pager
   ip -br address show bat0
   ```

   Expect `active (exited)` and an address matching the hostname:
   `air-07` → `10.67.0.7/24`.