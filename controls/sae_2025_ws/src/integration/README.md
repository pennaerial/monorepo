# PennAiR Auton Deploy

React + FastAPI dashboard for Pi connectivity, WiFi control, and build deploy/rollback.

## Structure

```
integration/
├── app.py              # FastAPI entrypoint
├── backend/            # Backend modules
│   ├── app_factory.py
│   ├── config.py
│   ├── context.py
│   ├── models.py
│   ├── ssh.py
│   ├── state.py
│   ├── routers/
│   └── services/
├── frontend/           # React frontend
│   ├── src/
│   │   ├── App.jsx     # Main UI composition
│   │   ├── hooks/      # API/WebSocket hooks
│   │   ├── services/   # API client helpers
│   │   ├── App.css     # Styles
│   │   ├── main.jsx    # Entry point
│   │   └── index.css   # Global styles
│   ├── index.html
│   ├── vite.config.js
│   └── package.json
```

## Setup & Run

### 1. Install Dependencies (first time)

```bash
cd /Users/ethanyu/VSCodeProjects/monorepo/controls/sae_2025_ws/src/integration/frontend
npm install
```

`./launch.sh` also auto-checks dependencies at startup:
- frontend deps (`npm install` when needed)
- backend Python deps in conda mode (`fastapi[standard]`, `python-multipart`, `httpx`, `pydantic>=2,<3`)
- `sshpass` via package manager when available (`brew`, `apt`, or `dnf`)

### 2. Launch Backend + Frontend Together

```bash
cd /Users/ethanyu/VSCodeProjects/monorepo/controls/sae_2025_ws/src/integration
./launch.sh
```

This starts:
- Backend API on `http://localhost:8080`
- Frontend dev server on `http://localhost:3000`

Stop both with `Ctrl+C` in the terminal running `launch.sh`.

### Conda Workflow (optional)

If you use conda, activate your env first:

```bash
conda activate <your-env>
cd /Users/ethanyu/VSCodeProjects/monorepo/controls/sae_2025_ws/src/integration
./launch.sh
```

When a conda env is active, `launch.sh` uses `python app.py` for the backend and installs missing backend Python deps automatically.
If no conda env is active, it uses `uv run app.py`.

## Building for Production

```bash
cd frontend
npm run build
```

This creates a `dist/` folder with optimized static files you can serve with any web server.

## Provisioning a Pi

For first-time Pi setup, use one of these patterns (where `x` is a number):

```bash
# Default client-mode Pi. Uses the default travel router profile:
#   SSID: pennair_5G
#   PSK:  pennair123!
# Default fallback AP PSK: pennair123!
cd /home/penn/monorepo/controls/sae_2025_ws
scripts/hardware/provision-pi.sh \
  --hostname air-0x \
  --authorized-key-url https://raw.githubusercontent.com/pennaerial/monorepo/main/controls/sae_2025_ws/ops/keys/pennair_pi_ed25519.pub
```

```bash
# AP-capable Pi. If pennair_5G is unavailable, this Pi will host
# a fallback AP named pennair-ap-<hostname>.
cd /home/penn/monorepo/controls/sae_2025_ws
scripts/hardware/provision-pi.sh \
  --hostname air-0x \
  --authorized-key-url https://raw.githubusercontent.com/pennaerial/monorepo/main/controls/sae_2025_ws/ops/keys/pennair_pi_ed25519.pub \
  --network-role ap
```

```bash
# Optional: if you want bootstrap-time client fallback restricted to specific
# AP Pis before any fleet deploy override exists, pin the allowed AP hostnames here.
cd /home/penn/monorepo/controls/sae_2025_ws
scripts/hardware/provision-pi.sh \
  --hostname payload-01 \
  --authorized-key-url https://raw.githubusercontent.com/pennaerial/monorepo/main/controls/sae_2025_ws/ops/keys/pennair_pi_ed25519.pub \
  --network-role client \
  --allowed-ap-hosts air-01,air-02
```

That script configures:
- hostname / mDNS identity
- `openssh-server` and `avahi-daemon`
- explicit Avahi `_ssh._tcp` service advertisement for dashboard discovery
- the deploy user (default: `penn`)
- SSH authorized keys
- optional passwordless sudo for the deploy user
- the existing hardware bootstrap path, including default NetworkManager failover setup

Recommended naming:
- hostname: `air-0x.local`
- inventory `target_id`: `air-0x`

Keep physical Pi identity in the hostname. Use inventory `vehicle_name` for the current assigned controllable.

Notes:
- run the provisioning script on the Pi itself
- ROS 2 Humble still needs to exist on the Pi at `/opt/ros/humble`
- the default travel-router profile is `pennair_5G` / `pennair123!`, and the default fallback AP PSK is also `pennair123!`; override them with `--travel-router-ssid`, `--travel-router-psk`, and `--fallback-psk` if needed
- bootstrap-time fallback targeting is still host-based via `--allowed-ap-hosts`, but that list is optional; if omitted, client-mode Pis discover any visible PennAiR fallback AP matching the configured prefix
- the integration fleet deploy UI is vehicle-based and resolves vehicles to hostnames at deploy time
- the script now attempts NTP time sync before any `apt` work and falls back to an HTTP `Date` header if NTP is unreachable, but broken third-party apt repos on the Pi can still block provisioning
- if an existing Pi was provisioned before this change, rerun `provision-pi.sh` once so `/etc/avahi/services/ssh.service` is installed
- `--no-bootstrap` skips the deploy-root/systemd install if you only want hostname/user/SSH setup
- if you do not want a hosted public key, use `--authorized-key-file` or `--authorized-key` instead
