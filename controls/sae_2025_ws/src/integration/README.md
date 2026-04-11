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
- backend Python deps in conda mode (`fastapi[standard]`, `python-multipart`, `httpx`)
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

For first-time Pi setup, use (where `x` is a number):

```bash
cd /home/penn/monorepo/controls/sae_2025_ws
scripts/hardware/provision-pi.sh \
  --hostname air-0x \
  --authorized-key-url https://raw.githubusercontent.com/pennaerial/monorepo/main/controls/sae_2025_ws/ops/keys/pennair_pi_ed25519.pub
```

That script configures:
- hostname / mDNS identity
- `openssh-server` and `avahi-daemon`
- the deploy user (default: `penn`)
- SSH authorized keys
- optional passwordless sudo for the deploy user
- the existing hardware bootstrap path

Recommended naming:
- hostname: `air-0x.local`
- inventory `target_id`: `air-0x`

Keep physical Pi identity in the hostname. Use inventory `vehicle_name` for the current assigned controllable.

Notes:
- run the provisioning script on the Pi itself
- ROS 2 Humble still needs to exist on the Pi at `/opt/ros/humble`
- the script now attempts NTP time sync before any `apt` work and falls back to an HTTP `Date` header if NTP is unreachable, but broken third-party apt repos on the Pi can still block provisioning
- `--no-bootstrap` skips the deploy-root/systemd install if you only want hostname/user/SSH setup
- if you do not want a hosted public key, use `--authorized-key-file` or `--authorized-key` instead
