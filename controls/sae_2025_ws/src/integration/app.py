# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "fastapi[standard]",
#     "python-multipart",
# ]
# ///

import asyncio
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

from fastapi import FastAPI, Form, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

app = FastAPI(title="SAE Deploy Dashboard")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Configuration ────────────────────────────────────────────
# Override with environment variables, or change at runtime via /api/config

config = {
    "pi_user": os.environ.get("PI_USER", "penn"),
    "pi_host": os.environ.get("PI_HOST", "penn-desktop.local"),
    "remote_dir": os.environ.get("REMOTE_DIR", "/home/penn/monorepo/controls/sae_2025_ws"),
    "ssh_key": os.environ.get("SSH_KEY", ""),
    "ssh_pass": os.environ.get("SSH_PASS", "123"),
    "github_repo": os.environ.get("GITHUB_REPO", ""),
    "github_token": os.environ.get("GITHUB_TOKEN", ""),
    "hotspot_name": os.environ.get("HOTSPOT_CON_NAME", "penn-desktop"),
}

# Auto-detect GitHub repo from git remote if not set
if not config['github_repo']:
    try:
        result = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            capture_output=True, text=True, cwd=Path(__file__).parent,
        )
        match = re.search(r"github\.com[:/]([^/]+/[^/.]+)", result.stdout)
        if match:
            config['github_repo'] = match.group(1).strip()
    except Exception:
        pass


# ── Config API ───────────────────────────────────────────────


@app.get("/api/config")
async def get_config():
    """Return current configuration (password masked)."""
    safe = {**config, "ssh_pass": "••••" if config["ssh_pass"] else ""}
    return {"success": True, "config": safe}


@app.post("/api/config")
async def set_config(
    pi_user: str = Form(None),
    pi_host: str = Form(None),
    remote_dir: str = Form(None),
    ssh_key: str = Form(None),
    ssh_pass: str = Form(None),
    github_repo: str = Form(None),
    hotspot_name: str = Form(None),
):
    """Update configuration at runtime."""
    updates = {}
    if pi_user is not None:
        config["pi_user"] = pi_user; updates["pi_user"] = pi_user
    if pi_host is not None:
        config["pi_host"] = pi_host; updates["pi_host"] = pi_host
    if remote_dir is not None:
        config['remote_dir'] = remote_dir; updates["remote_dir"] = remote_dir
    if ssh_key is not None:
        config["ssh_key"] = ssh_key; updates["ssh_key"] = ssh_key
    if ssh_pass is not None and ssh_pass != "••••":
        config["ssh_pass"] = ssh_pass; updates["ssh_pass"] = "(set)"
    if github_repo is not None:
        config['github_repo'] = github_repo; updates["github_repo"] = github_repo
    if hotspot_name is not None:
        config["hotspot_name"] = hotspot_name; updates["hotspot_name"] = hotspot_name

    return {"success": True, "output": f"Updated: {', '.join(updates.keys())}"}


# ── SSH Helpers ──────────────────────────────────────────────


def _ssh_target():
    return f"{config['pi_user']}@{config['pi_host']}"


def _ssh_opts():
    opts = ["-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=3"]
    if config["ssh_key"]:
        opts += ["-i", config["ssh_key"]]
    return opts


def _run_ssh_sync(command: str, timeout: int = 15) -> subprocess.CompletedProcess:
    """Run a command on the Pi via SSH (blocking)."""
    cmd = ["ssh"] + _ssh_opts() + [_ssh_target(), command]
    if config["ssh_pass"]:
        cmd = ["sshpass", "-p", config["ssh_pass"]] + cmd
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def _run_scp_sync(local_path: str, remote_path: str, timeout: int = 300) -> subprocess.CompletedProcess:
    """Copy a file to the Pi via SCP (blocking)."""
    cmd = ["scp"] + _ssh_opts() + [local_path, f"{_ssh_target()}:{remote_path}"]
    if config["ssh_pass"]:
        cmd = ["sshpass", "-p", config["ssh_pass"]] + cmd
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


async def _run_ssh(command: str, timeout: int = 15) -> subprocess.CompletedProcess:
    """Run a command on the Pi via SSH (non-blocking)."""
    return await asyncio.to_thread(_run_ssh_sync, command, timeout)


async def _run_scp(local_path: str, remote_path: str, timeout: int = 300) -> subprocess.CompletedProcess:
    """Copy a file to the Pi via SCP (non-blocking)."""
    return await asyncio.to_thread(_run_scp_sync, local_path, remote_path, timeout)


# ── Connection ───────────────────────────────────────────────


@app.get("/api/connection/status")
async def connection_status():
    """Check if the Pi is reachable via SSH."""
    try:
        r = await _run_ssh("echo ok", timeout=5)
        if r.returncode == 0:
            info = await _run_ssh("hostname && hostname -I", timeout=5)
            return {
                "success": True,
                "connected": True,
                "target": _ssh_target(),
                "info": info.stdout.strip() if info.returncode == 0 else "",
            }
        return {"success": True, "connected": False, "error": r.stderr.strip()}
    except subprocess.TimeoutExpired:
        return {"success": True, "connected": False, "error": "Connection timed out"}
    except Exception as e:
        return {"success": True, "connected": False, "error": str(e)}


@app.get("/api/connection/ssh-command")
async def ssh_command():
    """Return the SSH command for the user to copy."""
    cmd = "ssh"
    if config["ssh_key"]:
        cmd += f" -i {config['ssh_key']}"
    cmd += f" {_ssh_target()}"
    return {"success": True, "command": cmd}


# ── WiFi Management (via SSH to Pi) ─────────────────────────


@app.get("/api/wifi/status")
async def wifi_status():
    """Get Pi's current WiFi status via SSH."""
    try:
        r = await _run_ssh("nmcli -f NAME,TYPE,DEVICE,STATE con show --active | tail -n +2", timeout=15)
        if r.returncode != 0:
            return {"success": False, "error": r.stderr.strip()}

        connections = []
        for line in r.stdout.strip().split("\n"):
            if not line.strip():
                continue
            parts = re.split(r'\s{2,}', line.strip())
            if len(parts) >= 4:
                connections.append({
                    "name": parts[0].strip(),
                    "type": parts[1].strip(),
                    "device": parts[2].strip(),
                    "state": parts[3].strip(),
                })

        is_hotspot = any(c["name"] == config["hotspot_name"] for c in connections)
        wifi_con = next((c for c in connections if c["type"] == "802-11-wireless"), None)

        return {
            "success": True,
            "is_hotspot": is_hotspot,
            "current_wifi": wifi_con["name"] if wifi_con else None,
            "connections": connections,
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "SSH timed out"}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/wifi/scan")
async def wifi_scan():
    """Scan for WiFi networks on the Pi via SSH."""
    try:
        # Use tab separator to avoid issues with colons in SSID/security fields
        r = await _run_ssh(
            r"nmcli -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes | tail -n +2",
            timeout=20,
        )
        if r.returncode != 0:
            return {"success": False, "error": r.stderr.strip(), "networks": []}

        networks = []
        for line in r.stdout.strip().split("\n"):
            if not line.strip():
                continue
            # nmcli tabular output is whitespace-aligned; parse by splitting on 2+ spaces
            parts = re.split(r'\s{2,}', line.strip())
            if len(parts) >= 3:
                ssid = parts[0].strip()
                if not ssid:
                    continue
                try:
                    signal = int(parts[1].strip())
                except ValueError:
                    signal = 0
                security = parts[2].strip() if len(parts) > 2 else ""
                networks.append({
                    "ssid": ssid,
                    "signal": signal,
                    "security": security,
                })

        # Deduplicate by SSID, keep strongest signal
        seen: dict = {}
        for n in networks:
            if n["ssid"] not in seen or n["signal"] > seen[n["ssid"]]["signal"]:
                seen[n["ssid"]] = n

        return {
            "success": True,
            "networks": sorted(seen.values(), key=lambda x: -x["signal"]),
        }
    except Exception as e:
        return {"success": False, "error": str(e), "networks": []}


@app.post("/api/wifi/connect")
async def wifi_connect(ssid: str = Form(...), password: str = Form("")):
    """Connect the Pi to a WiFi network (disables hotspot)."""
    try:
        # Bring down hotspot
        await _run_ssh(f"nmcli con down '{config['hotspot_name']}' 2>/dev/null; sleep 2", timeout=15)

        # Connect
        pwd_arg = f' password "{password}"' if password else ""
        r = await _run_ssh(f'nmcli dev wifi connect "{ssid}"{pwd_arg}', timeout=30)

        if r.returncode != 0:
            # Restore hotspot on failure
            await _run_ssh(f"nmcli con up '{config['hotspot_name']}'", timeout=15)
            return {"success": False, "error": f"Failed to connect: {r.stderr.strip()}"}

        return {"success": True, "output": f"Pi connected to {ssid}"}
    except Exception as e:
        await _run_ssh(f"nmcli con up '{config['hotspot_name']}'", timeout=15)
        return {"success": False, "error": str(e)}


@app.post("/api/wifi/hotspot")
async def wifi_hotspot():
    """Switch Pi back to hotspot mode."""
    try:
        await _run_ssh("nmcli dev disconnect wlan0 2>/dev/null; sleep 1", timeout=15)
        r = await _run_ssh(f"nmcli con up '{config['hotspot_name']}'", timeout=15)
        if r.returncode != 0:
            return {"success": False, "error": r.stderr.strip()}
        return {"success": True, "output": "Hotspot activated"}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/wifi/switch-local")
async def switch_local_wifi(ssid: str = Form(...), password: str = Form("")):
    """Switch THIS Mac's WiFi to match (macOS only)."""
    try:
        cmd = ["networksetup", "-setairportnetwork", "en0", ssid]
        if password:
            cmd.append(password)
        r = await asyncio.to_thread(subprocess.run, cmd, capture_output=True, text=True, timeout=15)
        if r.returncode != 0:
            return {"success": False, "error": r.stderr.strip() or r.stdout.strip()}
        return {"success": True, "output": f"Mac connected to {ssid}"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _github_headers() -> dict:
    """Build headers for GitHub API requests."""
    headers = {"Accept": "application/vnd.github+json"}
    if config.get("github_token"):
        headers["Authorization"] = f"Bearer {config['github_token']}"
    return headers


# ── Build Management (via SCP/SSH) ──────────────────────────


@app.get("/api/builds/current")
async def current_build():
    """Get info about the currently deployed build on the Pi."""
    try:
        r = await _run_ssh(f"cat {config['remote_dir']}/install/BUILD_INFO.txt 2>/dev/null", timeout=10)
        if r.returncode == 0 and r.stdout.strip():
            return {"success": True, "installed": True, "info": r.stdout.strip()}

        r = await _run_ssh(f"test -d {config['remote_dir']}/install && echo yes || echo no", timeout=10)
        if r.stdout.strip() == "yes":
            return {"success": True, "installed": True, "info": "Build installed (no BUILD_INFO.txt)"}

        return {"success": True, "installed": False, "info": "No build deployed"}
    except Exception as e:
        return {"success": True, "installed": False, "info": str(e)}


@app.post("/api/builds/upload")
async def upload_build(file: UploadFile = File(...)):
    """Upload a build artifact to the Pi via SCP and extract it."""
    try:
        # Save to temp file locally
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp.write(await file.read())
            tmp_path = tmp.name

        # Ensure remote dir exists
        await _run_ssh(f"mkdir -p {config['remote_dir']}", timeout=10)

        # SCP to Pi
        remote_file = f"{config['remote_dir']}/{file.filename}"
        r = await _run_scp(tmp_path, remote_file, timeout=300)
        os.unlink(tmp_path)

        if r.returncode != 0:
            return {"success": False, "error": f"SCP failed: {r.stderr.strip()}"}

        # Backup + extract on Pi
        extract_cmd = f"""
            cd {config['remote_dir']}
            if [ -d install ]; then
                rm -rf install.bak src.bak
                mv install install.bak 2>/dev/null || true
                mv src src.bak 2>/dev/null || true
            fi
            tar -xzf {file.filename}
            rm -f {file.filename}
            chmod -R +x install/
        """
        r = await _run_ssh(extract_cmd, timeout=120)
        if r.returncode != 0:
            return {"success": False, "error": f"Extract failed: {r.stderr.strip()}"}

        return {"success": True, "output": f"Deployed {file.filename} to Pi"}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/builds/list")
async def list_builds():
    """List available builds from GitHub Releases."""
    if not config['github_repo']:
        return {"success": False, "error": "GITHUB_REPO not configured", "builds": []}
    try:
        import httpx
        async with httpx.AsyncClient(timeout=15) as client:
            resp = await client.get(
                f"https://api.github.com/repos/{config['github_repo']}/releases",
                headers=_github_headers(),
            )
            resp.raise_for_status()
            releases = resp.json()

        if isinstance(releases, dict) and "message" in releases:
            return {"success": False, "error": releases["message"], "builds": []}

        builds = []
        for rel in releases:
            if rel.get("tag_name", "").startswith("build-"):
                assets = rel.get("assets", [])
                builds.append({
                    "tag": rel["tag_name"],
                    "sha": rel["tag_name"].replace("build-", ""),
                    "name": rel.get("name", rel["tag_name"]),
                    "date": rel.get("published_at", "")[:10],
                    "download_url": assets[0]["browser_download_url"] if assets else None,
                    "size_mb": round(assets[0]["size"] / 1_000_000, 1) if assets else None,
                })
        return {"success": True, "builds": builds[:20]}
    except Exception as e:
        return {"success": False, "error": str(e), "builds": []}


@app.post("/api/builds/download")
async def download_build(tag: str = Form(...)):
    """Download a build from GitHub on your Mac, then SCP it to the Pi."""
    if not config['github_repo']:
        return {"success": False, "error": "GITHUB_REPO not configured"}
    try:
        import httpx

        # Get release info
        async with httpx.AsyncClient(timeout=15) as client:
            resp = await client.get(
                f"https://api.github.com/repos/{config['github_repo']}/releases/tags/{tag}",
                headers=_github_headers(),
            )
            resp.raise_for_status()
            release = resp.json()

        assets = release.get("assets", [])
        if not assets:
            return {"success": False, "error": f"No assets found for {tag}"}

        download_url = assets[0]["browser_download_url"]
        filename = assets[0]["name"]

        # Download to temp file on Mac
        tmp_path = tempfile.mktemp(suffix=".tar.gz")
        async with httpx.AsyncClient(timeout=300, follow_redirects=True) as client:
            async with client.stream("GET", download_url, headers=_github_headers()) as resp:
                resp.raise_for_status()
                with open(tmp_path, "wb") as f:
                    async for chunk in resp.aiter_bytes():
                        f.write(chunk)

        # SCP to Pi
        await _run_ssh(f"mkdir -p {config['remote_dir']}", timeout=10)
        remote_file = f"{config['remote_dir']}/{filename}"
        r = await _run_scp(tmp_path, remote_file, timeout=300)
        os.unlink(tmp_path)

        if r.returncode != 0:
            return {"success": False, "error": f"SCP failed: {r.stderr.strip()}"}

        # Backup + extract on Pi
        extract_cmd = f"""
            cd {config['remote_dir']}
            if [ -d install ]; then
                rm -rf install.bak src.bak
                mv install install.bak 2>/dev/null || true
                mv src src.bak 2>/dev/null || true
            fi
            tar -xzf {filename}
            rm -f {filename}
            chmod -R +x install/
        """
        r = await _run_ssh(extract_cmd, timeout=120)
        if r.returncode != 0:
            return {"success": False, "error": f"Extract failed: {r.stderr.strip()}"}

        return {"success": True, "output": f"Downloaded {tag} and deployed to Pi"}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/builds/rollback")
async def rollback_build():
    """Rollback to the previous build on the Pi."""
    try:
        rollback_cmd = f"""
            cd {config['remote_dir']}
            if [ ! -d install.bak ]; then
                echo "NO_BACKUP"
                exit 1
            fi
            if [ -d install ]; then
                mv install install.tmp
                mv install.bak install
                mv install.tmp install.bak
            else
                mv install.bak install
            fi
        """
        r = await _run_ssh(rollback_cmd, timeout=30)
        if r.returncode != 0:
            if "NO_BACKUP" in r.stdout:
                return {"success": False, "error": "No backup build to rollback to"}
            return {"success": False, "error": r.stderr.strip()}

        return {"success": True, "output": "Rolled back to previous build"}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ── Failsafe ────────────────────────────────────────────────


@app.post("/api/failsafe")
async def trigger_failsafe():
    """Trigger failsafe on the Pi via ros2 topic pub."""
    try:
        cmd = (
            f"cd {config['remote_dir']} && "
            f"source /opt/ros/humble/setup.bash && "
            f"source install/setup.bash && "
            f'ros2 topic pub --once /failsafe_trigger std_msgs/msg/Bool "{{data: true}}"'
        )
        r = await _run_ssh(f"bash -c '{cmd}'", timeout=15)
        if r.returncode != 0:
            return {"success": False, "error": r.stderr.strip() or "Failsafe command failed"}
        return {"success": True, "output": "Failsafe triggered"}
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Failsafe command timed out"}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ── Static Files (production) ────────────────────────────────

frontend_dist = Path(__file__).parent / "frontend" / "dist"
if frontend_dist.exists():
    app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")



if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
