# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "fastapi[standard]",
#     "python-multipart",
#     "httpx",
# ]
# ///

import asyncio
import json
import os
import re
import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path

from fastapi import FastAPI, Form, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

app = FastAPI(title="PennAiR Auton Deploy")

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
    "remote_dir": os.environ.get(
        "REMOTE_DIR", "/home/penn/monorepo/controls/sae_2025_ws"
    ),
    "ssh_key": os.environ.get("SSH_KEY", ""),
    "ssh_pass": os.environ.get("SSH_PASS", ""),
    "github_repo": os.environ.get("GITHUB_REPO", ""),
    "github_token": os.environ.get("GITHUB_TOKEN", ""),
    "hotspot_name": os.environ.get("HOTSPOT_CON_NAME", "penn-desktop"),
}

# Auto-detect GitHub repo from git remote if not set
if not config["github_repo"]:
    try:
        result = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent,
        )
        match = re.search(r"github\.com[:/]([^/]+/[^/.]+)", result.stdout)
        if match:
            config["github_repo"] = match.group(1).strip()
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
        config["pi_user"] = pi_user
        updates["pi_user"] = pi_user
    if pi_host is not None:
        config["pi_host"] = pi_host
        updates["pi_host"] = pi_host
    if remote_dir is not None:
        config["remote_dir"] = remote_dir
        updates["remote_dir"] = remote_dir
    if ssh_key is not None:
        config["ssh_key"] = ssh_key
        updates["ssh_key"] = ssh_key
    if ssh_pass is not None and ssh_pass != "••••":
        config["ssh_pass"] = ssh_pass
        updates["ssh_pass"] = "(set)"
    if github_repo is not None:
        config["github_repo"] = github_repo
        updates["github_repo"] = github_repo
    if hotspot_name is not None:
        config["hotspot_name"] = hotspot_name
        updates["hotspot_name"] = hotspot_name

    return {"success": True, "output": f"Updated: {', '.join(updates.keys())}"}


# ── SSH Helpers ──────────────────────────────────────────────


def _ssh_target():
    return f"{config['pi_user']}@{config['pi_host']}"


def _ssh_opts():
    opts = ["-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=3"]
    if config["ssh_key"]:
        opts += ["-i", config["ssh_key"]]
    # If no password is configured, force non-interactive auth so failures
    # return quickly (instead of waiting for password prompt timeouts).
    if not config["ssh_pass"]:
        opts += ["-o", "BatchMode=yes", "-o", "NumberOfPasswordPrompts=0"]
    return opts


def _run_ssh_sync(command: str, timeout: int = 15) -> subprocess.CompletedProcess:
    """Run a command on the Pi via SSH (blocking)."""
    cmd = ["ssh"] + _ssh_opts() + [_ssh_target(), command]
    if config["ssh_pass"]:
        if not shutil.which("sshpass"):
            raise RuntimeError(
                "SSH password auth is configured, but `sshpass` is not installed. "
                "Install sshpass or clear SSH password in Settings and use SSH key auth."
            )
        cmd = ["sshpass", "-p", config["ssh_pass"]] + cmd
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def _run_scp_sync(
    local_path: str, remote_path: str, timeout: int = 300
) -> subprocess.CompletedProcess:
    """Copy a file to the Pi via SCP (blocking)."""
    cmd = ["scp"] + _ssh_opts() + [local_path, f"{_ssh_target()}:{remote_path}"]
    if config["ssh_pass"]:
        if not shutil.which("sshpass"):
            raise RuntimeError(
                "SSH password auth is configured, but `sshpass` is not installed. "
                "Install sshpass or clear SSH password in Settings and use SSH key auth."
            )
        cmd = ["sshpass", "-p", config["ssh_pass"]] + cmd
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


async def _run_ssh(command: str, timeout: int = 15) -> subprocess.CompletedProcess:
    """Run a command on the Pi via SSH (non-blocking)."""
    return await asyncio.to_thread(_run_ssh_sync, command, timeout)


async def _run_scp(
    local_path: str, remote_path: str, timeout: int = 300
) -> subprocess.CompletedProcess:
    """Copy a file to the Pi via SCP (non-blocking)."""
    return await asyncio.to_thread(_run_scp_sync, local_path, remote_path, timeout)


def _friendly_ssh_error(stderr: str) -> str:
    """Convert low-level SSH/SCP errors into operator-facing guidance."""
    raw = (stderr or "").strip()
    lower = raw.lower()
    target = _ssh_target()

    if any(
        s in lower
        for s in (
            "could not resolve hostname",
            "name or service not known",
            "no address associated with hostname",
            "nodename nor servname provided",
            "temporary failure in name resolution",
        )
    ):
        return (
            f"Cannot find {target}. Connect to the Pi WiFi and make sure "
            "Pi host/user in Settings are correct."
        )

    if any(
        s in lower
        for s in (
            "no route to host",
            "connection refused",
            "network is unreachable",
            "operation timed out",
            "connection timed out",
        )
    ):
        return (
            f"Cannot reach {target}. Connect to the Pi WiFi and make sure "
            "you are SSHing into the correct Pi host."
        )

    if any(
        s in lower
        for s in (
            "permission denied",
            "authentication failed",
            "auth fail",
            "access denied",
        )
    ):
        if config["ssh_pass"]:
            return (
                f"SSH reached {target}, but the configured password was rejected. "
                "Enter the correct SSH password in Settings."
            )
        return (
            f"SSH reached {target}, but password authentication is required and no SSH password is set. "
            "Enter the Pi SSH password in Settings."
        )

    if "host key verification failed" in lower:
        return (
            f"SSH host key verification failed for {target}. "
            "Clear the stale known_hosts entry for this host and retry."
        )

    if "ssh password auth is configured, but `sshpass` is not installed" in lower:
        return raw

    return raw or (
        f"SSH failed for {target}. Connect to the Pi WiFi and verify SSH target/user/password in Settings."
    )


def _friendly_ssh_timeout() -> str:
    target = _ssh_target()
    return (
        f"SSH to {target} timed out. Connect to the Pi WiFi and make sure "
        "you are SSHing into the correct Pi host."
    )


def _format_remote_error(raw_error: str, prefix: str) -> str:
    """Keep context-specific prefixes unless this is clearly an SSH reachability/auth issue."""
    raw = (raw_error or "").strip()
    friendly = _friendly_ssh_error(raw)
    if friendly != raw or not raw:
        return friendly
    return f"{prefix}: {raw}"


def _q(value: str) -> str:
    return shlex.quote(value)


def _mission_paths() -> dict:
    remote_dir = config["remote_dir"]
    return {
        "log": f"{remote_dir}/.mission_main_launch.log",
        "pid": f"{remote_dir}/.mission_main_launch.pid",
        "pgid": f"{remote_dir}/.mission_main_launch.pgid",
        "launch_params": f"{remote_dir}/src/uav/launch/launch_params.yaml",
    }


async def _mission_launch_status() -> dict:
    paths = _mission_paths()
    cmd = f"""
        pid_file={_q(paths["pid"])}
        log_file={_q(paths["log"])}
        if [ -f "$pid_file" ]; then
            pid="$(cat "$pid_file" 2>/dev/null)"
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                echo "RUNNING:$pid"
                exit 0
            fi
            echo "STOPPED"
            exit 0
        fi
        if [ -f "$log_file" ]; then
            echo "STOPPED"
        else
            echo "NOT_PREPARED"
        fi
    """
    r = await _run_ssh(cmd, timeout=8)
    if r.returncode != 0:
        return {
            "success": False,
            "running": False,
            "state": "error",
            "error": _friendly_ssh_error(r.stderr),
        }
    out = r.stdout.strip()
    if out.startswith("RUNNING:"):
        return {
            "success": True,
            "running": True,
            "state": "running",
            "pid": out.split(":", 1)[1],
        }
    if out == "NOT_PREPARED":
        return {"success": True, "running": False, "state": "not_prepared"}
    return {"success": True, "running": False, "state": "stopped"}


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
        return {
            "success": True,
            "connected": False,
            "error": _friendly_ssh_error(r.stderr),
        }
    except subprocess.TimeoutExpired:
        return {"success": True, "connected": False, "error": _friendly_ssh_timeout()}
    except Exception as e:
        return {
            "success": True,
            "connected": False,
            "error": _friendly_ssh_error(str(e)),
        }


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
        r = await _run_ssh(
            "nmcli -f NAME,TYPE,DEVICE,STATE con show --active | tail -n +2", timeout=15
        )
        if r.returncode != 0:
            return {"success": False, "error": _friendly_ssh_error(r.stderr)}

        connections = []
        for line in r.stdout.strip().split("\n"):
            if not line.strip():
                continue
            parts = re.split(r"\s{2,}", line.strip())
            if len(parts) >= 4:
                connections.append(
                    {
                        "name": parts[0].strip(),
                        "type": parts[1].strip(),
                        "device": parts[2].strip(),
                        "state": parts[3].strip(),
                    }
                )

        is_hotspot = any(c["name"] == config["hotspot_name"] for c in connections)
        wifi_con = next(
            (c for c in connections if c["type"] == "802-11-wireless"), None
        )

        return {
            "success": True,
            "is_hotspot": is_hotspot,
            "current_wifi": wifi_con["name"] if wifi_con else None,
            "connections": connections,
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": _friendly_ssh_timeout()}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


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
            return {
                "success": False,
                "error": _friendly_ssh_error(r.stderr),
                "networks": [],
            }

        networks = []
        for line in r.stdout.strip().split("\n"):
            if not line.strip():
                continue
            # nmcli tabular output is whitespace-aligned; parse by splitting on 2+ spaces
            parts = re.split(r"\s{2,}", line.strip())
            if len(parts) >= 3:
                ssid = parts[0].strip()
                if not ssid:
                    continue
                try:
                    signal = int(parts[1].strip())
                except ValueError:
                    signal = 0
                security = parts[2].strip() if len(parts) > 2 else ""
                networks.append(
                    {
                        "ssid": ssid,
                        "signal": signal,
                        "security": security,
                    }
                )

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
        return {"success": False, "error": _friendly_ssh_error(str(e)), "networks": []}


@app.post("/api/wifi/connect")
async def wifi_connect(ssid: str = Form(...), password: str = Form("")):
    """Connect the Pi to a WiFi network (disables hotspot)."""
    try:
        # Bring down hotspot
        await _run_ssh(
            f"nmcli con down '{config['hotspot_name']}' 2>/dev/null; sleep 2",
            timeout=15,
        )

        # Connect
        pwd_arg = f' password "{password}"' if password else ""
        r = await _run_ssh(f'nmcli dev wifi connect "{ssid}"{pwd_arg}', timeout=30)

        if r.returncode != 0:
            # Restore hotspot on failure
            await _run_ssh(f"nmcli con up '{config['hotspot_name']}'", timeout=15)
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "Failed to connect"),
            }

        return {"success": True, "output": f"Pi connected to {ssid}"}
    except Exception as e:
        await _run_ssh(f"nmcli con up '{config['hotspot_name']}'", timeout=15)
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.post("/api/wifi/hotspot")
async def wifi_hotspot():
    """Switch Pi back to hotspot mode."""
    try:
        await _run_ssh("nmcli dev disconnect wlan0 2>/dev/null; sleep 1", timeout=15)
        r = await _run_ssh(f"nmcli con up '{config['hotspot_name']}'", timeout=15)
        if r.returncode != 0:
            return {"success": False, "error": _friendly_ssh_error(r.stderr)}
        return {"success": True, "output": "Hotspot activated"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.post("/api/wifi/switch-local")
async def switch_local_wifi(ssid: str = Form(...), password: str = Form("")):
    """Switch THIS Mac's WiFi to match (macOS only)."""
    try:
        cmd = ["networksetup", "-setairportnetwork", "en0", ssid]
        if password:
            cmd.append(password)
        r = await asyncio.to_thread(
            subprocess.run, cmd, capture_output=True, text=True, timeout=15
        )
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


def _require_httpx():
    try:
        import httpx
    except ModuleNotFoundError as e:
        raise RuntimeError(
            "Missing Python dependency `httpx`. "
            "If using conda/pip, run: pip install httpx. "
            "If using uv, rerun with `uv run app.py`."
        ) from e
    return httpx


# ── Build Management (via SCP/SSH) ──────────────────────────


@app.get("/api/builds/current")
async def current_build():
    """Get info about the currently deployed build on the Pi."""
    try:
        r = await _run_ssh(
            f"cat {config['remote_dir']}/install/BUILD_INFO.txt 2>/dev/null", timeout=10
        )
        if r.returncode == 0 and r.stdout.strip():
            return {"success": True, "installed": True, "info": r.stdout.strip()}

        r = await _run_ssh(
            f"test -d {config['remote_dir']}/install && echo yes || echo no", timeout=10
        )
        if r.returncode != 0:
            return {
                "success": True,
                "installed": False,
                "info": _friendly_ssh_error(r.stderr),
            }
        if r.stdout.strip() == "yes":
            return {
                "success": True,
                "installed": True,
                "info": "Build installed (no BUILD_INFO.txt)",
            }

        return {"success": True, "installed": False, "info": "No build deployed"}
    except Exception as e:
        return {
            "success": True,
            "installed": False,
            "info": _friendly_ssh_error(str(e)),
        }


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
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "SCP failed"),
            }

        # Backup + extract on Pi
        extract_cmd = f"""
            cd {config["remote_dir"]}
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
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "Extract failed"),
            }

        return {"success": True, "output": f"Deployed {file.filename} to Pi"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.get("/api/builds/list")
async def list_builds():
    """List available builds from GitHub Releases."""
    if not config["github_repo"]:
        return {"success": False, "error": "GITHUB_REPO not configured", "builds": []}
    try:
        httpx = _require_httpx()
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
                builds.append(
                    {
                        "tag": rel["tag_name"],
                        "sha": rel["tag_name"].replace("build-", ""),
                        "name": rel.get("name", rel["tag_name"]),
                        "date": rel.get("published_at", "")[:10],
                        "download_url": assets[0]["browser_download_url"]
                        if assets
                        else None,
                        "size_mb": round(assets[0]["size"] / 1_000_000, 1)
                        if assets
                        else None,
                    }
                )
        return {"success": True, "builds": builds[:20]}
    except Exception as e:
        return {"success": False, "error": str(e), "builds": []}


@app.post("/api/builds/download")
async def download_build(tag: str = Form(...)):
    """Download a build from GitHub on your Mac, then SCP it to the Pi."""
    if not config["github_repo"]:
        return {"success": False, "error": "GITHUB_REPO not configured"}
    try:
        httpx = _require_httpx()

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
            async with client.stream(
                "GET", download_url, headers=_github_headers()
            ) as resp:
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
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "SCP failed"),
            }

        # Backup + extract on Pi
        extract_cmd = f"""
            cd {config["remote_dir"]}
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
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "Extract failed"),
            }

        return {"success": True, "output": f"Downloaded {tag} and deployed to Pi"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.post("/api/builds/rollback")
async def rollback_build():
    """Rollback to the previous build on the Pi."""
    try:
        rollback_cmd = f"""
            cd {config["remote_dir"]}
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
            return {"success": False, "error": _friendly_ssh_error(r.stderr)}

        return {"success": True, "output": "Rolled back to previous build"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


# ── Failsafe ────────────────────────────────────────────────


@app.post("/api/failsafe")
async def trigger_failsafe():
    """Trigger failsafe on the Pi via ros2 topic pub."""
    try:
        cmd = (
            f"cd {_q(config['remote_dir'])} && "
            f"source /opt/ros/humble/setup.bash && "
            f"source install/setup.bash && "
            f'timeout 3 ros2 topic pub /failsafe_trigger std_msgs/msg/Bool "{{data: true}}"'
        )
        r = await _run_ssh(f"bash -c '{cmd}'", timeout=15)
        # timeout(1) returns 124 when it stops the publisher; that still means
        # the command ran and published while active.
        if r.returncode not in (0, 124):
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "Failsafe command failed"),
            }
        return {"success": True, "output": "Failsafe triggered"}
    except subprocess.TimeoutExpired:
        return {"success": False, "error": _friendly_ssh_timeout()}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


# ── Mission Control ────────────────────────────────────────


@app.get("/api/mission/launch/status")
async def mission_launch_status():
    """Check if ros2 launch uav main.launch.py is running on the Pi."""
    return await _mission_launch_status()


@app.get("/api/mission/launch/logs")
async def mission_launch_logs(lines: int = 200, offset: int | None = None):
    """Get recent launch output from the Pi."""
    try:
        paths = _mission_paths()
        status = await _mission_launch_status()

        if offset is not None:
            start = max(0, int(offset))
            cmd = f"""
                log_file={_q(paths["log"])}
                start={start}
                if [ ! -f "$log_file" ]; then
                    echo "__META__:0:0"
                    exit 0
                fi

                size="$(wc -c < "$log_file" | tr -d ' ')"
                reset=0
                if [ "$start" -gt "$size" ]; then
                    start=0
                    reset=1
                fi

                echo "__META__:$size:$reset"
                if [ "$size" -gt "$start" ]; then
                    tail -c +$((start + 1)) "$log_file"
                fi
            """
        else:
            cat_cmd = (
                'cat "$log_file"'
                if line_count == 0
                else f'tail -n {line_count} "$log_file"'
            )
            line_count = 0 if lines <= 0 else max(20, min(lines, 20000))
            cmd = f"""
                log_file={_q(paths["log"])}
                if [ -f "$log_file" ]; then
                    {cat_cmd}
                fi
            """

        r = await _run_ssh(cmd, timeout=10)
        if r.returncode != 0:
            return {
                "success": False,
                "running": status.get("running", False),
                "error": _friendly_ssh_error(r.stderr),
            }

        if offset is not None:
            out = r.stdout or ""
            first_line, sep, rest = out.partition("\n")
            meta_match = re.match(r"^__META__:(\d+):([01])$", first_line.strip())
            if not meta_match:
                return {
                    "success": False,
                    "running": status.get("running", False),
                    "error": "Failed to parse mission logs metadata",
                }
            if not sep:
                rest = ""
            next_offset = int(meta_match.group(1))
            reset = meta_match.group(2) == "1"
            return {
                "success": True,
                "running": status.get("running", False),
                "logs": rest,
                "next_offset": next_offset,
                "reset": reset,
            }

        return {
            "success": True,
            "running": status.get("running", False),
            "logs": r.stdout,
        }
    except Exception as e:
        return {
            "success": False,
            "running": False,
            "error": _friendly_ssh_error(str(e)),
        }


@app.post("/api/mission/prepare")
async def prepare_mission():
    """Start ros2 launch uav main.launch.py on the Pi and stream logs from file."""
    try:
        paths = _mission_paths()
        cmd = f"""
            set -e
            cd {_q(config["remote_dir"])}
            source /opt/ros/humble/setup.bash
            source install/setup.bash

            pid_file={_q(paths["pid"])}
            pgid_file={_q(paths["pgid"])}
            log_file={_q(paths["log"])}
            had_previous=0

            if [ -f "$pid_file" ]; then
                old_pid="$(cat "$pid_file" 2>/dev/null || true)"
                old_pgid=""
                if [ -f "$pgid_file" ]; then
                    old_pgid="$(cat "$pgid_file" 2>/dev/null || true)"
                fi
                if [ -z "$old_pgid" ] && [ -n "$old_pid" ]; then
                    old_pgid="$(ps -o pgid= "$old_pid" 2>/dev/null | tr -d ' ' || true)"
                fi

                target=""
                if [ -n "$old_pgid" ] && kill -0 "-$old_pgid" 2>/dev/null; then
                    target="-$old_pgid"
                elif [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                    target="$old_pid"
                fi

                if [ -n "$target" ]; then
                    had_previous=1
                    kill -INT "$target" 2>/dev/null || true
                    for _ in 1 2 3 4 5; do
                        if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                            sleep 1
                        else
                            break
                        fi
                    done
                    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                        kill -TERM "$target" 2>/dev/null || true
                        sleep 1
                    fi
                    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                        kill -KILL "$target" 2>/dev/null || true
                    fi
                fi

                rm -f "$pid_file"
            fi
            rm -f "$pgid_file"

            : > "$log_file"
            nohup setsid ros2 launch uav main.launch.py >> "$log_file" 2>&1 < /dev/null &
            new_pid=$!
            new_pgid="$(ps -o pgid= "$new_pid" 2>/dev/null | tr -d ' ' || true)"
            echo "$new_pid" > "$pid_file"
            if [ -n "$new_pgid" ]; then
                echo "$new_pgid" > "$pgid_file"
            fi
            sleep 1

            if kill -0 "$new_pid" 2>/dev/null; then
                if [ "$had_previous" -eq 1 ]; then
                    echo "RESTARTED:$new_pid"
                else
                    echo "STARTED:$new_pid"
                fi
                exit 0
            fi

            echo "FAILED"
            exit 1
        """
        r = await _run_ssh(f"bash -lc {_q(cmd)}", timeout=20)
        stdout = (r.stdout or "").strip()
        stderr = (r.stderr or "").strip()
        combined = "\n".join(part for part in (stdout, stderr) if part)
        match = re.search(r"(STARTED|RESTARTED)\s*:\s*([0-9]+)", combined)
        if match:
            pid = match.group(2)
            if match.group(1) == "RESTARTED":
                return {
                    "success": True,
                    "output": "Mission launch restarted",
                    "running": True,
                    "pid": pid,
                }
            return {
                "success": True,
                "output": "Mission launch started",
                "running": True,
                "pid": pid,
            }

        if r.returncode != 0:
            return {
                "success": False,
                "error": _format_remote_error(
                    r.stderr or r.stdout, "Prepare mission failed"
                ),
            }

        return {
            "success": False,
            "error": "Prepare mission failed (launch did not start)",
        }
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.post("/api/mission/stop")
async def stop_mission():
    """Stop the mission launch process (Ctrl+C style first, then escalate if needed)."""
    try:
        paths = _mission_paths()
        cmd = f"""
            pid_file={_q(paths["pid"])}
            pgid_file={_q(paths["pgid"])}

            if [ ! -f "$pid_file" ]; then
                echo "NOT_RUNNING"
                exit 0
            fi

            pid="$(cat "$pid_file" 2>/dev/null || true)"
            pgid=""
            if [ -f "$pgid_file" ]; then
                pgid="$(cat "$pgid_file" 2>/dev/null || true)"
            fi

            target=""
            if [ -n "$pgid" ] && kill -0 "-$pgid" 2>/dev/null; then
                target="-$pgid"
            elif [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                target="$pid"
            else
                rm -f "$pid_file" "$pgid_file"
                echo "NOT_RUNNING"
                exit 0
            fi

            # Ctrl+C equivalent first.
            kill -INT "$target" 2>/dev/null || true
            for _ in 1 2 3 4 5; do
                if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                    sleep 1
                else
                    rm -f "$pid_file" "$pgid_file"
                    echo "STOPPED"
                    exit 0
                fi
            done

            # Escalate if still alive.
            kill -TERM "$target" 2>/dev/null || true
            sleep 1
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$target" 2>/dev/null || true
            fi
            rm -f "$pid_file" "$pgid_file"
            echo "STOPPED"
        """
        r = await _run_ssh(f"bash -lc {_q(cmd)}", timeout=20)
        if r.returncode != 0:
            return {
                "success": False,
                "error": _format_remote_error(
                    r.stderr or r.stdout, "Stop mission failed"
                ),
            }

        out = (r.stdout or "").strip()
        if "NOT_RUNNING" in out:
            return {"success": True, "output": "Mission launch is not running"}
        return {"success": True, "output": "Mission launch stopped"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.post("/api/mission/start")
async def start_mission():
    """Call /mode_manager/start_mission service on the Pi."""
    try:
        cmd = (
            f"cd {_q(config['remote_dir'])} && "
            f"source /opt/ros/humble/setup.bash && "
            f"source install/setup.bash && "
            f'ros2 service call /mode_manager/start_mission std_srvs/srv/Trigger "{{}}"'
        )
        r = await _run_ssh(f"bash -lc {_q(cmd)}", timeout=20)
        if r.returncode != 0:
            return {
                "success": False,
                "error": _format_remote_error(
                    r.stderr or r.stdout, "Start mission failed"
                ),
            }
        return {
            "success": True,
            "output": r.stdout.strip() or "Start mission service called",
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Start mission service call timed out"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.get("/api/mission/launch-params")
async def get_launch_params():
    """Read launch_params.yaml from the Pi workspace."""
    try:
        params_path = _mission_paths()["launch_params"]
        r = await _run_ssh(f"cat {_q(params_path)}", timeout=10)
        if r.returncode != 0:
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "Read launch params failed"),
            }
        return {"success": True, "content": r.stdout}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}


@app.post("/api/mission/launch-params")
async def set_launch_params(content: str = Form(...)):
    """Write launch_params.yaml on the Pi workspace."""
    tmp_path = ""
    try:
        params_path = _mission_paths()["launch_params"]
        await _run_ssh(f"mkdir -p {_q(str(Path(params_path).parent))}", timeout=10)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name

        r = await _run_scp(tmp_path, params_path, timeout=30)
        if r.returncode != 0:
            return {
                "success": False,
                "error": _format_remote_error(r.stderr, "Write launch params failed"),
            }

        return {"success": True, "output": "launch_params.yaml updated"}
    except Exception as e:
        return {"success": False, "error": _friendly_ssh_error(str(e))}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


# ── Static Files (production) ────────────────────────────────

frontend_dist = Path(__file__).parent / "frontend" / "dist"
if frontend_dist.exists():
    app.mount(
        "/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend"
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8080)
