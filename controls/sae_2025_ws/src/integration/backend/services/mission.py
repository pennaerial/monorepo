from __future__ import annotations

import asyncio
import os
import re
import subprocess
import tempfile
from contextlib import suppress
from pathlib import Path

from fastapi import WebSocket, WebSocketDisconnect
import yaml
from uav.runtime.mission_spec import MissionSpec

from ..context import AppContext, TargetContext
from ..models import TerminalChunkMessage, TerminalInfoMessage
from . import deploy as deploy_service


MISSION_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def _is_offline_error(message: str) -> bool:
    lower = (message or "").lower()
    return "cannot find " in lower or "cannot reach " in lower or "timed out" in lower


def _normalize_mission_name(name: str) -> str:
    mission = (name or "").strip()
    if not mission:
        raise ValueError("Mission name is required")
    if not MISSION_NAME_RE.fullmatch(mission):
        raise ValueError(
            "Invalid mission name. Use letters, numbers, underscores, or dashes."
        )
    return mission


def _runtime_paths(target_ctx: TargetContext) -> dict[str, str]:
    return target_ctx.target.deploy_paths()


def _runtime_shell(target_ctx: TargetContext, command: str) -> str:
    paths = _runtime_paths(target_ctx)
    return f"""
        set -e
        cd {target_ctx.ssh.q(paths["deploy_root"])}
        source /opt/ros/humble/setup.bash
        source {target_ctx.ssh.q(paths["current_link"])}/install/setup.bash
        {command}
    """


def _systemd_access_error(target_ctx: TargetContext, raw: str) -> str:
    lower = (raw or "").lower()
    if "password is required" in lower or "sudo:" in lower:
        return (
            f"Systemd control on {target_ctx.target.ssh_target()} requires sudo. "
            "Bootstrap the Pi or grant passwordless sudo for install/systemctl."
        )
    return target_ctx.ssh.friendly_error(raw)


def _require_runtime_target(ctx: AppContext, target_ctx: TargetContext) -> None:
    ctx.require_deploy_context()
    if not target_ctx.target.vehicle_name:
        raise ValueError(
            f"Target '{target_ctx.target.target_id}' must be assigned to a controllable before using runtime controls."
        )


async def probe_launch_status(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
    except ValueError as exc:
        return {
            "success": False,
            "running": False,
            "state": "error",
            "error": str(exc),
        }
    paths = _runtime_paths(target_ctx)
    cmd = f"""
        current_link={target_ctx.ssh.q(paths["current_link"])}
        runtime_fleet={target_ctx.ssh.q(paths["runtime_fleet"])}
        if [ ! -d "$current_link" ] || [ ! -f "$runtime_fleet" ]; then
            echo "NOT_PREPARED"
            exit 0
        fi

        state="$(sudo -n systemctl is-active {target_ctx.target.service_unit} 2>/dev/null || true)"
        pid="$(sudo -n systemctl show -p MainPID --value {target_ctx.target.service_unit} 2>/dev/null || echo 0)"

        case "$state" in
            active)
                echo "RUNNING:${{pid:-0}}"
                ;;
            inactive|failed|activating|deactivating)
                echo "STOPPED"
                ;;
            *)
                echo "STOPPED"
                ;;
        esac
    """
    result = await target_ctx.ssh.run(f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=8)
    if result.returncode != 0:
        error = _systemd_access_error(target_ctx, result.stderr or result.stdout)
        return {
            "success": False,
            "running": False,
            "state": "offline" if _is_offline_error(error) else "error",
            "error": error,
        }

    out = (result.stdout or "").strip()
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


async def refresh_runtime_state(
    ctx: AppContext, *, target_id: str | None = None
) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    status = await probe_launch_status(ctx, target_id=target_id)
    await target_ctx.mission_state.apply_launch_status(
        success=status.get("success", False),
        state=status.get("state", "error"),
        running=status.get("running", False),
        pid=status.get("pid"),
        error=status.get("error"),
    )
    return status


async def mission_state(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    await refresh_runtime_state(ctx, target_id=target_id)
    state = await target_ctx.mission_state.snapshot()
    return {"success": True, "state": state.model_dump()}


async def launch_status(ctx: AppContext, *, target_id: str | None = None) -> dict:
    return await refresh_runtime_state(ctx, target_id=target_id)


async def launch_logs(
    ctx: AppContext,
    *,
    target_id: str | None = None,
    lines: int = 200,
    offset: int | None = None,
    inode: int | None = None,
) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        status = await refresh_runtime_state(ctx, target_id=target_id)
        line_count = 400 if lines <= 0 else max(20, min(lines, 4000))
        cmd = f"""
            sudo -n journalctl -u {target_ctx.target.service_unit} --no-pager -n {line_count} -o short-iso 2>/dev/null || true
        """
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=10
        )
        if result.returncode != 0:
            return {
                "success": False,
                "running": status.get("running", False),
                "error": _systemd_access_error(
                    target_ctx, result.stderr or result.stdout
                ),
            }

        text = result.stdout or ""
        return {
            "success": True,
            "running": status.get("running", False),
            "logs": text,
            "next_offset": len(text),
            "inode": 0 if inode is None else inode,
            "reset": True,
        }
    except Exception as exc:
        return {
            "success": False,
            "running": False,
            "error": target_ctx.ssh.friendly_error(str(exc)),
        }


async def stream_terminal(
    ctx: AppContext,
    websocket: WebSocket,
    *,
    target_id: str | None = None,
    offset: int = 0,
    inode: int = 0,
) -> None:
    await websocket.accept()
    previous = ""
    try:
        while True:
            chunk = await launch_logs(
                ctx,
                target_id=target_id,
                lines=400,
                offset=offset,
                inode=inode,
            )
            if not chunk.get("success"):
                message = TerminalInfoMessage(
                    type="error",
                    message=chunk.get("error") or "Terminal stream failed",
                )
                await websocket.send_text(message.model_dump_json())
                if _is_offline_error(chunk.get("error") or ""):
                    break
                await asyncio.sleep(1.0)
                continue

            text = chunk.get("logs") or ""
            reset = text != previous
            previous = text
            if reset:
                payload = TerminalChunkMessage(
                    data=text,
                    next_offset=len(text),
                    inode=0,
                    reset=True,
                )
                await websocket.send_text(payload.model_dump_json())

            with suppress(asyncio.TimeoutError):
                await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        return
    except Exception as exc:
        with suppress(Exception):
            message = TerminalInfoMessage(
                type="error",
                message=ctx.resolve_target(target_id).ssh.friendly_error(str(exc)),
            )
            await websocket.send_text(message.model_dump_json())
    finally:
        with suppress(Exception):
            await websocket.close()


async def prepare_mission(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
    except ValueError as exc:
        return {"success": False, "error": str(exc)}
    await target_ctx.mission_state.set(
        phase="preparing",
        launch_state="running",
        running=False,
        message="Starting mission runtime",
        error=None,
    )
    try:
        paths = _runtime_paths(target_ctx)
        cmd = f"""
            current_link={target_ctx.ssh.q(paths["current_link"])}
            runtime_fleet={target_ctx.ssh.q(paths["runtime_fleet"])}
            if [ ! -d "$current_link" ] || [ ! -f "$runtime_fleet" ]; then
                echo "NOT_PREPARED"
                exit 1
            fi
            sudo -n systemctl restart {target_ctx.target.service_unit}
            pid="$(sudo -n systemctl show -p MainPID --value {target_ctx.target.service_unit} 2>/dev/null || echo 0)"
            echo "STARTED:${{pid:-0}}"
        """
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=25
        )
        if result.returncode != 0:
            combined = result.stderr or result.stdout
            if "NOT_PREPARED" in combined:
                error = "No prepared runtime is available on the Pi. Deploy a build and target overlay first."
            else:
                error = _systemd_access_error(target_ctx, combined)
            await target_ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        match = re.search(r"STARTED:([0-9]+)", result.stdout or "")
        pid = match.group(1) if match else None
        await target_ctx.mission_state.set(
            phase="running",
            launch_state="running",
            running=True,
            pid=pid,
            message="Mission runtime running",
            error=None,
        )
        return {"success": True, "output": "Mission runtime started", "pid": pid}
    except Exception as exc:
        error = target_ctx.ssh.friendly_error(str(exc))
        await target_ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def stop_mission(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    await target_ctx.mission_state.set(
        phase="stopping",
        launch_state="stopped",
        running=True,
        message="Stopping mission runtime",
        error=None,
    )
    try:
        result = await target_ctx.ssh.run(
            f"sudo -n systemctl stop {target_ctx.target.service_unit}", timeout=20
        )
        if result.returncode != 0:
            error = _systemd_access_error(target_ctx, result.stderr or result.stdout)
            await target_ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        await target_ctx.mission_state.set(
            phase="idle",
            launch_state="stopped",
            running=False,
            pid=None,
            message="Mission runtime stopped",
            error=None,
        )
        return {"success": True, "output": "Mission runtime stopped"}
    except Exception as exc:
        error = target_ctx.ssh.friendly_error(str(exc))
        await target_ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def _call_vehicle_service(
    ctx: AppContext,
    *,
    target_id: str | None,
    service_name: str,
    timeout: int,
    error_prefix: str,
) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
        cmd = _runtime_shell(
            target_ctx,
            (
                f"ros2 service call /{target_ctx.target.vehicle_name}/{service_name} "
                'std_srvs/srv/Trigger "{}"'
            ),
        )
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=timeout
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": target_ctx.ssh.format_remote_error(
                    result.stderr or result.stdout, error_prefix
                ),
            }
        return {
            "success": True,
            "output": result.stdout.strip() or f"{service_name} called",
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": f"{error_prefix} timed out"}
    except Exception as exc:
        return {"success": False, "error": target_ctx.ssh.friendly_error(str(exc))}


async def start_mission(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    result = await _call_vehicle_service(
        ctx,
        target_id=target_id,
        service_name="mode_manager/start_mission",
        timeout=20,
        error_prefix="Start mission failed",
    )
    if result.get("success"):
        await refresh_runtime_state(ctx, target_id=target_id)
    else:
        await target_ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=result.get("error"),
        )
    return result


async def trigger_failsafe(ctx: AppContext, *, target_id: str | None = None) -> dict:
    return await _call_vehicle_service(
        ctx,
        target_id=target_id,
        service_name="mode_manager/failsafe",
        timeout=15,
        error_prefix="Failsafe command failed",
    )


async def get_launch_params(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
        overlay_path = target_ctx.target.mission_paths()["overlay_file"]
        result = await target_ctx.ssh.run(
            f"cat {target_ctx.ssh.q(overlay_path)}", timeout=10
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": target_ctx.ssh.format_remote_error(
                    result.stderr, "Read target overlay failed"
                ),
            }
        target_ctx.target.overlay_yaml = result.stdout
        ctx.inventory.save()
        return {"success": True, "content": result.stdout}
    except Exception as exc:
        return {"success": False, "error": target_ctx.ssh.friendly_error(str(exc))}


async def list_mission_names(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
        missions_dir = target_ctx.target.mission_paths()["missions_dir"]
        cmd = f"""
            missions_dir={target_ctx.ssh.q(missions_dir)}
            if [ ! -d "$missions_dir" ]; then
                echo "__ERR__:missing_dir"
                exit 0
            fi
            for f in "$missions_dir"/*.yaml "$missions_dir"/*.yml; do
                [ -e "$f" ] || continue
                base="$(basename "$f")"
                case "$base" in
                    *.yaml) echo "${{base%.yaml}}" ;;
                    *.yml) echo "${{base%.yml}}" ;;
                esac
            done | sort -u
        """
        result = await target_ctx.ssh.run(cmd, timeout=10)
        if result.returncode != 0:
            return {
                "success": False,
                "missions": [],
                "error": target_ctx.ssh.format_remote_error(
                    result.stderr, "Read mission names failed"
                ),
            }

        lines = [
            line.strip() for line in (result.stdout or "").splitlines() if line.strip()
        ]
        if lines and lines[0] == "__ERR__:missing_dir":
            return {
                "success": False,
                "missions": [],
                "error": f"Mission directory not found on Pi: {missions_dir}",
            }

        return {"success": True, "missions": lines}
    except Exception as exc:
        return {
            "success": False,
            "missions": [],
            "error": target_ctx.ssh.friendly_error(str(exc)),
        }


async def _resolve_mission_file_path(
    ctx: AppContext, mission_name: str, *, target_id: str | None, allow_create: bool
) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
    except ValueError as exc:
        return {"success": False, "error": str(exc)}
    missions_dir = target_ctx.target.mission_paths()["missions_dir"]
    cmd = f"""
        missions_dir={target_ctx.ssh.q(missions_dir)}
        mission_name={target_ctx.ssh.q(mission_name)}
        yaml_path="$missions_dir/$mission_name.yaml"
        yml_path="$missions_dir/$mission_name.yml"

        if [ -f "$yaml_path" ]; then
            echo "$yaml_path"
            exit 0
        fi
        if [ -f "$yml_path" ]; then
            echo "$yml_path"
            exit 0
        fi
        if [ {1 if allow_create else 0} -eq 1 ]; then
            echo "$yaml_path"
            exit 0
        fi
        echo "__ERR__:not_found"
    """
    result = await target_ctx.ssh.run(cmd, timeout=10)
    if result.returncode != 0:
        return {
            "success": False,
            "error": target_ctx.ssh.format_remote_error(
                result.stderr, "Resolve mission file failed"
            ),
        }

    resolved = (result.stdout or "").strip()
    if not resolved or resolved == "__ERR__:not_found":
        return {
            "success": False,
            "error": (
                f"Mission file '{mission_name}.yaml' was not found in "
                f"{missions_dir}. Select a valid mission in the target overlay."
            ),
        }

    return {"success": True, "path": resolved}


async def get_mission_file(
    ctx: AppContext, *, target_id: str | None = None, name: str
) -> dict:
    try:
        mission_name = _normalize_mission_name(name)
    except ValueError as exc:
        return {"success": False, "mission": None, "error": str(exc)}

    target_ctx = ctx.resolve_target(target_id)
    try:
        resolved = await _resolve_mission_file_path(
            ctx, mission_name, target_id=target_id, allow_create=False
        )
        if not resolved.get("success"):
            return {
                "success": False,
                "mission": mission_name,
                "error": resolved.get("error", "Mission file not found"),
            }

        mission_path = resolved["path"]
        result = await target_ctx.ssh.run(
            f"cat {target_ctx.ssh.q(mission_path)}", timeout=10
        )
        if result.returncode != 0:
            return {
                "success": False,
                "mission": mission_name,
                "path": mission_path,
                "error": target_ctx.ssh.format_remote_error(
                    result.stderr, "Read mission YAML failed"
                ),
            }

        return {
            "success": True,
            "mission": mission_name,
            "path": mission_path,
            "content": result.stdout,
        }
    except Exception as exc:
        return {
            "success": False,
            "mission": mission_name,
            "error": target_ctx.ssh.friendly_error(str(exc)),
        }


async def set_mission_file(
    ctx: AppContext, *, target_id: str | None = None, name: str, content: str
) -> dict:
    tmp_path = ""
    try:
        mission_name = _normalize_mission_name(name)
    except ValueError as exc:
        return {"success": False, "mission": None, "error": str(exc)}

    target_ctx = ctx.resolve_target(target_id)
    try:
        parsed = yaml.safe_load(content) or {}
        if not isinstance(parsed, dict):
            raise ValueError("Mission YAML must define a mapping at the top level.")
        MissionSpec.from_dict(parsed)

        missions_dir = target_ctx.target.mission_paths()["missions_dir"]
        mkdir_result = await target_ctx.ssh.run(
            f"mkdir -p {target_ctx.ssh.q(missions_dir)}", timeout=10
        )
        if mkdir_result.returncode != 0:
            return {
                "success": False,
                "mission": mission_name,
                "error": target_ctx.ssh.format_remote_error(
                    mkdir_result.stderr,
                    "Write mission YAML failed",
                ),
            }

        resolved = await _resolve_mission_file_path(
            ctx, mission_name, target_id=target_id, allow_create=True
        )
        if not resolved.get("success"):
            return {
                "success": False,
                "mission": mission_name,
                "error": resolved.get("error", "Failed to resolve mission file"),
            }

        mission_path = resolved["path"]
        suffix = ".yml" if mission_path.endswith(".yml") else ".yaml"
        with tempfile.NamedTemporaryFile(mode="w", suffix=suffix, delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name

        scp_result = await target_ctx.ssh.scp(tmp_path, mission_path, timeout=30)
        if scp_result.returncode != 0:
            return {
                "success": False,
                "mission": mission_name,
                "path": mission_path,
                "error": target_ctx.ssh.format_remote_error(
                    scp_result.stderr, "Write mission YAML failed"
                ),
            }

        return {
            "success": True,
            "mission": mission_name,
            "path": mission_path,
            "output": f"Mission YAML updated ({mission_name})",
        }
    except Exception as exc:
        return {
            "success": False,
            "mission": mission_name,
            "error": target_ctx.ssh.friendly_error(str(exc)),
        }
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


async def set_launch_params(
    ctx: AppContext, *, target_id: str | None = None, content: str
) -> dict:
    tmp_path = ""
    target_ctx = ctx.resolve_target(target_id)
    try:
        _require_runtime_target(ctx, target_ctx)
        deploy_service.validate_overlay_preview(ctx, target_ctx, content)
        overlay_path = target_ctx.target.mission_paths()["overlay_file"]
        mkdir_result = await target_ctx.ssh.run(
            f"mkdir -p {target_ctx.ssh.q(str(Path(overlay_path).parent))}", timeout=10
        )
        if mkdir_result.returncode != 0:
            return {
                "success": False,
                "error": target_ctx.ssh.format_remote_error(
                    mkdir_result.stderr,
                    "Write target overlay failed",
                ),
            }

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name

        result = await target_ctx.ssh.scp(tmp_path, overlay_path, timeout=30)
        if result.returncode != 0:
            return {
                "success": False,
                "error": target_ctx.ssh.format_remote_error(
                    result.stderr, "Write target overlay failed"
                ),
            }

        target_ctx.target.overlay_yaml = content
        ctx.inventory.save()
        return {"success": True, "output": "Target overlay updated"}
    except Exception as exc:
        return {"success": False, "error": target_ctx.ssh.friendly_error(str(exc))}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)
