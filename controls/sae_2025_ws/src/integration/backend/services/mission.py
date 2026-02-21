from __future__ import annotations

import asyncio
import logging
import os
import re
import subprocess
import sys
import tempfile
import time
from contextlib import suppress
from pathlib import Path

logger = logging.getLogger(__name__)

from fastapi import WebSocket, WebSocketDisconnect

from ..context import AppContext
from ..models import TerminalChunkMessage, TerminalInfoMessage


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


async def probe_launch_status(ctx: AppContext) -> dict:
    paths = ctx.config.mission_paths()
    cmd = f"""
        pid_file={ctx.ssh.q(paths["pid"])}
        log_file={ctx.ssh.q(paths["log"])}
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
    result = await ctx.ssh.run(cmd, timeout=8)
    if result.returncode != 0:
        error = ctx.ssh.friendly_error(result.stderr)
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


async def refresh_runtime_state(ctx: AppContext) -> dict:
    status = await probe_launch_status(ctx)
    await ctx.mission_state.apply_launch_status(
        success=status.get("success", False),
        state=status.get("state", "error"),
        running=status.get("running", False),
        pid=status.get("pid"),
        error=status.get("error"),
    )
    return status


async def mission_state(ctx: AppContext) -> dict:
    await refresh_runtime_state(ctx)
    state = await ctx.mission_state.snapshot()
    return {"success": True, "state": state.model_dump()}


async def launch_status(ctx: AppContext) -> dict:
    status = await refresh_runtime_state(ctx)
    return status


async def launch_logs(
    ctx: AppContext,
    *,
    lines: int = 200,
    offset: int | None = None,
    inode: int | None = None,
) -> dict:
    try:
        status = await refresh_runtime_state(ctx)
        paths = ctx.config.mission_paths()

        if offset is not None:
            start = max(0, int(offset))
            inode_value = max(0, int(inode or 0))
            cmd = f"""
                log_file={ctx.ssh.q(paths["log"])}
                start={start}
                req_inode={inode_value}

                if [ ! -f "$log_file" ]; then
                    echo "__META__:0:0:0:1"
                    exit 0
                fi

                inode_now="$(stat -c %i "$log_file" 2>/dev/null || stat -f %i "$log_file" 2>/dev/null || echo 0)"
                size="$(wc -c < "$log_file" | tr -d ' ')"
                reset=0

                if [ "$req_inode" -gt 0 ] && [ "$inode_now" -ne "$req_inode" ]; then
                    start=0
                    reset=1
                fi

                if [ "$start" -gt "$size" ]; then
                    start=0
                    reset=1
                fi

                echo "__META__:$size:$reset:$inode_now:0"
                if [ "$size" -gt "$start" ]; then
                    tail -c +$((start + 1)) "$log_file"
                fi
            """
            result = await ctx.ssh.run(cmd, timeout=10)
            if result.returncode != 0:
                return {
                    "success": False,
                    "running": status.get("running", False),
                    "error": ctx.ssh.friendly_error(result.stderr),
                }

            out = result.stdout or ""
            first_line, sep, rest = out.partition("\n")
            meta_match = re.match(
                r"^__META__:(\d+):([01]):(\d+):([01])$", first_line.strip()
            )
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
            next_inode = int(meta_match.group(3))

            return {
                "success": True,
                "running": status.get("running", False),
                "logs": rest,
                "next_offset": next_offset,
                "inode": next_inode,
                "reset": reset,
            }

        line_count = 0 if lines <= 0 else max(20, min(lines, 20000))
        cat_cmd = (
            'cat "$log_file"'
            if line_count == 0
            else f'tail -n {line_count} "$log_file"'
        )
        cmd = f"""
            log_file={ctx.ssh.q(paths["log"])}
            if [ -f "$log_file" ]; then
                {cat_cmd}
            fi
        """
        result = await ctx.ssh.run(cmd, timeout=10)
        if result.returncode != 0:
            return {
                "success": False,
                "running": status.get("running", False),
                "error": ctx.ssh.friendly_error(result.stderr),
            }

        return {
            "success": True,
            "running": status.get("running", False),
            "logs": result.stdout,
        }
    except Exception as exc:
        return {
            "success": False,
            "running": False,
            "error": ctx.ssh.friendly_error(str(exc)),
        }


async def stream_terminal(
    ctx: AppContext,
    websocket: WebSocket,
    *,
    offset: int = 0,
    inode: int = 0,
) -> None:
    await websocket.accept()
    current_offset = max(0, offset)
    current_inode = max(0, inode)

    try:
        while True:
            chunk = await launch_logs(
                ctx,
                offset=current_offset,
                inode=current_inode,
            )
            if not chunk.get("success"):
                message = TerminalInfoMessage(
                    type="error",
                    message=chunk.get("error") or "Terminal stream failed",
                )
                await websocket.send_text(message.model_dump_json())
                if _is_offline_error(chunk.get("error") or ""):
                    break
                await asyncio.sleep(0.5)
                continue

            text = chunk.get("logs") or ""
            next_offset = int(chunk.get("next_offset") or current_offset)
            next_inode = int(chunk.get("inode") or current_inode)
            reset = bool(chunk.get("reset"))

            if text or reset:
                payload = TerminalChunkMessage(
                    data=text,
                    next_offset=next_offset,
                    inode=next_inode,
                    reset=reset,
                )
                await websocket.send_text(payload.model_dump_json())

            current_offset = next_offset
            current_inode = next_inode

            with suppress(asyncio.TimeoutError):
                await asyncio.wait_for(websocket.receive_text(), timeout=0.01)

            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        return
    except Exception as exc:
        with suppress(Exception):
            message = TerminalInfoMessage(
                type="error", message=ctx.ssh.friendly_error(str(exc))
            )
            await websocket.send_text(message.model_dump_json())
    finally:
        with suppress(Exception):
            await websocket.close()


async def prepare_mission(ctx: AppContext) -> dict:
    await ctx.mission_state.set(
        phase="preparing",
        launch_state="running",
        running=False,
        message="Starting mission launch",
        error=None,
    )
    try:
        paths = ctx.config.mission_paths()
        cmd = f"""
            set -e
            cd {ctx.ssh.q(ctx.config.remote_dir)}
            source /opt/ros/humble/setup.bash
            source install/setup.bash

            pid_file={ctx.ssh.q(paths["pid"])}
            pgid_file={ctx.ssh.q(paths["pgid"])}
            log_file={ctx.ssh.q(paths["log"])}
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
        result = await ctx.ssh.run(f"bash -lc {ctx.ssh.q(cmd)}", timeout=20)
        stdout = (result.stdout or "").strip()
        stderr = (result.stderr or "").strip()
        combined = "\n".join(part for part in (stdout, stderr) if part)
        match = re.search(r"(STARTED|RESTARTED)\s*:\s*([0-9]+)", combined)
        if match:
            pid = match.group(2)
            await ctx.mission_state.set(
                phase="running",
                launch_state="running",
                running=True,
                pid=pid,
                message="Mission launch running",
                error=None,
            )
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

        if result.returncode != 0:
            error = ctx.ssh.format_remote_error(
                result.stderr or result.stdout, "Prepare mission failed"
            )
            await ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        error = "Prepare mission failed (launch did not start)"
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}
    except Exception as exc:
        error = ctx.ssh.friendly_error(str(exc))
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def stop_mission(ctx: AppContext) -> dict:
    await ctx.mission_state.set(
        phase="stopping",
        launch_state="stopped",
        running=True,
        message="Stopping mission launch",
        error=None,
    )
    try:
        paths = ctx.config.mission_paths()
        cmd = f"""
            pid_file={ctx.ssh.q(paths["pid"])}
            pgid_file={ctx.ssh.q(paths["pgid"])}

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

            kill -TERM "$target" 2>/dev/null || true
            sleep 1
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$target" 2>/dev/null || true
            fi
            rm -f "$pid_file" "$pgid_file"
            echo "STOPPED"
        """
        result = await ctx.ssh.run(f"bash -lc {ctx.ssh.q(cmd)}", timeout=20)
        if result.returncode != 0:
            error = ctx.ssh.format_remote_error(
                result.stderr or result.stdout, "Stop mission failed"
            )
            await ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        out = (result.stdout or "").strip()
        await ctx.mission_state.set(
            phase="idle",
            launch_state="stopped",
            running=False,
            pid=None,
            message="Mission launch stopped",
            error=None,
        )
        if "NOT_RUNNING" in out:
            return {"success": True, "output": "Mission launch is not running"}
        return {"success": True, "output": "Mission launch stopped"}
    except Exception as exc:
        error = ctx.ssh.friendly_error(str(exc))
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def start_mission(ctx: AppContext) -> dict:
    try:
        cmd = (
            f"cd {ctx.ssh.q(ctx.config.remote_dir)} && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            'ros2 service call /mode_manager/start_mission std_srvs/srv/Trigger "{}"'
        )
        result = await ctx.ssh.run(f"bash -lc {ctx.ssh.q(cmd)}", timeout=20)
        if result.returncode != 0:
            error = ctx.ssh.format_remote_error(
                result.stderr or result.stdout, "Start mission failed"
            )
            await ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        await refresh_runtime_state(ctx)
        return {
            "success": True,
            "output": result.stdout.strip() or "Start mission service called",
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Start mission service call timed out"}
    except Exception as exc:
        error = ctx.ssh.friendly_error(str(exc))
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def trigger_failsafe(ctx: AppContext) -> dict:
    """Trigger failsafe via the daemon's Unix socket (fast) or fallback to ros2 service call."""
    socket_path = "/tmp/penn_failsafe"
    try:
        # Fast path: daemon listening on Unix socket (mission must be running)
        cmd = f"echo 1 | socat - UNIX-CONNECT:{ctx.ssh.q(socket_path)}"
        t0 = time.perf_counter()
        result = await ctx.ssh.run(cmd, timeout=5)
        elapsed = time.perf_counter() - t0
        if result.returncode == 0:
            print(f"[Failsafe] daemon took {elapsed:.2f} s", file=sys.stderr, flush=True)
            return {"success": True, "output": f"Failsafe triggered ({elapsed:.2f} s)"}
        # Fallback: daemon not running, use ros2 service call
        fallback_cmd = (
            f"cd {ctx.ssh.q(ctx.config.remote_dir)} && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            'ros2 service call /mode_manager/failsafe std_srvs/srv/Trigger "{}"'
        )
        result = await ctx.ssh.run(f"bash -lc {ctx.ssh.q(fallback_cmd)}", timeout=15)
        elapsed = time.perf_counter() - t0
        if result.returncode not in (0, 124):
            return {
                "success": False,
                "error": ctx.ssh.format_remote_error(
                    result.stderr, "Failsafe command failed"
                ),
            }
        print(f"[Failsafe] fallback took {elapsed:.2f} s", file=sys.stderr, flush=True)
        return {"success": True, "output": f"Failsafe triggered ({elapsed:.2f} s)"}
    except subprocess.TimeoutExpired:
        return {"success": False, "error": ctx.ssh.friendly_timeout()}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}


async def get_launch_params(ctx: AppContext) -> dict:
    try:
        params_path = ctx.config.mission_paths()["launch_params"]
        result = await ctx.ssh.run(f"cat {ctx.ssh.q(params_path)}", timeout=10)
        if result.returncode != 0:
            return {
                "success": False,
                "error": ctx.ssh.format_remote_error(
                    result.stderr, "Read launch params failed"
                ),
            }
        return {"success": True, "content": result.stdout}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}


async def list_mission_names(ctx: AppContext) -> dict:
    try:
        missions_dir = ctx.config.mission_paths()["missions_dir"]
        cmd = f"""
            missions_dir={ctx.ssh.q(missions_dir)}
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
        result = await ctx.ssh.run(cmd, timeout=10)
        if result.returncode != 0:
            return {
                "success": False,
                "missions": [],
                "error": ctx.ssh.format_remote_error(
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
            "error": ctx.ssh.friendly_error(str(exc)),
        }


async def _resolve_mission_file_path(
    ctx: AppContext, mission_name: str, *, allow_create: bool
) -> dict:
    missions_dir = ctx.config.mission_paths()["missions_dir"]
    cmd = f"""
        missions_dir={ctx.ssh.q(missions_dir)}
        mission_name={ctx.ssh.q(mission_name)}
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
    result = await ctx.ssh.run(cmd, timeout=10)
    if result.returncode != 0:
        return {
            "success": False,
            "error": ctx.ssh.format_remote_error(
                result.stderr, "Resolve mission file failed"
            ),
        }

    resolved = (result.stdout or "").strip()
    if not resolved or resolved == "__ERR__:not_found":
        return {
            "success": False,
            "error": (
                f"Mission file '{mission_name}.yaml' was not found in "
                f"{missions_dir}. Select a valid mission in launch params."
            ),
        }

    return {"success": True, "path": resolved}


async def get_mission_file(ctx: AppContext, *, name: str) -> dict:
    try:
        mission_name = _normalize_mission_name(name)
    except ValueError as exc:
        return {"success": False, "mission": None, "error": str(exc)}

    try:
        resolved = await _resolve_mission_file_path(
            ctx, mission_name, allow_create=False
        )
        if not resolved.get("success"):
            return {
                "success": False,
                "mission": mission_name,
                "error": resolved.get("error", "Mission file not found"),
            }

        mission_path = resolved["path"]
        result = await ctx.ssh.run(f"cat {ctx.ssh.q(mission_path)}", timeout=10)
        if result.returncode != 0:
            return {
                "success": False,
                "mission": mission_name,
                "path": mission_path,
                "error": ctx.ssh.format_remote_error(
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
            "error": ctx.ssh.friendly_error(str(exc)),
        }


async def set_mission_file(ctx: AppContext, *, name: str, content: str) -> dict:
    tmp_path = ""
    try:
        mission_name = _normalize_mission_name(name)
    except ValueError as exc:
        return {"success": False, "mission": None, "error": str(exc)}

    try:
        missions_dir = ctx.config.mission_paths()["missions_dir"]
        mkdir_result = await ctx.ssh.run(
            f"mkdir -p {ctx.ssh.q(missions_dir)}", timeout=10
        )
        if mkdir_result.returncode != 0:
            return {
                "success": False,
                "mission": mission_name,
                "error": ctx.ssh.format_remote_error(
                    mkdir_result.stderr,
                    "Write mission YAML failed",
                ),
            }

        resolved = await _resolve_mission_file_path(
            ctx, mission_name, allow_create=True
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

        scp_result = await ctx.ssh.scp(tmp_path, mission_path, timeout=30)
        if scp_result.returncode != 0:
            return {
                "success": False,
                "mission": mission_name,
                "path": mission_path,
                "error": ctx.ssh.format_remote_error(
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
            "error": ctx.ssh.friendly_error(str(exc)),
        }
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


async def set_launch_params(ctx: AppContext, content: str) -> dict:
    tmp_path = ""
    try:
        params_path = ctx.config.mission_paths()["launch_params"]
        mkdir_result = await ctx.ssh.run(
            f"mkdir -p {ctx.ssh.q(str(Path(params_path).parent))}", timeout=10
        )
        if mkdir_result.returncode != 0:
            return {
                "success": False,
                "error": ctx.ssh.format_remote_error(
                    mkdir_result.stderr,
                    "Write launch params failed",
                ),
            }

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name

        result = await ctx.ssh.scp(tmp_path, params_path, timeout=30)
        if result.returncode != 0:
            return {
                "success": False,
                "error": ctx.ssh.format_remote_error(
                    result.stderr, "Write launch params failed"
                ),
            }

        return {"success": True, "output": "launch_params.yaml updated"}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)
