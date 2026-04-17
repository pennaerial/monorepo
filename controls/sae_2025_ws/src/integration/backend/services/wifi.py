from __future__ import annotations

import asyncio
import json
import re
import subprocess

from ..context import AppContext, TargetContext


def _target_context(
    ctx: AppContext,
    *,
    target_ctx: TargetContext | None = None,
    target_id: str | None = None,
) -> TargetContext:
    if target_ctx is not None:
        return target_ctx
    return ctx.resolve_target(target_id)


async def _policy_status(session) -> dict:
    try:
        result = await session.ssh.run(
            (
                "if command -v pennair-wifi-failover >/dev/null 2>&1; then "
                "sudo -n pennair-wifi-failover --status 2>/dev/null || "
                "pennair-wifi-failover --status 2>/dev/null; "
                "fi"
            ),
            timeout=15,
        )
        if result.returncode != 0:
            return {}
        raw = (result.stdout or "").strip()
        if not raw:
            return {}
        parsed = json.loads(raw)
        return parsed if isinstance(parsed, dict) else {}
    except Exception:
        return {}


async def wifi_status(
    ctx: AppContext,
    *,
    target_ctx: TargetContext | None = None,
    target_id: str | None = None,
) -> dict:
    session = _target_context(ctx, target_ctx=target_ctx, target_id=target_id)
    try:
        policy_status = await _policy_status(session)
        result = await session.ssh.run(
            "nmcli -f NAME,TYPE,DEVICE,STATE con show --active | tail -n +2",
            timeout=15,
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": session.ssh.friendly_error(result.stderr),
            }

        connections = []
        for line in result.stdout.strip().split("\n"):
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

        legacy_hotspot = any(
            c["name"] == ctx.operator_config.hotspot_name for c in connections
        )
        wifi_con = next(
            (c for c in connections if c["type"] == "802-11-wireless"), None
        )
        return {
            "success": True,
            "is_hotspot": policy_status.get("is_hotspot", legacy_hotspot),
            "current_wifi": policy_status.get("current_wifi")
            or (wifi_con["name"] if wifi_con else None),
            "current_mode": policy_status.get("current_mode"),
            "effective_role": policy_status.get("effective_role"),
            "policy_source": policy_status.get("policy_source"),
            "runtime_active": policy_status.get("runtime_active"),
            "mission_started": policy_status.get("mission_started"),
            "travel_router_locked": policy_status.get("travel_router_locked"),
            "switching_disabled": policy_status.get("switching_disabled"),
            "local_ap_profile": policy_status.get("local_ap_profile"),
            "travel_router_profile": policy_status.get("travel_router_profile"),
            "allowed_ap_hosts": policy_status.get("allowed_ap_hosts") or [],
            "connections": connections,
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": session.ssh.friendly_timeout()}
    except Exception as exc:
        return {"success": False, "error": session.ssh.friendly_error(str(exc))}


async def wifi_scan(
    ctx: AppContext,
    *,
    target_ctx: TargetContext | None = None,
    target_id: str | None = None,
) -> dict:
    session = _target_context(ctx, target_ctx=target_ctx, target_id=target_id)
    try:
        result = await session.ssh.run(
            r"nmcli -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes | tail -n +2",
            timeout=20,
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": session.ssh.friendly_error(result.stderr),
                "networks": [],
            }

        networks = []
        for line in result.stdout.strip().split("\n"):
            if not line.strip():
                continue
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
                networks.append({"ssid": ssid, "signal": signal, "security": security})

        seen: dict[str, dict] = {}
        for network in networks:
            if (
                network["ssid"] not in seen
                or network["signal"] > seen[network["ssid"]]["signal"]
            ):
                seen[network["ssid"]] = network

        return {
            "success": True,
            "networks": sorted(seen.values(), key=lambda n: -n["signal"]),
        }
    except Exception as exc:
        return {
            "success": False,
            "error": session.ssh.friendly_error(str(exc)),
            "networks": [],
        }


async def wifi_connect(
    ctx: AppContext,
    *,
    target_ctx: TargetContext | None = None,
    target_id: str | None = None,
    ssid: str,
    password: str,
) -> dict:
    session = _target_context(ctx, target_ctx=target_ctx, target_id=target_id)
    policy_status = await _policy_status(session)
    hotspot_name = session.ssh.q(
        str(policy_status.get("local_ap_profile") or ctx.operator_config.hotspot_name)
    )
    try:
        await session.ssh.run(
            f"nmcli con down {hotspot_name} 2>/dev/null; sleep 2", timeout=15
        )
        pwd_arg = f" password {session.ssh.q(password)}" if password else ""
        result = await session.ssh.run(
            f"nmcli dev wifi connect {session.ssh.q(ssid)}{pwd_arg}", timeout=30
        )
        if result.returncode != 0:
            await session.ssh.run(f"nmcli con up {hotspot_name}", timeout=15)
            return {
                "success": False,
                "error": session.ssh.format_remote_error(
                    result.stderr, "Failed to connect"
                ),
            }

        return {"success": True, "output": f"Pi connected to {ssid}"}
    except Exception as exc:
        await session.ssh.run(f"nmcli con up {hotspot_name}", timeout=15)
        return {"success": False, "error": session.ssh.friendly_error(str(exc))}


async def wifi_hotspot(
    ctx: AppContext,
    *,
    target_ctx: TargetContext | None = None,
    target_id: str | None = None,
) -> dict:
    session = _target_context(ctx, target_ctx=target_ctx, target_id=target_id)
    policy_status = await _policy_status(session)
    hotspot_name = str(
        policy_status.get("local_ap_profile") or ctx.operator_config.hotspot_name
    )
    try:
        await session.ssh.run(
            "nmcli dev disconnect wlan0 2>/dev/null; sleep 1", timeout=15
        )
        result = await session.ssh.run(
            f"nmcli con up {session.ssh.q(hotspot_name)}",
            timeout=15,
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": session.ssh.friendly_error(result.stderr),
            }
        return {"success": True, "output": "Hotspot activated"}
    except Exception as exc:
        return {"success": False, "error": session.ssh.friendly_error(str(exc))}


async def switch_local_wifi(ssid: str, password: str) -> dict:
    try:
        cmd = ["networksetup", "-setairportnetwork", "en0", ssid]
        if password:
            cmd.append(password)
        result = await asyncio.to_thread(
            subprocess.run,
            cmd,
            capture_output=True,
            text=True,
            timeout=15,
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": result.stderr.strip() or result.stdout.strip(),
            }
        return {"success": True, "output": f"Mac connected to {ssid}"}
    except Exception as exc:
        return {"success": False, "error": str(exc)}
