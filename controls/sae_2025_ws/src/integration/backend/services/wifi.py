from __future__ import annotations

import asyncio
import re
import subprocess

from ..context import AppContext


async def wifi_status(ctx: AppContext) -> dict:
    try:
        result = await ctx.ssh.run(
            "nmcli -f NAME,TYPE,DEVICE,STATE con show --active | tail -n +2",
            timeout=15,
        )
        if result.returncode != 0:
            return {"success": False, "error": ctx.ssh.friendly_error(result.stderr)}

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

        is_hotspot = any(c["name"] == ctx.config.hotspot_name for c in connections)
        wifi_con = next((c for c in connections if c["type"] == "802-11-wireless"), None)
        return {
            "success": True,
            "is_hotspot": is_hotspot,
            "current_wifi": wifi_con["name"] if wifi_con else None,
            "connections": connections,
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": ctx.ssh.friendly_timeout()}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}


async def wifi_scan(ctx: AppContext) -> dict:
    try:
        result = await ctx.ssh.run(
            r"nmcli -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes | tail -n +2",
            timeout=20,
        )
        if result.returncode != 0:
            return {
                "success": False,
                "error": ctx.ssh.friendly_error(result.stderr),
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
            "error": ctx.ssh.friendly_error(str(exc)),
            "networks": [],
        }


async def wifi_connect(ctx: AppContext, ssid: str, password: str) -> dict:
    hotspot_name = ctx.ssh.q(ctx.config.hotspot_name)
    try:
        await ctx.ssh.run(f"nmcli con down {hotspot_name} 2>/dev/null; sleep 2", timeout=15)
        pwd_arg = f" password {ctx.ssh.q(password)}" if password else ""
        result = await ctx.ssh.run(
            f"nmcli dev wifi connect {ctx.ssh.q(ssid)}{pwd_arg}", timeout=30
        )
        if result.returncode != 0:
            await ctx.ssh.run(f"nmcli con up {hotspot_name}", timeout=15)
            return {
                "success": False,
                "error": ctx.ssh.format_remote_error(result.stderr, "Failed to connect"),
            }

        return {"success": True, "output": f"Pi connected to {ssid}"}
    except Exception as exc:
        await ctx.ssh.run(f"nmcli con up {hotspot_name}", timeout=15)
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}


async def wifi_hotspot(ctx: AppContext) -> dict:
    try:
        await ctx.ssh.run("nmcli dev disconnect wlan0 2>/dev/null; sleep 1", timeout=15)
        result = await ctx.ssh.run(
            f"nmcli con up {ctx.ssh.q(ctx.config.hotspot_name)}", timeout=15
        )
        if result.returncode != 0:
            return {"success": False, "error": ctx.ssh.friendly_error(result.stderr)}
        return {"success": True, "output": "Hotspot activated"}
    except Exception as exc:
        return {"success": False, "error": ctx.ssh.friendly_error(str(exc))}


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
